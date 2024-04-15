#include <ODriveArduino.h>
#include <ODriveEnums.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin setup
// INPUTS
#define spinpin 27
#define shootpin 26
#define uibutton 23
#define uiscrollpulse 25
#define uiscrolldir 24
// OUTPUTS
#define beltpin 8
#define flywheelpin 7
#define solenoidpin 0

#define INVALID -9999999

// Hardware parameters
static float drum_gear_ratio = 3.0;
static int drum_mags = 9;

// Operation parameters
static int belts_off_pwm = 1000;
static int belts_prime_pwm = 1280;  // was 1150
static int belts_shoot_pwm = 1430;  // was 1250
static int fwheel_off_pwm = 1000;
static int fwheel_prime_pwm = 1500;
static int fwheel_shoot_pwm = 1900;
static int default_rps = 80;
static int max_fire_rps = 101;
static int debounce_delay = 20;  // Increase if being weird
static int edit_blink_frames = 20;

// Interrupt stuff
static bool buttonPressed = false;
static bool scrollUpPressed = false;
static bool scrollDnPressed = false;

// Operational State machine parameters
const int INITIALIZING = 0;
const int INITIALIZED = 1;
const int SPINNING_UP = 2;
const int READY_TO_SHOOT = 3;
const int SHOOTING = 4;
const int SPINNING_DOWN = 5;
const int ERRORSTATE = 6;

int op_state = INITIALIZING;
int last_state = op_state;
bool statechange = false;

// UI State machine parameters
const int SELECTING = 0;
const int EDITING = 1;

int ui_state = EDITING;

// UI Fields
const int FIRE_MODE = 0;
const int FIRE_RPS = 1;

int ui_field = FIRE_RPS;

// Fire Modes
const int SINGLE = 0;
const int BURST = 1;
const int AUTO = 2;

int fire_mode = BURST;
int rounds_per_second = 100;

// Code and stuff
void setup() {
  pinMode(spinpin, INPUT);
  pinMode(shootpin, INPUT);
  pinMode(solenoidpin, OUTPUT);
  digitalWrite(solenoidpin, LOW);

  // pinMode(uibutton, INPUT_PULLUP);
  // pinMode(uiscrollpulse, INPUT);
  // pinMode(uiscrolldir, INPUT);

  // attachInterrupt(uibutton, UI_buttonHandler, FALLING);
  // attachInterrupt(uiscrollpulse, UI_scrollHandler, CHANGE);
}

void arm_servos(Servo myservos[]) {
  // Janky but it kinda works?
  for (int i = 0; i < sizeof(myservos) / sizeof(myservos[0]); i++) {
    myservos[i].writeMicroseconds(1000);
  }
  delay(3000);
  ramp_servos(myservos, 1000, 1400, 0.5);
  delay(2000);
  ramp_servos(myservos, 1400, 1000, 0.5);
  delay(2000);
}

float get_odrive_vel() {
  Serial1.flush();
  Serial1.println("f 0");
  String str = "";
  static const unsigned long timeout = 1000;
  unsigned long timeout_start = millis();
  float vel, pos;
  for (;;) {
    while (!Serial1.available()) {
      if (millis() - timeout_start >= timeout) {
        Serial.println("TIMEOUT");
        break;
      }
    }
    char c = Serial1.read();
    if (c == ' ') {
      if (str == "unknown")
        pos = INVALID;
      else
        pos = str.toFloat();
      str = "";
    } else if (c == '\n') {
      if (pos == INVALID)
        vel = INVALID;
      else
        vel = str.toFloat();
      break;
    } else {
      str += c;
    }
  }

  return vel;
}

void display_rps() {
  if (rounds_per_second < 10)
    display.setCursor(26, 10);
  else if (rounds_per_second < 100)
    display.setCursor(20, 10);
  else
    display.setCursor(14, 10);
  display.print(rounds_per_second);
}

void display_mode() {
  display.setCursor(72, 10);
  if (fire_mode == SINGLE)
    display.print("SNGL");
  else if (fire_mode == BURST)
    display.print("BRST");
  else if (fire_mode == AUTO)
    display.print("AUTO");
}

void clear_inputs() {
  buttonPressed = false;
  scrollUpPressed = false;
  scrollDnPressed = false;
}

void ramp_servo(Servo myservo, int start_us, int end_us, float ramptime_s) {
  if (start_us == end_us) return;
  int delay_us = (int)(ramptime_s * 1000000 / abs(start_us - end_us));
  int step_now = start_us;
  int sign = start_us < end_us ? 1 : -1;
  while (step_now != end_us) {
    myservo.writeMicroseconds(step_now);
    step_now += sign;
    delayMicroseconds(delay_us);
  }
}

void ramp_servos(Servo myservos[], int start_us, int end_us, float ramptime_s) {
  if (start_us == end_us) return;
  int delay_us = (int)(1000000 / abs(start_us - end_us));
  int step_now = start_us;
  int sign = start_us < end_us ? 1 : -1;
  while (step_now != end_us) {
    for (int i = 0; i < sizeof(myservos) / sizeof(myservos[0]); i++) {
      myservos[i].writeMicroseconds(step_now);
    }
    step_now += sign;
    delayMicroseconds(delay_us);
  }
}

void debug_spin() {
  Servo belts, flywheels;
  belts.attach(beltpin, 1000, 2000);
  flywheels.attach(flywheelpin, 1000, 2000);

  Servo servos[] = {belts, flywheels};
  arm_servos(servos);

  belts.writeMicroseconds(1000);
  flywheels.writeMicroseconds(1000);
  delay(2000);
  ramp_servo(belts, 1000, 1300, 0.3);
  delay(1000);
  ramp_servo(belts, 1300, 1000, 0.1);
}

void debug_manual() {
  int allpisn[4] = {7,8,9,10};
  for (auto &p : allpisn) {
    // pinMode(p, OUTPUT);
    // digitalWrite(p, HIGH);
    // delay(100);
    // digitalWrite(p, LOW);
    Servo s;
    s.attach(p, 1000, 2000);
    s.writeMicroseconds(1200);
    delay(100);
  }
  delay(10000);
  return;


  Servo belts, flywheels;
  belts.attach(beltpin, 1000, 2000);
  flywheels.attach(flywheelpin, 1000, 2000);
  belts.writeMicroseconds(1000);
  flywheels.writeMicroseconds(1000);

  while(true) {
    if (Serial.available() > 0) {
        int data = Serial.parseInt();
        Serial.println(data);
        belts.writeMicroseconds(data);
        flywheels.writeMicroseconds(data);
    }
  }
}

void loop() {

  // delay(500);

  // TESTING STUFF
  debug_spin();
  while (true) {};

  // Set up stuff
  uint32_t frame = 0;
  bool editblink = true;
  float drum_target_vel = -(default_rps / drum_mags) * drum_gear_ratio;
  // Servo m0, m1, m2, m3;

  Serial.begin(115200);
  Serial1.begin(115200);

  ODriveArduino odrive(Serial1);

  // Wait for serial to begin
  //  while(!Serial);
  delay(1000);  // Needed to work but idk how long is necessary

  // ODrive Configuration
  Serial1.println("w axis0.controller.config.vel_limit 40.0");
  Serial1.println("w axis0.controller.config.vel_ramp_rate 25.0");
  Serial1.println("w axis0.motor.config.current_lim 40.0");

  // Run the looooop
  while (true) {
    switch (op_state) {
      case INITIALIZING:
        {
          // Start display first so status can be displayed
          Serial.println("DISPLAY: Initializing");
          if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            op_state = ERRORSTATE;
            break;
          }
          display.clearDisplay();
          display.setTextSize(1);               // Normal 1:1 pixel scale
          display.setTextColor(SSD1306_WHITE);  // Draw white text
          display.setCursor(0, 0);              // Start at top-left corner
          display.cp437(true);                  // Use full 256 char 'Code Page 437' font

          // m0.attach(beltpin0, 1000, 2000);
          // m1.attach(beltpin1, 1000, 2000);
          // m2.attach(flywheelpin0, 1000, 2000);
          // m3.attach(flywheelpin1, 1000, 2000);

          Serial.println("BLDC: Arming ESCs");
          display.println(F("BLDC: Arming ESCs"));
          display.display();
          // arm_servo(m0);
          // arm_servo(m1);
          // arm_servo(m2);
          // arm_servo(m3);
          Serial.println("BLDC: ESCs Armed");

          Serial.println("ODRIVE: Starting Calibration");
          delay(5000);
          display.println(F("ODRIVE: Starting Calibration"));
          display.display();
          // if(!odrive.run_state(0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true, 25.0f)){
          //   op_state = ERRORSTATE;
          //   break;
          // }
          Serial.println("ODRIVE: Entering Closed-Loop Control");
          delay(5000);
          display.println(F("ODRIVE: Entering Closed-Loop Control"));
          display.display();
          // if(!odrive.run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL, false /*don't wait*/)){
          //   op_state = ERRORSTATE;
          //   break;
          // }

          display.clearDisplay();
          display.setCursor(0, 0);
          display.println(F("Initialization\nComplete."));
          display.display();
          delay(500);
          op_state = INITIALIZED;
          break;
        }
      case INITIALIZED:
        {
          if (!digitalRead(spinpin)) {
            delay(debounce_delay);
            op_state = SPINNING_UP;
          }

          // Update display
          // size 1 - 21 char wide, 6 pixels
          // size 2 - 10 char wide, 12 pixels

          // Static stuff first
          display.clearDisplay();
          display.setTextSize(1);
          display.setCursor(23, 0);
          display.print("rps");
          display.setCursor(84, 0);
          display.print("mode");
          display.setTextSize(2);
          display.drawFastVLine(62, 2, 28, SSD1306_WHITE);

          if (ui_state == SELECTING) {
            // Handle inputs when in this state
            if (buttonPressed)
              ui_state = EDITING;
            if (scrollUpPressed)
              ui_field = (ui_field + 1) % 2;
            if (scrollDnPressed)
              ui_field = (ui_field - 1) % 2;
            clear_inputs();

            // Update display stuff
            display_rps();
            display_mode();
            if (ui_field == FIRE_RPS)
              display.drawFastHLine(14, 30, 36, SSD1306_WHITE);
            else if (ui_field == FIRE_MODE)
              display.drawFastHLine(78, 30, 36, SSD1306_WHITE);
          } else if (ui_state == EDITING) {
            // Handle inputs when in this state
            if (buttonPressed)
              ui_state = SELECTING;
            if (scrollUpPressed) {
              if (ui_field == FIRE_RPS)
                rounds_per_second = (rounds_per_second + 1) % max_fire_rps;
              else if (ui_field == FIRE_MODE)
                fire_mode = (fire_mode + 1) % 3;
            }
            if (scrollDnPressed) {
              if (ui_field == FIRE_RPS)
                rounds_per_second = (rounds_per_second - 1) % max_fire_rps;
              else if (ui_field == FIRE_MODE)
                fire_mode = (fire_mode - 1) % 3;
            }
            clear_inputs();

            // Update display stuff
            if (frame % edit_blink_frames == 0)
              editblink = !editblink;
            if (ui_field == FIRE_RPS) {
              if (editblink)
                display_rps();
              display_mode();
            } else if (ui_field == FIRE_MODE) {
              display_rps();
              if (editblink)
                display_mode();
            }
          }

          display.display();

          break;
        }
      case SPINNING_UP:
        {
          // TODO delete - Demo mode only
          op_state = READY_TO_SHOOT;
          break;

          if (statechange) {
            // m0.writeMicroseconds(belts_prime_pwm);
            // m1.writeMicroseconds(belts_prime_pwm);
            // m2.writeMicroseconds(fwheel_prime_pwm);
            // m3.writeMicroseconds(fwheel_prime_pwm);
            // odrive.SetVelocity(0, drum_target_vel);
          }

          // No more spin up
          if (digitalRead(spinpin)) {
            op_state = SPINNING_DOWN;
            delay(debounce_delay);
          }

          float drum_vel = get_odrive_vel();
          if (drum_vel == INVALID)
            break;
          Serial.println(drum_vel);
          if (abs(drum_vel - drum_target_vel) < 0.1)
            op_state = READY_TO_SHOOT;
          break;
        }
      case READY_TO_SHOOT:
        {
          if (statechange) {
            // m0.writeMicroseconds(belts_prime_pwm);
            // m1.writeMicroseconds(belts_prime_pwm);
            // m2.writeMicroseconds(fwheel_prime_pwm);
            // m3.writeMicroseconds(fwheel_prime_pwm);
            // odrive.SetVelocity(0, drum_target_vel);
            // digitalWrite(solenoidpin, LOW);
          }

          // No more spin up
          if (digitalRead(spinpin)) {
            op_state = SPINNING_DOWN;
            delay(debounce_delay);
          }

          // shoot time
          if (!digitalRead(shootpin)) {
            op_state = SHOOTING;
            delay(debounce_delay);
          }
          break;
        }
      case SHOOTING:
        {
          if (statechange) {
            // m0.writeMicroseconds(belts_shoot_pwm);
            // m1.writeMicroseconds(belts_shoot_pwm);
            // m2.writeMicroseconds(fwheel_shoot_pwm);
            // m3.writeMicroseconds(fwheel_shoot_pwm);
            // odrive.SetVelocity(0, drum_target_vel);
            // digitalWrite(solenoidpin, HIGH);
          }

          if (digitalRead(shootpin)) {
            // odrive.SetVelocity(0, 0);
            op_state = READY_TO_SHOOT;
            delay(debounce_delay);
          }
          break;
        }
      case SPINNING_DOWN:
        {
          op_state = INITIALIZED;
          break;
          if (statechange) {
            // m0.writeMicroseconds(belts_off_pwm);
            // m1.writeMicroseconds(belts_off_pwm);
            // m2.writeMicroseconds(fwheel_off_pwm);
            // m3.writeMicroseconds(fwheel_off_pwm);
            // odrive.SetVelocity(0, 0.0);
          }

          // Spin back up if button pressed
          if (!digitalRead(spinpin)) {
            delay(debounce_delay);
            op_state = SPINNING_UP;
          }

          // Get drum vel
          float drum_vel = get_odrive_vel();
          if (drum_vel == INVALID)
            break;
          if (drum_vel < 0.1)
            op_state = INITIALIZED;
          break;
        }
      case ERRORSTATE:
        {
          if (statechange) {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.setTextSize(3);
            display.println(F("ERROR"));
            display.display();
            display.startscrollleft(0x00, 0x0F);
          }
          digitalWrite(LED_BUILTIN, HIGH);
          delay(100);
          digitalWrite(LED_BUILTIN, LOW);
          delay(100);
          break;
        }
    }
    statechange = (last_state != op_state);
    last_state = op_state;
    Serial.print("STATE: ");
    Serial.println(op_state);

    frame += 1;
    delay(10);
  }
}

void UI_buttonHandler() {
  delay(5);
  if (!digitalRead(uibutton)) {
    Serial.println("BUTTON");
    buttonPressed = true;
  }
}

void UI_scrollHandler() {
  delay(5);
  if (digitalRead(uiscrolldir) == digitalRead(uiscrollpulse)) {
    Serial.println("SCROOLL UP");
    scrollUpPressed = true;
  } else {
    Serial.println("SCROOLL DN");
    scrollDnPressed - true;
  }
}
