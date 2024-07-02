#include <ODriveArduino.h>
#include <ODriveEnums.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

// SoftwareSerial odrive_serial(8, 9);

// SDA pin 18, SCL pin 19
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin setup
// INPUTS
#define spinpin 16
#define shootpin 17
#define magbutton 14
#define uibutton 20
#define uiscrollpulse 21

// OUTPUTS
#define beltpin0 6
#define beltpin1 8
#define fwheelpin0 5
#define fwheelpin1 7
#define solenoidpin 15
#define magservopin 9

#define INVALID -9999999

// Hardware parameters
const float drum_gear_ratio = 3.0;
const int drum_mags = 9;
const int rounds_per_mag = 8;

// Operation parameters
const int belts_off_pwm = 1000;
const int belts_prime_pwm = 1470; //1175;
const int belts_shoot_pwm = 1600; //1200;
const int fwheel_off_pwm = 1000;
const int fwheel_prime_pwm = 1600;
const int fwheel_shoot_pwm = 1600;
const int rps_inc_size = 10;
const int max_fire_rps = 130;
const int debounce_delay = 20;  // Increase if being weird
const int edit_blink_frames = 10;
const int mag_servo_release_angle = 50;
const int mag_servo_engage_angle = 175;
const int mag_engage_time_ms = 500;
const int solenoid_engage_time_ms = 3; // Extra time added or subtracted to single/burst round timings
const int burst_qty = 5;

// Interrupt stuff
static bool buttonPressed = false;
static bool cyclePressed = false;

// Operational State machine parameters
enum op_state_t {
  INITIALIZING = 0,
  INITIALIZED,
  SPINNING_UP,
  READY_TO_SHOOT,
  SHOOTING,
  NOT_SHOOTING,
  SPINNING_DOWN,
  MAG_RELEASED,
  MAG_ENGAGING,
  ERRORSTATE
};

op_state_t op_state = INITIALIZING;
op_state_t last_state = op_state;
bool statechange = false;
static String error_msg = "";

// UI State machine parameters
enum ui_state_t {
  SELECTING = 0,
  EDITING
};

ui_state_t ui_state = SELECTING;

// UI Fields
enum ui_field_t {
  FIRE_MODE = 0,
  FIRE_RPS
};

ui_field_t ui_field = FIRE_RPS;

// Fire Modes
enum ui_mode_t {
  SINGLE = 0,
  BURST,
  AUTO
};

ui_mode_t fire_mode = AUTO;
int rounds_per_second = 20;

// Code and stuff
void setup() {
  pinMode(spinpin, INPUT_PULLUP);
  pinMode(shootpin, INPUT_PULLUP);
  pinMode(magbutton, INPUT_PULLUP);
  pinMode(solenoidpin, OUTPUT);
  digitalWriteFast(solenoidpin, LOW);

  pinMode(uibutton, INPUT_PULLUP);
  pinMode(uiscrollpulse, INPUT_PULLUP);

  attachInterrupt(uibutton, UI_buttonHandler, FALLING);
  attachInterrupt(uiscrollpulse, UI_scrollHandler, CHANGE);
}

float get_odrive_vel() {
  Serial1.flush();
  Serial1.println("f 0");
  String str = "";
  static const unsigned long timeout = 1000;
  unsigned long timeout_start = millis();
  float vel = INVALID, pos = INVALID;
  for (;;) {
    while (!Serial1.available()) {
      if (millis() - timeout_start >= timeout) {
        Serial.println("TIMEOUT");
        return INVALID;
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

void display_rounds_remaining(int rounds_remaining) {
  if (rounds_remaining < 10)
    display.setCursor(56, 0);
  else if (rounds_remaining < 100)
    display.setCursor(50, 0);
  else
    display.setCursor(44, 0);
  display.setTextSize(3);
  display.print(rounds_remaining);

  display.setCursor(20, 24);
  display.setTextSize(1);
  display.print("Rounds Remaining");
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
  cyclePressed = false;
}

bool write_esc_values(Servo s0, Servo s1, uint16_t goal_pwm) {
  uint16_t pwm_now = s0.readMicroseconds();
  if (pwm_now == goal_pwm) {
    return true;
  } else if (pwm_now < goal_pwm) {
    s0.writeMicroseconds(pwm_now + 1);
    s1.writeMicroseconds(pwm_now + 1);
  } else if (pwm_now > goal_pwm) {
    s0.writeMicroseconds(pwm_now - 1);
    s1.writeMicroseconds(pwm_now - 1);
  }
  return false;
}

void set_escs(Servo b0, Servo b1, Servo f0, Servo f1, uint16_t bval_pwm, uint16_t fval_pwm, float rate_pwm_per_s = 0) {
  // Unspecified ramp rate will set instant values
  if (rate_pwm_per_s == 0) {
    b0.writeMicroseconds(bval_pwm);
    b1.writeMicroseconds(bval_pwm);
    f0.writeMicroseconds(fval_pwm);
    f1.writeMicroseconds(fval_pwm);
    return;
  }

  // Otherwise, ramp the profiles
  uint32_t dwell_us = 1000000 / rate_pwm_per_s;
  bool b_done = false, f_done = false;
  while (!b_done || !f_done) {
    f_done = write_esc_values(f0, f1, fval_pwm);
    b_done = write_esc_values(b0, b1, bval_pwm);
    delayMicroseconds(dwell_us);
  }
}

void loop() {
  // Set up stuff
  uint32_t frame = 0;
  int invalid_vel_count = 0;
  int rounds_remaining = drum_mags * rounds_per_mag;
  bool editblink = true;
  float drum_target_vel = -(rounds_per_second / drum_mags) * drum_gear_ratio;
  uint32_t ms_per_round = 0;
  uint16_t rounds_fired = 0;
  uint16_t rounds_starting = 0;
  uint32_t start_firing_time_ms = 0;
  Servo belt_m0, belt_m1, fwheel_m2, fwheel_m3, magservo;

  Serial.begin(115200);
  Serial1.begin(115200);

  ODriveArduino odrive1(Serial1);
  delay(500);

  // Run the looooop
  while (true) {
    switch (op_state) {
      case INITIALIZING:
        {
          // Start display first so status can be displayed
          Serial.println("DISPLAY: Initializing");
          if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            op_state = ERRORSTATE;
            error_msg = "Display Failure";
            break;
          }
          display.clearDisplay();
          display.setTextSize(1);               // Normal 1:1 pixel scale
          display.setTextColor(SSD1306_WHITE);  // Draw white text
          display.setCursor(0, 0);              // Start at top-left corner
          display.cp437(true);                  // Use full 256 char 'Code Page 437' font

          // Start the ESCs
          belt_m0.attach(beltpin0, 1000, 2000);
          belt_m1.attach(beltpin1, 1000, 2000);
          fwheel_m2.attach(fwheelpin0, 1000, 2000);
          fwheel_m3.attach(fwheelpin1, 1000, 2000);

          Serial.println("BLDC: Arming ESCs");
          display.println(F("BLDC: Arming ESCs"));
          display.display();
          set_escs(belt_m0, belt_m1, fwheel_m2, fwheel_m3, belts_off_pwm, fwheel_off_pwm);
          delay(2500);
          Serial.println("BLDC: ESCs Armed");
          display.println(F("BLDC: ESCs Armed"));
          display.display();

          // ODrive Configuration
          Serial.println("ODRIVE: Initializing");
          while (odrive1.getState() == AXIS_STATE_UNDEFINED) {
            delay(100);
          }
          Serial.println("Found ODrive");
          Serial1.println("w axis0.controller.config.vel_limit 60.0");
          Serial1.println("w axis0.controller.config.vel_ramp_rate 50.0");
          Serial1.println("w axis0.motor.config.current_lim 40.0");

          Serial.println("ODRIVE: Starting Calibration");
          display.println(F("ODRIVE: Starting Calibration"));
          display.display();

          odrive1.setState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
          delay(2000);
          while (odrive1.getState() != AXIS_STATE_IDLE) {
            delay(100);
          }
          Serial.println("Enabling closed loop control...");
          while (odrive1.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
            odrive1.clearErrors();
            odrive1.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
          }

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
          if (!digitalRead(magbutton)) {
            delay(debounce_delay);
            op_state = MAG_RELEASED;
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
            if (cyclePressed)
              ui_field = ui_field == FIRE_RPS ? FIRE_MODE : FIRE_RPS;
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
            if (cyclePressed) {
              if (ui_field == FIRE_RPS) {
                rounds_per_second = (rounds_per_second + rps_inc_size) % max_fire_rps;
                rounds_per_second = rounds_per_second == 0 ? 10 : rounds_per_second;
              } else if (ui_field == FIRE_MODE) {
                switch (fire_mode) {
                  case SINGLE:
                    fire_mode = BURST;
                    break;
                  case BURST:
                    fire_mode = AUTO;
                    break;
                  case AUTO:
                    fire_mode = SINGLE;
                    break;
                }
              }
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
          if (statechange) {
            drum_target_vel = -(rounds_per_second / drum_mags) * drum_gear_ratio;
            ms_per_round = 1000 / rounds_per_second;
            odrive1.setVelocity(drum_target_vel);
            set_escs(belt_m0, belt_m1, fwheel_m2, fwheel_m3, belts_prime_pwm, fwheel_prime_pwm, 2000);

            display.clearDisplay();
            display_rounds_remaining(rounds_remaining);
            display.display();

            // Check for odrive errors
            if (odrive1.getState() == AXIS_STATE_IDLE)
            {
              op_state = ERRORSTATE;
              error_msg = "ODrive Error !";
              break;
            }
          }

          // No more spin up
          if (digitalRead(spinpin)) {
            op_state = SPINNING_DOWN;
            delay(debounce_delay);
          }

          float drum_vel = get_odrive_vel();
          if (drum_vel == INVALID) {
            ++invalid_vel_count;
            if (invalid_vel_count > 10) {
              op_state = ERRORSTATE;
              error_msg = "Invalid ODrive Vel";
            }
          } else
            invalid_vel_count = 0;
          Serial.println(drum_vel);
          if (abs(drum_vel - drum_target_vel) < 0.1)
            op_state = READY_TO_SHOOT;
          break;
        }
      case READY_TO_SHOOT:
        {
          if (statechange) {
            odrive1.setVelocity(drum_target_vel);
            digitalWriteFast(solenoidpin, LOW);
            set_escs(belt_m0, belt_m1, fwheel_m2, fwheel_m3, belts_prime_pwm, fwheel_prime_pwm);
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
            odrive1.setVelocity(drum_target_vel);
            digitalWriteFast(solenoidpin, HIGH);
            set_escs(belt_m0, belt_m1, fwheel_m2, fwheel_m3, belts_shoot_pwm, fwheel_shoot_pwm);

            delay(solenoid_engage_time_ms);
            start_firing_time_ms = millis();
            rounds_starting = rounds_remaining;
            rounds_fired = 0;
          }

          // Update rounds fired/remaining count
          int32_t time_elapsed = millis(); 
          time_elapsed -= start_firing_time_ms;
          rounds_fired = floor(time_elapsed / ms_per_round);
          rounds_remaining = max(rounds_starting - rounds_fired, 0);
          display.clearDisplay();
          display_rounds_remaining(rounds_remaining);
          display.display();

          // Stop shooting if rounds fired limit hit for current mode
          switch (fire_mode)
          {
            case SINGLE:
            {
              if (rounds_fired >= 1)
                op_state = NOT_SHOOTING;
              break;
            }
            case BURST:
            {
              if (rounds_fired >= burst_qty)
                op_state = NOT_SHOOTING;
              break;
            }
            case AUTO:
            {
              // Some buffer added here in case mag velocity sags
              if (rounds_fired > (rounds_starting + 10))
                op_state = NOT_SHOOTING;
              break;
            }
          }

          // Stop shooting if trigger released
          if (digitalRead(shootpin)) {
            op_state = READY_TO_SHOOT;
            delay(debounce_delay);
          }
          break;
        }
      case NOT_SHOOTING:
        {
          if (statechange) {
            digitalWriteFast(solenoidpin, LOW);
            delay(debounce_delay);
          }

          // Return to ready to shoot once trigger is released
          if (digitalRead(shootpin)) {
            op_state = READY_TO_SHOOT;
            delay(debounce_delay);
          }
          break;
        }
      case SPINNING_DOWN:
        {
          if (statechange) {
            odrive1.setVelocity(0.0);
            set_escs(belt_m0, belt_m1, fwheel_m2, fwheel_m3, belts_off_pwm, fwheel_off_pwm);
          }

          // Spin back up if button pressed
          if (!digitalRead(spinpin)) {
            delay(debounce_delay);
            op_state = SPINNING_UP;
          }

          // Get drum vel
          float drum_vel = get_odrive_vel();
          if (drum_vel == INVALID) {
            ++invalid_vel_count;
            if (invalid_vel_count > 10) {
              op_state = ERRORSTATE;
              error_msg = "Invalid ODrive Vel";
            }
          } else
            invalid_vel_count = 0;
          if (drum_vel < 0.1)
            op_state = INITIALIZED;
          break;
        }
      case MAG_RELEASED:
        {
          magservo.attach(magservopin);
          if (digitalRead(magbutton)) {
            delay(debounce_delay);
            op_state = MAG_ENGAGING;
          }
          magservo.write(mag_servo_release_angle);
          break;
        }
      case MAG_ENGAGING:
        {
          magservo.write(mag_servo_engage_angle);
          delay(mag_engage_time_ms);
          magservo.detach();
          rounds_remaining = drum_mags * rounds_per_mag;
          op_state = INITIALIZED;
          break;
        }
      case ERRORSTATE:
        {
          if (statechange) {
            set_escs(belt_m0, belt_m1, fwheel_m2, fwheel_m3, belts_off_pwm, fwheel_off_pwm);
            odrive1.setVelocity(0.0);
            display.clearDisplay();
            display.setCursor(0, 0);
            display.setTextSize(2);
            display.println(F("ERROR"));
            display.setTextSize(1);
            display.println(error_msg);
            display.display();
            display.startscrollleft(0x00, 0x0F);
          }
          digitalWriteFast(LED_BUILTIN, HIGH);
          delay(100);
          digitalWriteFast(LED_BUILTIN, LOW);
          delay(100);
          break;
        }
    }
    // Check for ODrive Errors
    // Serial.println(odrive1.getState());

    // Handle state changes
    statechange = (last_state != op_state);
    last_state = op_state;
    // Serial.print("STATE: ");
    // Serial.println(op_state);

    frame += 1;
    delay(5);
  }
}

void UI_buttonHandler() {
  delay(debounce_delay);
  if (!digitalRead(uibutton)) {
    buttonPressed = true;
  }
}

void UI_scrollHandler() {
  delay(debounce_delay);
  if (!digitalRead(uiscrollpulse)) {
    cyclePressed = true;
  }
}
