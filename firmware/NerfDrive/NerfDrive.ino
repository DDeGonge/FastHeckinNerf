#include <ODriveArduino.h>
#include <ODriveEnums.h>
#include <Servo.h>

// Pin setup
#define beltpin0 10
#define beltpin1 11
#define spinpin  6
#define shootpin 9

#define INVALID -9999999

// Operation parameters
static int belts_off_pwm      = 1000;
static int belts_prime_pwm    = 1150;
static int belts_shoot_pwm    = 1250;
static float drum_target_vel  = -30.0;
static int debounce_delay     = 20; // Increase if being weird

// State machine parameters
const int INITIALIZING   = 0;
const int INITIALIZED    = 1;
const int SPINNING_UP    = 2;
const int READY_TO_SHOOT = 3;
const int SHOOTING       = 4;
const int SPINNING_DOWN  = 5;
const int ERRORSTATE     = 6;

// Set initial state
int state = INITIALIZING;
int last_state = state;
bool statechange = false;

// Code and stuff
void setup(){
  pinMode(spinpin, INPUT);
  pinMode(shootpin, INPUT);
}

void arm_servo(Servo myservo)
{
  // Janky but it kinda works?
  myservo.writeMicroseconds(1600);
  delay(500);
  for (uint32_t t = 1600; t >= 1000; t-=20)
  {
    myservo.writeMicroseconds(t);
    delay(2);
  }
  delay(500);
  for (uint32_t t = 1000; t <= 1600; t+=20)
  {
    myservo.writeMicroseconds(t);
    delay(2);
  }
  for (uint32_t t = 1600; t >= 1000; t-=20)
  {
    myservo.writeMicroseconds(t);
    delay(2);
  }
  delay(1000);
  myservo.writeMicroseconds(1250);
  delay(500);
  myservo.writeMicroseconds(1000);
  delay(1000);
}

float get_odrive_vel()
{
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
    if (c == ' ')
    {
      if (str == "unknown") pos = INVALID;
      else pos = str.toFloat();
      str = "";
    }
    else if (c == '\n')
    {
      if (pos == INVALID) vel = INVALID;
      else vel = str.toFloat();
      break;
    }
    else
    {
      str += c;
    }
  }

  return vel;
}

void loop()
{
  // Set up stuff
  Servo m0, m1;
  ODriveArduino odrive(Serial1);
  Serial.begin(115200);
  Serial1.begin(115200);

  // Wait for serial to begin
  while(!Serial);

  // ODrive Configuration
  Serial1.println("w axis0.controller.config.vel_limit 40.0");
  Serial1.println("w axis0.motor.config.current_lim 20.0");

  // Run the looooop
  while(true)
  {
    switch(state){
      case INITIALIZING:{
        m0.attach(beltpin0, 1000, 2000);
        m1.attach(beltpin1, 1000, 2000);

        Serial.println("BLDC: Arming ESCs");
        arm_servo(m0);
        arm_servo(m1);
        Serial.println("BLDC: ESCs Armed");

        Serial.println("ODRIVE: Starting Calibration");
        if(!odrive.run_state(0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true, 25.0f)){
          state = ERRORSTATE;
          break;
        }
        Serial.println("ODRIVE: Entering Closed-Loop Control");
        if(!odrive.run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL, false /*don't wait*/)){
          state = ERRORSTATE;
          break;
        }
        
        state = INITIALIZED;
        break;
      }
      case INITIALIZED:{
        if(digitalRead(spinpin)){
          delay(debounce_delay);
          state = SPINNING_UP;
        }
        break;
      }
      case SPINNING_UP:{
        // TODO delete - Demo mode only
        state = READY_TO_SHOOT;
        break;
        
        if(statechange)
        {
          m0.writeMicroseconds(belts_prime_pwm);
          m1.writeMicroseconds(belts_prime_pwm);
          odrive.SetVelocity(0, drum_target_vel);
        }

        // No more spin up
        if(!digitalRead(spinpin)){
          state = SPINNING_DOWN;
          delay(debounce_delay);
        }

        float drum_vel = get_odrive_vel();
        if (drum_vel == INVALID) break;
        Serial.println(drum_vel);
        if (drum_vel > (drum_target_vel - 0.1)) state = READY_TO_SHOOT;
        break;
      }
      case READY_TO_SHOOT:{
        if(statechange)
        {
          m0.writeMicroseconds(belts_prime_pwm);
          m1.writeMicroseconds(belts_prime_pwm);
//          odrive.SetVelocity(0, drum_target_vel);
        }

        // No more spin up
        if(!digitalRead(spinpin)){
          state = SPINNING_DOWN;
          delay(debounce_delay);
        }

        // shoot time
        if(digitalRead(shootpin)){
          state = SHOOTING;
          delay(debounce_delay);
        }
        break;
      }
      case SHOOTING:{
        if(statechange)
        {
          m0.writeMicroseconds(belts_shoot_pwm);
          m1.writeMicroseconds(belts_shoot_pwm);
          odrive.SetVelocity(0, drum_target_vel);
        }

        if(!digitalRead(shootpin)){
          odrive.SetVelocity(0, 0);
          state = READY_TO_SHOOT;
          delay(debounce_delay);
        }
        break;
      }
      case SPINNING_DOWN:{
        if(statechange)
        {
          m0.writeMicroseconds(belts_off_pwm);
          m1.writeMicroseconds(belts_off_pwm);
          odrive.SetVelocity(0, 0.0);
        }

        // Spin back up if button pressed
        if(digitalRead(spinpin)){
          delay(debounce_delay);
          state = SPINNING_UP;
        }
        
        // Get drum vel
        float drum_vel = get_odrive_vel();
        if (drum_vel == INVALID) break;
        if (drum_vel < 0.1) state = INITIALIZED;
        break;
      }
      case ERRORSTATE:{
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        break;
      }
    }
    statechange = (last_state != state);
    last_state = state;
    Serial.print("STATE: ");
    Serial.println(state);
//    delay(5);
  }
}
