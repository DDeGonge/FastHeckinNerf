#include <Servo.h>

int power_on_pwm = 1100;

void setup(){}

void arm_servo(Servo myservo)
{
  // Janky but it kinda works?
  myservo.writeMicroseconds(1600);
  delay(500);
  for (uint32_t t = 1600; t >= 1000; t-=20)
  {
    myservo.writeMicroseconds(t);
    Serial.println(t);
    delay(2);
  }
  delay(500);
  for (uint32_t t = 1000; t <= 1600; t+=20)
  {
    myservo.writeMicroseconds(t);
    Serial.println(t);
    delay(5);
  }
  for (uint32_t t = 1600; t >= 1000; t-=20)
  {
    myservo.writeMicroseconds(t);
    Serial.println(t);
    delay(5);
  }
  delay(2000);
}

void loop()
{
  Servo m0, m1;
  while(!Serial);
  m0.attach(10, 1000, 2000);
  m1.attach(11, 1000, 2000);

  arm_servo(m0);
  arm_servo(m1);

  while(true)
  {
    m0.writeMicroseconds(power_on_pwm);
    m1.writeMicroseconds(power_on_pwm);
    Serial.println("ON");
    delay(300);
    m0.writeMicroseconds(1000);
    m1.writeMicroseconds(1000);
    Serial.println("OFF");
    delay(2000);
  }
  
}
