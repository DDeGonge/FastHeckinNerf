

#define b1 27
#define b2 21
#define b3 26
#define b4 22
#define s1 7
#define s2 8
#define s3 9
#define s4 10
#define m1 0
#define m2 1
#define m3 2
#define m4 3

#define knob_pulse 25
#define knob_dir 24
#define knob_click 23

#define battery_V 6


void setup() {
  pinMode(b1, INPUT);
  pinMode(b2, INPUT);
  pinMode(b3, INPUT);
  pinMode(b4, INPUT);
  pinMode(knob_pulse, INPUT);
  pinMode(knob_dir, INPUT);
  pinMode(knob_click, INPUT_PULLUP);
  pinMode(battery_V, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(s4, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
}

void loop() {
  Serial.print(digitalRead(knob_pulse));
  Serial.print("\t");
  Serial.print(digitalRead(knob_dir));
  Serial.print("\t");
  Serial.println(digitalRead(knob_click));
}

//void loop() {
//  digitalWrite(m1, HIGH);
//  digitalWrite(m2, HIGH);
//  digitalWrite(m3, HIGH);
//  digitalWrite(m4, HIGH);
//  digitalWrite(LED_BUILTIN, HIGH);
//  delay(1000);
//  float voltage = analogRead(battery_V);
//  voltage /= 30.98;
//  Serial.println(voltage);
//
//  digitalWrite(m1, LOW);
//  digitalWrite(m2, LOW);
//  digitalWrite(m3, LOW);
//  digitalWrite(m4, LOW);
//  digitalWrite(LED_BUILTIN, LOW);
//  delay(1000);
//}
