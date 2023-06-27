#define solenoid_pin 5
#define sw_end_pin   A3
#define sw_beg_pin   A2

void setup() {
  // put your setup code here, to run once:
  pinMode(solenoid_pin, OUTPUT);
  pinMode(sw_end_pin, INPUT);
  pinMode(sw_beg_pin, INPUT);
  digitalWrite(solenoid_pin, LOW);
}

void loop() {
  // test parameters
  int nCycles = 5;
  while(!Serial);

  uint32_t tstart = millis();
  for (uint16_t c = 0; c < nCycles; c++)
  {
    digitalWrite(solenoid_pin, HIGH);
    Serial.println("HIGH");
    delay(2);
    while(digitalRead(sw_end_pin) == HIGH) {};
    digitalWrite(solenoid_pin, LOW);
    Serial.println("LOW");
    delay(2);
    while(digitalRead(sw_beg_pin) == HIGH) {};
  }
  uint32_t tend = millis();
  Serial.print(nCycles);
  Serial.print(" cycles completed in ");
  Serial.print(tend-tstart);
  Serial.println("ms");
  while(true);
}
