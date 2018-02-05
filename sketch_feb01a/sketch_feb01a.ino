void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  Serial.println(analogRead(A0));
  delay(100);

}
