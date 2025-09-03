const int LED_PIN = 7;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') digitalWrite(LED_PIN, HIGH);
    else if (c == '0') digitalWrite(LED_PIN, LOW);
  }
}
