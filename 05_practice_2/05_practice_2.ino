#define LED_PIN 7
void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  for(int i = 1; i < 11; i++){
    digitalWrite(LED_PIN, i % 2);
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH);
  while(1){}
}
