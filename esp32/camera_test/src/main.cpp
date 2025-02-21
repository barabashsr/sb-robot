#include <Arduino.h>
#define RGB_BUILTIN 48

void setup() {
  pinMode(RGB_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Serial started");
}

void loop() {
  digitalWrite(RGB_BUILTIN, HIGH);
  Serial.println("LED ON");
  delay(1000);
  digitalWrite(RGB_BUILTIN, LOW);
  Serial.println("LED OFF");
  delay(1000);
}
