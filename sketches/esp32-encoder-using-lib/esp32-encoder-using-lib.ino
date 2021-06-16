#include <Arduino.h>
#include <ESP32Encoder.h>

#define ROTARY_PIN_A 34
#define ROTARY_PIN_B 35

ESP32Encoder encoder;

void setup() {
  // put your setup code here, to run once:
  
  encoder.attachFullQuad(ROTARY_PIN_A, ROTARY_PIN_B);
  
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Hello, rotValue is: ");
  Serial.println(encoder.getCount());
  
  delay(100);
}
