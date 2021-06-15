#include <Arduino.h>

#define POT_PIN 35

int potValue = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  potValue = analogRead(POT_PIN);
  Serial.print("The potentiometer reads: ");
  Serial.println(potValue);
  
  delay(700);
}
