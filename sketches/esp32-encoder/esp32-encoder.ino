#include <Arduino.h>

long int rotValue = 0;
uint8_t state=0;

#define ROTARY_PIN_A 34
#define ROTARY_PIN_B 35


portMUX_TYPE gpioMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrAB() {
  portENTER_CRITICAL_ISR(&gpioMux);
    rotValue++;
  portEXIT_CRITICAL_ISR(&gpioMux);
}


void setup() {
  // put your setup code here, to run once:

  pinMode(ROTARY_PIN_A, INPUT_PULLUP);
  pinMode(ROTARY_PIN_B, INPUT_PULLUP);

  attachInterrupt(ROTARY_PIN_A, isrAB, CHANGE);
  attachInterrupt(ROTARY_PIN_B, isrAB, CHANGE);
  
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Hello, rotValue is: ");
  Serial.println(rotValue);
  
  delay(700);
}
