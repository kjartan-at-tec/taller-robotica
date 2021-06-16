#include <Arduino.h>
#include <ESP32Encoder.h>

#define ENA 13
#define IN_1 12
#define IN_2 14

#define ROTARY_PIN_A 34
#define ROTARY_PIN_B 35

// For motor controller
const int CCW = 2;
const int CW = 1;
const int pwmChannel = 0;
const int freq = 5000;
const int resolution = 8;

// For encoder
long int rotValue = 0;
uint8_t state=0;


void spinMotor(int inPlus, int inMin, int pwmCh, int dutyC);  

portMUX_TYPE gpioMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrAB() {
  portENTER_CRITICAL_ISR(&gpioMux);
    rotValue++;
  portEXIT_CRITICAL_ISR(&gpioMux);
}


void setup() {
  // put your setup code here, to run once:

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(ENA, pwmChannel);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  
  pinMode(ROTARY_PIN_A, INPUT_PULLUP);
  pinMode(ROTARY_PIN_B, INPUT_PULLUP);

  attachInterrupt(ROTARY_PIN_A, isrAB, CHANGE);
  attachInterrupt(ROTARY_PIN_B, isrAB, CHANGE);
  
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  for (int dc = 0; dc <= 255; dc++){
    spinMotor(IN_1, IN_2, pwmChannel, dc);
    delay(200);
  }
  
  Serial.print("Hello, rotValue is: ");
  Serial.println(rotValue);
  
  for (int dc = 255; dc >0; dc--){
    spinMotor(IN_1, IN_2, pwmChannel, dc);
    delay(200);
  }
  
  
  delay(700);
}

void spinMotor(int inPlus, int inMin, int pwmCh, int dutyCycle){
    digitalWrite(inPlus, HIGH);
    digitalWrite(inMin, LOW);
    ledcWrite(pwmCh, dutyCycle);  
}
