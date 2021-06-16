#include <Arduino.h>
#include <ESP32Encoder.h>

#define ROTARY_PIN_A 34
#define ROTARY_PIN_B 35
#define ENA 13
#define IN_1 12
#define IN_2 14

// For motor controller
const int CCW = 2;
const int CW = 1;
const int pwmChannel = 0;
const int freq = 5000;
const int resolution = 8;

ESP32Encoder encoder;

void rotateMotor(long int desiredCount, int dir, ESP32Encoder enc, int dutyC);

void setup() {
  // put your setup code here, to run once:
  
  encoder.attachFullQuad(ROTARY_PIN_A, ROTARY_PIN_B);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(ENA, pwmChannel);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Hello, rotValue is: ");
  Serial.println(encoder.getCount());
  Serial.println("Rotating CW");
  rotateMotor(800, CW, encoder, 200);
  delay(100);
  Serial.println(encoder.getCount());
  Serial.println("Rotating CCW");
  rotateMotor(800, CCW, encoder, 200);
}

void rotateMotor(long int desiredCount, int dir, ESP32Encoder enc, int dutyC){
    long int init_count =  enc.getCount();
    if (dir == CCW) {
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);

      while( enc.getCount() - init_count > -desiredCount) { 
        ledcWrite(pwmChannel, dutyC);  
        delay(10);
      }

    }
    else{
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);

      while( enc.getCount() - init_count < desiredCount) { 
        ledcWrite(pwmChannel, dutyC);  
        delay(10);
      }

    }
    
    ledcWrite(pwmChannel, 0);  

}
