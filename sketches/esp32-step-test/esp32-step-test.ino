// Step test to determine relationship between input voltage and output speed
#include <Arduino.h>
#include <ESP32Encoder.h>

// Definitions for encoder
#define ROTARY_PIN_A 34
#define ROTARY_PIN_B 35
// Definitions for H-bridge controller
#define ENA 13
#define IN_1 12
#define IN_2 14

// Global variables for step-response test
const int stepDutycycle = 200;
const int experimentLength = 400;
const int dt = 2;
long int counts[experimentLength];
bool done = false;


// Global variables for motor controller
const int CCW = 2;
const int CW = 1;
const int pwmChannel = 0;
const int freq = 5000;
const int resolution = 8;

ESP32Encoder encoder;

void rotateMotor(int dir, long int *counts, int ncounts, long int dt, ESP32Encoder enc, int dutyC);

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
  //Serial.println(encoder.getCount());
  if (!done) {
    rotateMotor(CW, counts, experimentLength, dt, encoder, stepDutycycle);
    for (int i=0; i < experimentLength; i++){
      Serial.println(counts[i]);
      delay(dt);
    }

    done = true;
  }


}

void rotateMotor(int dir, long int *counts, int ncounts, long int dt, ESP32Encoder enc, int dutyC){
    if (dir == CCW) {
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
    }
    else{
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
    }

    ledcWrite(pwmChannel, dutyC);
    for (int i=0; i < ncounts; i++){
      counts[i] = enc.getCount();
      delay(dt);
    }      
    
    ledcWrite(pwmChannel, 0);  

}
