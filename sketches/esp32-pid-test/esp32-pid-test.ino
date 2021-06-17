// Controlling the motor position using the PID library

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

// Definitions for encoder
#define ROTARY_PIN_A 34
#define ROTARY_PIN_B 35
// Definitions for H-bridge controller
#define ENA 13
#define IN_1 12
#define IN_2 14

// Global variables for the PID 
double Setpoint, Input, Output;
double PPR = 374*4; // Times 4 for full quadrature
//double setpoints[] = {0.25, 0.5, 0.75, 1, 0.75, 0.5, 0.25, 0., -0.25, -0.5, -1, -0.5, -0.25,0};
double setpoints[] = {1, 4, 1, 0, -1,-4,-2, 0};
// Output is encoder counts, input is pwm duty cycle
double Kp=1, Ki=1, Kd=0.2;
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double deadzoneLimit = 100;
 
// Global variables for step-response test
const int stepDutycycle = 200;
const int experimentLength = 400;
const int dt = 20;
long int counts[experimentLength];
bool done = false;


// Global variables for motor controller
const int CCW = 2;
const int CW = 1;
const int pwmChannel = 0;
const int freq = 5000;
const int resolution = 8;

ESP32Encoder encoder;

void spinMotor(int dir, int dutyC);

void setup() {
  // put your setup code here, to run once:
  
  encoder.attachFullQuad(ROTARY_PIN_A, ROTARY_PIN_B);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(ENA, pwmChannel);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);

  Input = (double) encoder.getCount();
  Setpoint = setpoints[0];
  
  Serial.begin(115200);
  delay(400);
  
  // put your main code here, to run repeatedly:
  Serial.println("Getting ready to run PID");
  Serial.print("Setpoint: ");
  Serial.println(Setpoint);
  Serial.print("Input: ");
  Serial.println(Input);

  pid.SetSampleTime(dt);
  pid.SetOutputLimits(-255, 255);
  pid.SetMode(AUTOMATIC);

}

void loop() {
 
  for (int i = 0; i< sizeof(setpoints)/sizeof(setpoints[0]); i++) {
    Setpoint = setpoints[i]*PPR;
    Input = (double) encoder.getCount();
    double error = Setpoint - Input;
    while( abs(error) > 6) {
      Input = (double) encoder.getCount();
      error = Setpoint - Input;
      pid.Compute();

     Serial.print(Setpoint);
    Serial.print("  ");
    Serial.print(Input);
    Serial.print("  ");
    Serial.println(Output);
    
  
      int dir;
      if (Output > 0){ 
        dir = CW;
        Output += deadzoneLimit;
      }
      else { 
        dir = CCW;
        Output -= deadzoneLimit;
      }

      Output = min(255.0, Output);
      Output = max(-255.0, Output);
      
      spinMotor(dir, abs(Output));
      delay(dt);
    }
    
    Input = Setpoint;
    pid.Compute();
 
    Serial.print(Setpoint);
    Serial.print("  ");
    Serial.print(Input);
    Serial.print("  ");
    Serial.println(Output);
    delay(5000);
  }
}


void spinMotor(int dir, int dutyC){
    if (dir == CCW) {
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
    }
    else{
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
    }

    ledcWrite(pwmChannel, dutyC);
}
