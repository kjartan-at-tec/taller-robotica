// Controlling the motor position using a discrete-time PID controller.
// The parameters are for a continuous-time PID controller, and the 
// discretization is by Tustin's method.
// The controller does anti-windup by backtracking, and deadzone
// compensation.
// The code is copied from Åström and Wittenmark Computer-controlled systems,
// Listing 8.1

#include <Arduino.h>
#include <ESP32Encoder.h>

// Definitions and global variables for encoder
#define ROTARY_PIN_A 34
#define ROTARY_PIN_B 35
ESP32Encoder encoder;

// Definitions and global variables for H-bridge controller
#define ENA 13
#define IN_1 12
#define IN_2 14
#define PWM_RES 10
const int pwm_max = pow(2,PWM_RES);
const int CCW = 2;
const int CW = 1;
const int pwmChannel = 0;
const int freq = 10000;

// Sampling period in ms
const long  DT = 30;

// ---------------------------------------------------------
// PID controller implementation
// ---------------------------------------------------------
struct PID_Data {
  struct {
    double uc;  // Setpoint
    double y;   // Measured variable
    double u;   // Controller output
    double v;   // Limited (saturated) controller output
  } Signals;
  struct {
    double I;    // Integral part
    double D;    // Derivative part
    double yold; // Delayed measured variable
  } States;
  struct {
    double K;    // Controller gain
    double Ti;   // Integral time constant
    double Td;   // Derivate time constant
    double Tt;   // Reset time constant (anti-windup)
    double N;    // Maximum derivative gain
    double b;    // Fraction of setpoint in prop term
    double ulow; // Controller output lower limit
    double uhigh;// Controller output higher limit
    double h;    // Sampling period
    double dzlow;// Lower limit of deadzone
    double dzhigh;//Higher limit of deadzone
    double bi, ar, bd, ad; // Used for convenience in computations
  } Params;
} pid_data;


void PID_Init(struct PID_Data *pd) {
   pd->States.I = 0;
   pd->States.D = 0;
   pd->States.yold = 0;
   pd->Params.K = 1;
   pd->Params.Ti = 0.4;
   pd->Params.Td = 0.2;
   pd->Params.Tt = 0.4;
   pd->Params.N = 8;
   pd->Params.ulow = -pwm_max;
   pd->Params.uhigh = pwm_max;
   pd->Params.dzlow = 200;
   pd->Params.dzhigh = 200;
   pd->Params.h = ((double) DT) / 1000.0;
   pd->Params.bi = pd->Params.K * pd->Params.h/pd->Params.Ti;
   pd->Params.ar = pd->Params.h / pd->Params.Tt;
   pd->Params.bd = pd->Params.K * pd->Params.N * pd->Params.Td /
            (pd->Params.Td + pd->Params.N * pd->Params.h);
   pd->Params.ad = pd->Params.Td / (pd->Params.Td + pd->Params.N * pd->Params.h);
}
 
void PID_Compute(struct PID_Data *pd){
  // Proportional part
  double P = pd->Params.K*(pd->Params.b * pd->Signals.uc - 
                               pd->Signals.y);
  // Derivative part
  pd->States.D = pd->Params.ad * pd->States.D - 
             pd->Params.bd * (pd->Signals.y - pd->States.yold);

  // Calculate control signal
  pd->Signals.v = P + pd->States.I + pd->States.D;
 
  // Deadzone compensation
  if ( pd->Signals.v > 0 ){
    pd->Signals.v += pd->Params.dzhigh;
  } else {
    pd->Signals.v -= pd->Params.dzlow;
  }

  // Handle actuator limitations
  if ( pd->Signals.v < pd->Params.ulow ) {
    pd->Signals.u = pd->Params.ulow;
  } else if ( pd->Signals.v > pd->Params.uhigh ) {
    pd->Signals.u = pd->Params.uhigh;
  } else {
    pd->Signals.u = pd->Signals.v;
  }
}

void PID_UpdateStates(struct PID_Data *pd){
  // Integral part
  pd->States.I = pd->States.I +
      pd->Params.bi * (pd->Signals.uc - pd->Signals.y) +
      pd->Params.ar * (pd->Signals.u - pd->Signals.v);

  // Store previous feedback signal
  pd->States.yold = pd->Signals.y;
}

// ---------------------------------------------------------
// End PID controller implementation
// ---------------------------------------------------------


void writeU(double *u); // Handles the motor
void spinMotor(int dir, int dutyC);

// Setpoints for testing
double PPR = 374*4; // Times 4 for full quadrature
//double setpoints[] = {0.25, 0.5, 0.75, 1, 0.75, 0.5, 0.25, 0., -0.25, -0.5, -1, -0.5, -0.25,0};
double setpoints[] = {0.5, 1, 0.5, 0, -0.5,-2,-1, 0};
 

void setup() {
  // put your setup code here, to run once:
  
  encoder.attachFullQuad(ROTARY_PIN_A, ROTARY_PIN_B);

  ledcSetup(pwmChannel, freq, PWM_RES);
  ledcAttachPin(ENA, pwmChannel);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);

  Serial.begin(115200);
  delay(400);
  
  PID_Init(&pid_data);
}

void loop() {
 
  for (int i = 0; i< sizeof(setpoints)/sizeof(setpoints[0]); i++) {
    double Setpoint = setpoints[i]*PPR;
    double Input = (double) encoder.getCount();
    pid_data.Signals.uc = Setpoint;
    double error = Setpoint - Input;
    while( abs(error) > 4) {
      pid_data.Signals.y = Input;
      
      PID_Compute(&pid_data);
      writeU(&pid_data.Signals.u);  
      PID_UpdateStates(&pid_data);
      
       
      Serial.print(pid_data.Signals.uc);
      Serial.print("  ");
      Serial.print(pid_data.Signals.y);
      Serial.print("  ");
      Serial.print(pid_data.Signals.v);
      Serial.print("  ");
      Serial.println(pid_data.Signals.u);

      delay(DT);
      Input = (double) encoder.getCount();
      error = Setpoint - Input;
    }
    double z = 0;
    writeU(&z);  
    delay(4000);
  }
}

void writeU(double *u) {
   int dir;
   if ( *u > 0){ 
        dir = CW;
      }
    else { 
      dir = CCW;
    }
    spinMotor(dir, abs(*u));
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
