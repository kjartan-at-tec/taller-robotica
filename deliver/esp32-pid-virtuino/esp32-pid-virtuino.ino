/// -------------------------------------------------------------
// Closed-loop control with PID controller, using virtuino to 
// control the setpoint.
// Based on virtuino example ESP32 - Control PWM pins 
// Created by Ilias Lamprou.
//
// Virtual pin V[0] is used to receive setpoint from Virtuino app
// Virtual pin V[1] is used to send current motor position to 
// Virtuino.
//
// @author Kjartan Halvorsen, kjartan@tec.mx
/// -------------------------------------------------------------

#include <WiFi.h>

float lastSetPoint = 0;  // In fractions of complete revolutions.

/// --------------------------------------------------------------------
// Sampling period in ms
const long  DT = 30;
/// --------------------------------------------------------------------

//--- SETTINGS ------------------------------------------------
const char* ssid = "INFINITUM27A2_2.4_EXT2";           // The name of your WiFi network (SSID)
const char* password = "mypwd";           // WIFI network PASSWORD
WiFiServer server(8000);                   // Default Virtuino Server port 
IPAddress ip(192, 168, 1, 150);            // where 150 is the desired IP Address. The first three numbers must be the same as the router IP
IPAddress gateway(192, 168, 1, 1);         // set gateway to match your network. Replace with your router IP
//---

//---VirtuinoCM  Library settings --------------
#include "VirtuinoCM.h"
VirtuinoCM virtuino;               
#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
//---


boolean debug = true;              // set this variable to false on the finale code to decrease the request time.


/// --------------------------------------------------------------------
// Definitions and global variables for encoder
/// --------------------------------------------------------------------
#include <ESP32Encoder.h>
#define ROTARY_PIN_A 34
#define ROTARY_PIN_B 35
ESP32Encoder encoder;
double PPR = 374*4; // Times 4 for full quadrature
/// --------------------------------------------------------------------


// Definitions and global variables for H-bridge controller
#define ENA 13
#define IN_1 12
#define IN_2 14
#define PWM_RES 10
const int pwm_max = pow(2,PWM_RES);
const int pwmChannel = 0;
const int freq = 10000;
const int CCW = 2;
const int CW = 1;

/// ---------------------------------------------------------
// PID controller implementation
/// ---------------------------------------------------------
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

/// ---------------------------------------------------------
// End PID controller implementation
/// ---------------------------------------------------------


//============================================================== setup
//==============================================================
void setup() {
  if (debug) {
    Serial.begin(115200);
    while (!Serial) continue;
  }

 encoder.attachFullQuad(ROTARY_PIN_A, ROTARY_PIN_B);


  virtuino.begin(onReceived,onRequested,512);  //Start Virtuino. Set the buffer to 512. With this buffer Virtuino can control about 50 pins (1 command >= 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library

  connectToWiFiNetwork();
  server.begin();
  
  pinMode(LED_BUILTIN,OUTPUT);    // On Virtuino panel add a button to control this pin
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);


  //---- setup for PWM
  ledcSetup(pwmChannel, freq, PWM_RES);
  ledcAttachPin(ENA, pwmChannel);
  // Run CCW
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);

  PID_Init(&pid_data);
  }

//============================================================== loop
//==============================================================
void loop() {
  virtuinoRun();        // Necessary function to communicate with Virtuino. Client handler

  // enter your code below. Avoid to use delays on this loop. Instead of the default delay function use the vDelay that is located on the bottom of this code
  // You don't need to add code to read or write to the pins. Just enter the  pinMode of each Pin you want to use on void setup

  // Control the setpoint. The virtual pin V0 contains the setpoint value
  if (V[0]!=lastSetPoint) {// Set the setpoint every time the V0 is changed
    pid_data.Signals.uc =V[0];
    lastSetPoint=V[0];                // store the V0 value
    Serial.print("Setting setpoint to: ");
    Serial.println(V[0]);
  }
  
  pid_data.Signals.y = (double) encoder.getCount();
  V[1] = (float) pid_data.Signals.y;
      
  PID_Compute(&pid_data);
  writeU(&pid_data.Signals.u);  
  PID_UpdateStates(&pid_data);

//  Serial.print(pid_data.Signals.uc);
//  Serial.print("  ");
//  Serial.print(pid_data.Signals.y);
//  Serial.print("  ");
//  Serial.print(pid_data.Signals.v);
//  Serial.print("  ");
//  Serial.println(pid_data.Signals.u);

  vDelay(DT);
}

/// -------------------------------------------------------------
// Code to spin the motor
/// -------------------------------------------------------------
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
/// -------------------------------------------------------------



/*================= Virtuino Code ==============================
You don't need to make changes to the code bellow

*/

//============================================================== connectToWiFiNetwork
void connectToWiFiNetwork(){
  Serial.println("Connecting to "+String(ssid));
   // If you don't want to config IP manually disable the next two lines
   IPAddress subnet(255, 255, 255, 0);        // set subnet mask to match your network
  WiFi.config(ip, gateway, subnet);          // If you don't want to config IP manually disable this line
  WiFi.mode(WIFI_STA);                       // Config module as station only.
  WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
    }
   Serial.println("");
   Serial.println("WiFi connected");
   Serial.println(WiFi.localIP());
}


//============================================================== onCommandReceived
//==============================================================
/* This function is called every time Virtuino app sends a request to server to change a Pin value
 * The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin 
 * The 'variableIndex' is the pin number index of Virtuino app
 * The 'valueAsText' is the value that has sent from the app   */
 void onReceived(char variableType, uint8_t variableIndex, String valueAsText){     
    if (variableType=='V'){
        float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
        if (variableIndex<V_memory_count) V[variableIndex]=value;              // copy the received value to arduino V memory array
    }
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex){     
    if (variableType=='V') {
    if (variableIndex<V_memory_count) return  String(V[variableIndex]);   // return the value of the arduino V memory array
  }
  return "";
}


 //==============================================================
  void virtuinoRun(){
   WiFiClient client = server.available();
   if (!client) return;
   if (debug) Serial.println("Connected");
   unsigned long timeout = millis() + 3000;
   while (!client.available() && millis() < timeout) delay(1);
   if (millis() > timeout) {
    Serial.println("timeout");
    client.flush();
    client.stop();
    return;
  }
    virtuino.readBuffer="";    // clear Virtuino input buffer. The inputBuffer stores the incoming characters
      while (client.available()>0) {        
        char c = client.read();         // read the incoming data
        virtuino.readBuffer+=c;         // add the incoming character to Virtuino input buffer
        if (debug) Serial.write(c);
      }
     client.flush();
     if (debug) Serial.println("\nReceived data: "+virtuino.readBuffer);
     String* response= virtuino.getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
     if (debug) Serial.println("Response : "+*response);
     client.print(*response);
     client.flush();
     delay(10);
     client.stop(); 
    if (debug) Serial.println("Disconnected");
}


 //============================================================== vDelay
  void vDelay(int delayInMillis){long t=millis()+delayInMillis;while (millis()<t) virtuinoRun();}
