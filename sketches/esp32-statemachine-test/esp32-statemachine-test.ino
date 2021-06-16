// Testing statemachine library for arduino. 
// See https://github.com/jrullan/StateMachine 

#include "BluetoothSerial.h"
#include "StateMachine.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define LED 2

const int STATE_DELAY = 1000;
int randomState = 0;

void idle();
void positionControl();
void changeSettings();
bool transitionS0();
bool transitionS0S1();
bool transitionS0S2();
bool transitionSS0();


StateMachine machine = StateMachine();
State* S0 = machine.addState(&idle);
State* S1 = machine.addState(&positionControl);
State* S2 = machine.addState(&changeSettings);

BluetoothSerial SerialBT;
String DATA;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED,OUTPUT);
  Serial.begin(115200);

  S0 ->addTransition(&transitionS0, S0);
  S0 ->addTransition(&transitionS0S1, S1);
  S0 ->addTransition(&transitionS0S2, S2);
  S1 ->addTransition(&transitionSS0, S0);
  S2 ->addTransition(&transitionSS0, S0);
  
  delay(3000);
  SerialBT.begin("ESP32test");
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  // put your main code here, to run repeatedly:
 
  machine.run();
  delay(STATE_DELAY);
    
}

// State definitions
void idle(){
  // State 0, the system is idle waiting for input
  Serial.println("State 0 - idle");
  SerialBT.println("State 0 - idle");
  SerialBT.println("0  - Return to main menu");
  SerialBT.println("1  - Position control");
  SerialBT.println("2  - Change settings");

  DATA = SerialBT.readString();
  DATA.trim();
  if(machine.executeOnce) {
    SerialBT.println("Execute once");
    digitalWrite(LED, !digitalRead(LED));
    
  }
}


void positionControl(){
  // State 1, the system is waiting for setpoint
  Serial.println("State 1 - position control");
  SerialBT.println("State 1 - position control\n");

  DATA = SerialBT.readString();
  DATA.trim();
  if(machine.executeOnce) {
    SerialBT.println("Execute once");
    digitalWrite(LED, !digitalRead(LED));
    
  }
}

void changeSettings(){
  // State 0, the system is idle waiting for input
  Serial.println("State 2 - Settings");
  SerialBT.println("State 2 - Settings");

  DATA = SerialBT.readString();
  DATA.trim();
  if(machine.executeOnce) {
    SerialBT.println("Execute once");
    digitalWrite(LED, !digitalRead(LED));
    
  }
}

// Transitions
bool transitionS0S1() {
   if (DATA == "1") {
      Serial.println("Switching to S1");
      return true;
    } 
    return false;
}
bool transitionS0S2() {
   if (DATA == "2") {
      Serial.println("Switching to S2");
      return true;
    } 
    return false;
}
bool transitionSS0() {
   if (DATA == "0") {
      Serial.println("Switching to S0");
      return true;
    } 
    return false;
}
bool transitionS0() {
  return false;
}
