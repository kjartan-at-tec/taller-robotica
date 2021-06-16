// Testing bluetooth connection. Following instructions in 
// https://randomnerdtutorials.com/esp32-bluetooth-classic-arduino-ide/

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define LED 2

BluetoothSerial SerialBT;
String DATA;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED,OUTPUT);
  Serial.begin(115200);
  delay(3000);
  SerialBT.begin("ESP32test");
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  // put your main code here, to run repeatedly:
 
  if (SerialBT.available()) {
    DATA = SerialBT.readString();
    DATA.trim();
    Serial.println("New DATA: ");
    Serial.println(DATA);
 
    if (DATA == "ON") {
      Serial.println("Turning LED on");
      digitalWrite(LED, HIGH);
      }
    if (DATA == "OFF") {
      Serial.println("Turning LED off");
      digitalWrite(LED, LOW);
    }
  }

  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  delay(20);
    
}
