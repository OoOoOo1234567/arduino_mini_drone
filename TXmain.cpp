#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>

RF24 radio(8, 9); // CE, CSN

byte addresses[][6] = {"1Node", "2Node"};


struct MyData {
  int16_t throttle;
  int16_t roll; 
  int16_t pitch; 
  int16_t yaw; 
} __attribute__((packed)); // שלא יהיה רווח 

MyData joystickData;

void setup() {

  pinMode(53, OUTPUT); // שומר על ה-Mega במצב Master ל-SPI 

  Serial.begin(115200);
  Serial.println("Transmitter Ready - Sending Joystick Data");

  radio.begin();

  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_2MBPS); 
  radio.setChannel(124); 

  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
}

void loop() {

  joystickData.throttle = analogRead(A2);
  joystickData.roll     = analogRead(A3) - 512;
  joystickData.pitch    = analogRead(A0) - 512;
  joystickData.yaw      = analogRead(A1) - 512;

//  תיקון רעשים 
if (joystickData.throttle < 10) joystickData.throttle = 0;
if (joystickData.roll > -10 && joystickData.roll < 10) joystickData.roll = 0;
if (joystickData.pitch > -10 && joystickData.pitch < 10) joystickData.pitch = 0;
if (joystickData.yaw > -10 && joystickData.yaw < 10) joystickData.yaw = 0; 


  radio.stopListening(); 

  Serial.print("Sending -> T:"); Serial.print(joystickData.throttle);
  
  if (!radio.write( &joystickData, sizeof(MyData) )) {
    Serial.println("Fail");    
  } else {
    Serial.println("Sent OK!");
  }

  radio.startListening();
  unsigned long started_waiting_at = millis();
  bool timeout = false;

  while ( ! radio.available() ) {
    if (millis() - started_waiting_at > 200 ) {
      timeout = true;
      break;
    }
  }

  if (timeout) {
    Serial.println("No response from Drone");
  } else {
    unsigned char response;
    radio.read( &response, sizeof(unsigned char) );
    Serial.print("Drone says: ");
    Serial.println(response);
  }

  delay(10); 

//////////////////////////////////////
/*
  Serial.print("Sent -> T:"); Serial.print(joystickData.throttle);
  Serial.print(" R:"); Serial.print(joystickData.roll);
  Serial.print(" P:"); Serial.print(joystickData.pitch);
  Serial.print(" Y:"); Serial.println(joystickData.yaw);
  delay(1000); // שליחה כל 100ms
  /////////////////////////////////////
  */
}
