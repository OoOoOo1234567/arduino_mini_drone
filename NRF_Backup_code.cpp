#include "NRF_Backup_code.h"
#include <SPI.h>
#include <RF24.h>
#include <Arduino.h>

// אתחול הבנאי והגדרת הפינים של הרדיו
RXnrf::RXnrf(uint8_t ce, uint8_t csn) : radio(ce, csn), TXwantthrottle(0), 
 TXwantroll(0), TXwantyaw(0), TXwantpitch(0) {

    memcpy(addresses[0], "1Node", 6);
    memcpy(addresses[1], "2Node", 6);
}

void RXnrf::setup() {
    Serial.begin(115200);
    Serial.println("Receiver Ready - Waiting for Joystick Data...");

    if (!radio.begin()) {
        Serial.println("Radio Hardware Error!");
    }

    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_2MBPS);
    radio.setChannel(124);

    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);

    radio.startListening();
    lastReceiveTime = millis();
}

void RXnrf::loop() {
    if (radio.available()) {
        while (radio.available()) {
            radio.read(&data, sizeof(MyData));
        }

        lastReceiveTime = millis();

        /*   // הדפסה לדיבאג
        Serial.print("Received -> T: "); Serial.print(data.throttle);
        Serial.print(" | R: "); Serial.println(data.roll);
        */

        // עיבוד ומיפוי
        TXwantthrottle = map(data.throttle, 0, 1023, 0, 255);
        TXwantroll = map(data.roll, -512, 512, -127, 127);
        TXwantpitch = map(data.pitch, -512, 512, -127, 127);
        TXwantyaw = map(data.yaw, -512, 512, -127, 127);

        // מענה למשדר
        radio.stopListening();
        unsigned char response = 1;
        radio.write(&response, sizeof(unsigned char));
        radio.startListening();
    }

    // בדיקת חירום מחוץ ל-if של ה-available
    if (millis() - lastReceiveTime > 500) {
        emergencyStop();
    }
}

void RXnrf::emergencyStop() {
    TXwantthrottle = 0;
    TXwantroll = 0;
    TXwantyaw = 0;
    TXwantpitch = 0;
}