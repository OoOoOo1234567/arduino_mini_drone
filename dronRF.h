#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

class RXnrf {
private:
    RF24 radio; // רק הגדרה, לא אתחול פינים כאן
    byte addresses[2][6];

    struct MyData {
        int16_t throttle;
        int16_t roll; 
        int16_t pitch; 
        int16_t yaw; 
    } __attribute__((packed));

    MyData data;

    uint8_t TXwantthrottle = 0;
    int8_t TXwantroll = 0;
    int8_t TXwantyaw = 0;
    int8_t TXwantpitch = 0;

    unsigned long lastReceiveTime;
    void emergencyStop();

public:
    RXnrf(uint8_t ce, uint8_t csn); // בנאי שמקבל פינים
    void setup();
    void loop();
    
    // Getters
    uint8_t getTXwantthrottle() { return TXwantthrottle; }
    uint8_t getTXwantroll() { return TXwantroll; }
    uint8_t getTXwantyaw() { return TXwantyaw; }
    uint8_t getTXwantpitch() { return TXwantpitch; }
};
