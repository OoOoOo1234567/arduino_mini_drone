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

    uint8_t TXwantthrottle;
    int8_t TXwantroll;
    int8_t TXwantyaw;
    int8_t TXwantpitch; 

    unsigned long lastReceiveTime;
    void emergencyStop();

public:
    RXnrf(uint8_t ce, uint8_t csn); // בנאי שמקבל פינים
    void setup();
    void loop();

    // Getters
    int8_t getTXwantthrottle() { return TXwantthrottle; }
    int8_t getTXwantroll() { return TXwantroll; }
    int8_t getTXwantyaw() { return TXwantyaw; }
    int8_t getTXwantpitch() { return TXwantpitch; }
};
