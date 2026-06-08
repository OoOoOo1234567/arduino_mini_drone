#pragma once
#include <Arduino.h>
#include <Wire.h>

class MPU6050 {

    // כתובות לצורך 
    // כתובות לצורך תקשורת I2C
    const uint8_t MPU_ADDRESS = 0x68;  // I2C address of the MPU6050
    const uint8_t PWR_MGMT_1 = 0x6B;   // Power management register
    const uint8_t ACCEL_CONFIG = 0x1C; // Accelerometer configuration register
    const uint8_t ACCEL_XOUT_H = 0x3B; // Accelerometer X-axis high byte
    const uint8_t GYRO_CONFIG = 0x1B;  // Gyroscope configuration register
    const uint8_t GYRO_XOUT_H = 0x43;  // Gyroscope X-axis high byte

    // מידע גולמי מהחיישן
    int16_t raw_accel_X, raw_accel_Y, raw_accel_Z;
    int16_t raw_gyro_X, raw_gyro_Y, raw_gyro_Z;

    // ממוצע טעויות לצורך קליברציה
    int16_t accel_X_offset = 0, accel_Y_offset = 0, accel_Z_offset = 0;
    int16_t gyro_X_offset = 0, gyro_Y_offset = 0, gyro_Z_offset = 0;

    // מידע לאחר תיקון הטעויות
    int16_t accel_X_after_offset, accel_Y_after_offset, accel_Z_after_offset; // אופסט מתוקן אקס  
    int16_t gyro_X_after_offset, gyro_Y_after_offset, gyro_Z_after_offset;    // אופסט מתוקן גיירו 

    // משתני סינון + low pass filter
    float accel_X_filtered = 0.0, accel_Y_filtered = 0.0, accel_Z_filtered = 0.0; // אחרי פילטר 

    // Low pass filter
    const float trust_the_old_data = 0.7; // כמה אני מסתמך על המידע הישן לעומת המידע החדש

    // זוויות מחושבות
    float roll = 0, pitch = 0, yaw = 0;

    // זמן של כל לופ
    unsigned long last_loop_time = 0;
    const unsigned long LOOP_INTERVAL = 10; // 10ms

    // sensor fusion
    const float ALPHA = 0.98; // כמה אני ''מאמין'' לג'יירו לעומת האקס

    // Timing variable for sensor fusion and yaw calculation
    unsigned long last_time = 0;

    //  המרה לרדיאנים פר שנייה בשביל הג'יירו (1.0 / 16.4) * (PI / 180.0) 
    const float radian = 0.001064;


    void calibrate_sensor();
    void initialize_angles();
    void read_accelerometer();
    void read_gyroscope();
    void calculate_angles();

    
    public:

    MPU6050();
    
    void setup();
    void loop();

    // פונקציית GETTER אבל עם המרה למעלות 
    float GET_ROLL() { return roll * (180.0 / PI); }
    float GET_PITCH() { return pitch * (180.0 / PI); }
    float GET_YAW() { return yaw * (180.0 / PI); }

    //  המרה מברדיאנים למעלו  
    // המרה למעלות מרדיאנים היא המשתנה כפול 180 חלקי פאי כי פאי רדיאנים שווים ל180 מעלות 
    // בערך 57.3 
};
