#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"


MPU6050::MPU6050(){}

void MPU6050::setup() {

    Serial.begin(115200);
    Wire.begin();

    // בדיקה אם החיישן מגיב:
 /*
    Wire.beginTransmission(MPU_ADDRESS);
    byte error = Wire.endTransmission();
    
    if (error != 0) {
        Serial.println("ERROR");
        while(1); // עצור את הקוד
    }
    
    Serial.println("SUCCESS");
 */

    // יציאה ממצב שינה
    Wire.beginTransmission(MPU_ADDRESS); // התחלת שיחה עם MPU6050
    Wire.write(PWR_MGMT_1);              // כניסה לרגיסטר של ניהול כוח
    Wire.write(0x00);                    // שווה להוצאה ממצב שינה (הוא ישן שם כברירת מחדל)
    Wire.endTransmission(true);          // סיום שיחה

    // הגדרת טווח לאקס
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(ACCEL_CONFIG); // כניסה לרגיסטר של הגדרות האקס
    Wire.write(0x10);         // הגדרת הטווח ל8 G
    Wire.endTransmission(true);

    // הגדרת טווח לגיירו
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(GYRO_CONFIG); // כניסה לרגיסטר של הגדרות הגיירו
    Wire.write(0x18);        // הגדרת הטווח ל2000
    Wire.endTransmission(true);

    delay(100); // התייצבות החיישן

    // קליברציה
    calibrate_sensor();

    // אתחול זוויות התחלתיות
    initialize_angles();

    // אתחול זמן לולאה (פה כי זה סוף הסטאפ ועכשיו מתחיל את הלופ)
    last_loop_time = millis();
    last_time = millis();
}

void MPU6050::loop() {

    const unsigned long current_time = millis();

    if (current_time - last_loop_time >= LOOP_INTERVAL)
    {
        last_loop_time = current_time;

        // קריאת נתונים
        read_accelerometer();
        read_gyroscope();

        // חישובים
        calculate_angles();

       /*
        Serial.print("Roll: ");
        Serial.print(roll);
        Serial.print("  Pitch: ");
        Serial.print(pitch);
        Serial.print("  Yaw: ");
        Serial.println(yaw);
        */
    }
}

void MPU6050::calibrate_sensor() {

    const int all_samples = 300; // מספר דגימות לקליברציה

    // משתנים ששומרים סכומים לחישוב הממוצע
    long accel_X_sum = 0, accel_Y_sum = 0, accel_Z_sum = 0;
    long gyro_X_sum = 0, gyro_Y_sum = 0, gyro_Z_sum = 0;


    unsigned long last_sample_time = millis(); // זמן התחלת הקליברציה
    unsigned int samples_collected = 0;        // מספר הדוגמאות שנאספו
    unsigned int interval = 10;                // 10 מילישניות בין דגימה לדגימה

    while (samples_collected < all_samples)
    {
        unsigned long current_time = millis();

        if (current_time - last_sample_time >= interval)
        {
            last_sample_time = current_time;

            // קריאת אקס
            read_accelerometer();
            // קריאת גיירו
            read_gyroscope();

            accel_X_sum += raw_accel_X;
            accel_Y_sum += raw_accel_Y;
            accel_Z_sum += raw_accel_Z;

            gyro_X_sum += raw_gyro_X;
            gyro_Y_sum += raw_gyro_Y;
            gyro_Z_sum += raw_gyro_Z;

            samples_collected++;
        }
    }

    // חישוב ממוצע הטעויות
    accel_X_offset = accel_X_sum / all_samples;
    accel_Y_offset = accel_Y_sum / all_samples;
    accel_Z_offset = (accel_Z_sum / all_samples) - 4096; // הורדת כוח הכבידה

    gyro_X_offset = gyro_X_sum / all_samples;
    gyro_Y_offset = gyro_Y_sum / all_samples;
    gyro_Z_offset = gyro_Z_sum / all_samples;
}

void MPU6050::initialize_angles() {

    read_accelerometer();

    // הזווית לאחר הסינון היא הזווית שמקבלים בקריאה פחות הממוצע טעויות
    accel_X_filtered = raw_accel_X - accel_X_offset;
    accel_Y_filtered = raw_accel_Y - accel_Y_offset;
    accel_Z_filtered = raw_accel_Z - accel_Z_offset;

    /* atan2(y,x)
       y מול x
       ליד
       roll: מול = Y ליד = Z
       pitch: מול = X ליד = sqrt(Y² + Z²)
       כתבתי הסבר בקובץ וורד 
    */

    // חישוב הזוויות ההתחלתיות
    roll = atan2(accel_Y_filtered, accel_Z_filtered);
    pitch = atan2(accel_X_filtered, sqrt(accel_Y_filtered * accel_Y_filtered + accel_Z_filtered * accel_Z_filtered));
    yaw = 0.0; // מתחיל ב0 בגלל שאי אפשר למדוד אותו דרך תאוצה
}

void MPU6050::read_accelerometer() {

    // בקשה של 6 בייט מידע מהאקס
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6);

    // קריאת המידע
    if (Wire.available() >= 6)
    {
        // משתנים לאכסון המידע הגולמי
        raw_accel_X = (Wire.read() /*high byte*/ << 8 | Wire.read() /*low byte*/);
        raw_accel_Y = (Wire.read() << 8 | Wire.read());
        raw_accel_Z = (Wire.read() << 8 | Wire.read());

        // תיקון הטעות הממוצעת
        accel_X_after_offset = raw_accel_X - accel_X_offset;
        accel_Y_after_offset = raw_accel_Y - accel_Y_offset;
        accel_Z_after_offset = raw_accel_Z - accel_Z_offset;

        // low pass filter
        accel_X_filtered = (trust_the_old_data * accel_X_filtered) + ((1 - trust_the_old_data) * accel_X_after_offset);
        accel_Y_filtered = (trust_the_old_data * accel_Y_filtered) + ((1 - trust_the_old_data) * accel_Y_after_offset);
        accel_Z_filtered = (trust_the_old_data * accel_Z_filtered) + ((1 - trust_the_old_data) * accel_Z_after_offset);
    }
}

void MPU6050::read_gyroscope() {

    // בקשה של 6 בייט מידע מהגיירו
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6);

    if (Wire.available() >= 6)
    {
        raw_gyro_X = (Wire.read() /*בייט גבוה*/ << 8 | Wire.read() /*בייט נמוך*/);
        raw_gyro_Y = (Wire.read() << 8 | Wire.read());
        raw_gyro_Z = (Wire.read() << 8 | Wire.read());

        // תיקון הטעות הממוצעת
        gyro_X_after_offset = raw_gyro_X - gyro_X_offset;
        gyro_Y_after_offset = raw_gyro_Y - gyro_Y_offset;
        gyro_Z_after_offset = raw_gyro_Z - gyro_Z_offset;
    }
}

void MPU6050::calculate_angles() {

    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0; // המרה לשניות
    last_time = current_time;

    // חישוב זוויות מהאקס
    float temp_roll = atan2(accel_Y_filtered, accel_Z_filtered);
    float temp_pitch = atan2(accel_X_filtered, sqrt(accel_Y_filtered * accel_Y_filtered + accel_Z_filtered * accel_Z_filtered));

    // fusing
    roll = ALPHA * (roll + gyro_X_after_offset * radian * dt) + (1 - ALPHA) * temp_roll;
    pitch = ALPHA * (pitch + gyro_Y_after_offset * radian * dt) + (1 - ALPHA) * temp_pitch;

    yaw += gyro_Z_after_offset * radian * dt; // אי אפשר למדוד דרך האקס לכן לא עושים fusing

    // טווח ברדיאנים
    if (yaw > PI) yaw -= 2.0 * PI;
    else if (yaw < -PI) yaw += 2.0 * PI; 
}
