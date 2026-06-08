#include <Arduino.h>
#include "control.h"
#include "MPU6050.h"
#include "dronRF.h"


// ימין למעלה פין 3 
// ימין למטה פין 9 
// שמאל למעלה פין 5 
// שמאל למטה פין 6 


void control::setup(){

    // הגדרת פינים של המנועים כיציאה 
    pinMode(9, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);

    

    // mpu.setup();
    // dronRF.setup();
}

void control::loop() {

    unsigned long currentTime = micros();
    elapsedTime = (currentTime - previousTime) * 1e-6; // המרה לשניות 
    previousTime = currentTime;

    // כדי למנוע בעיות של חלוקה באפס במקרה שהלופ רץ מהר מדי.. זמן מינימלי שעבר בין לופים 
    if (elapsedTime <= 0.001f) {
    elapsedTime = 0.001f; // 10 מיקרושניות מינימום 
    }

    calculateError();
    calculatePID();
    AntiWindup();
    MotorMix();

}

void control::calculatePID() {
        
    // ROLL PID // 

    // p 
    RollPID_P = RollKp * rollError;

    // i
    // במצב שהטעות לא גדולה מדי 
    if (-3 < rollError && rollError < 3){
    RollPID_I += (RollKi * rollError) * elapsedTime;
    }

    // d - עם low pass filter
    float RollPID_D_raw = RollKd *((rollError - Rollprevious_error)/elapsedTime);
    RollPID_D = (trust_old_data * RollPID_D_filtered) + ((1 - trust_old_data) * RollPID_D_raw);
    RollPID_D_filtered = RollPID_D;

    // חיבור ה p i d  רול 
    RollTotalPID = RollPID_P + RollPID_I + RollPID_D;

    // PITCH PID //
    
    // p
    PitchPID_P = PitchKp * pitchError;

    // i
    if (-3 < pitchError && pitchError < 3){
    PitchPID_I += (PitchKi * pitchError) * elapsedTime;
    }

    // d - עם low pass filter
    float PitchPID_D_raw = PitchKd *((pitchError - Pitchprevious_error)/elapsedTime);
    PitchPID_D = (trust_old_data * PitchPID_D_filtered) + ((1 - trust_old_data) * PitchPID_D_raw);
    PitchPID_D_filtered = PitchPID_D;

    PitchTotalPID = PitchPID_P + PitchPID_I + PitchPID_D;

    // YAW PID // 

    // p
    YawPID_P = YawKp * yawError;

    // i
    if (-3 < yawError && yawError < 3){
    YawPID_I += (YawKi * yawError) * elapsedTime;
    }

    // d - עם low pass filter
    float YawPID_D_raw = YawKd *((yawError - Yawprevious_error)/elapsedTime);
    YawPID_D = (trust_old_data * YawPID_D_filtered) + ((1 - trust_old_data) * YawPID_D_raw);
    YawPID_D_filtered = YawPID_D;

    YawTotalPID = YawPID_P + YawPID_I + YawPID_D;

    // בסוף PID חייב להגביל טווח למנועים בשביל לא לשרוף אותם חלילה לכן: 
    RollTotalPID = constrain(RollTotalPID, -255.0, 255.0);
    PitchTotalPID = constrain(PitchTotalPID, -255.0, 255.0);
    YawTotalPID = constrain(YawTotalPID, -255.0, 255.0);
    ThrottleTotalPID = constrain(ThrottleTotalPID, 0.0, 255.0);

    // עדכון הטעות האחרונה 
    Rollprevious_error = rollError;
    Pitchprevious_error = pitchError;
    Yawprevious_error = yawError;
}

void control::MotorMix() { 
    
    if (targetThrottle < 0) {
        targetThrottle = 0;
    }

    
    //motor_output0 = targetThrottle + PitchTotalPID  + RollTotalPID  + YawTotalPID; 
    motor_output1 = targetThrottle + PitchTotalPID  - RollTotalPID - YawTotalPID; 
    //motor_output2 = targetThrottle - PitchTotalPID  - RollTotalPID + YawTotalPID; 
    //motor_output3 = targetThrottle - PitchTotalPID  + RollTotalPID - YawTotalPID;

    // ימין אחורי (מוריד זנב, מוריד ימין)
motor_output0 = targetThrottle + PitchTotalPID - RollTotalPID - YawTotalPID; 

// שמאל אחורי (מוריד זנב, מוריד שמאל)
// motor_output1 = targetThrottle + PitchTotalPID + RollTotalPID + YawTotalPID; 

// ימין קדמי/עליון (מוריד אף, מוריד ימין)
motor_output2 = targetThrottle - PitchTotalPID - RollTotalPID + YawTotalPID; 

// שמאל קדמי/עליון (מוריד אף, מוריד שמאל)
motor_output3 = targetThrottle - PitchTotalPID + RollTotalPID - YawTotalPID; 


// M0 = קדמי ימני = CW
//motor_output0 = targetThrottle - RollTotalPID - PitchTotalPID - YawTotalPID;

// M1 = אחורי ימני = CCW
//motor_output1 = targetThrottle - RollTotalPID + PitchTotalPID + YawTotalPID;

// M2 = קדמי שמאלי = CCW
//motor_output2 = targetThrottle + RollTotalPID - PitchTotalPID + YawTotalPID;

// M3 = אחורי שמאלי = CW
//motor_output3 = targetThrottle + RollTotalPID + PitchTotalPID - YawTotalPID;

    // הגבלת טווח הפלט למנועים 
    motor_output0 = constrain(motor_output0, 0.0, 255.0);
    motor_output1 = constrain(motor_output1, 0.0, 255.0);
    motor_output2 = constrain(motor_output2, 0.0, 255.0);
    motor_output3 = constrain(motor_output3, 0.0, 255.0);

    analogWrite(9, motor_output0); 
    analogWrite(6, motor_output1); 
    analogWrite(3, motor_output2);
    analogWrite(5, motor_output3); 

}

 void control::calculateError() {

    currentRoll = mpu.GET_ROLL();
    currentPitch = mpu.GET_PITCH();
    currentYaw = mpu.GET_YAW();

    targetRoll = dronRF.getTXwantroll();
    targetPitch = dronRF.getTXwantpitch();
    targetYaw = dronRF.getTXwantyaw();
    targetThrottle = dronRF.getTXwantthrottle();

// חישוב הטעות 
    rollError = targetRoll - currentRoll;
    pitchError = targetPitch - currentPitch;
    yawError = targetYaw - currentYaw;

// נרמול טעוץ 
    while(rollError > 180) rollError -= 360;
    while(rollError < -180) rollError += 360;

    while(pitchError > 180) pitchError -= 360;
    while(pitchError < -180) pitchError += 360;

    while(yawError > 180) yawError -= 360;
    while(yawError < -180) yawError += 360;

}

void control::AntiWindup() {

    const float I_MAX = 20.0f;  // הגבלת ה i כדי למנוע Windup 

    // פונקציה להגבלה - מונע מה-i (האינטגרל) להמשיך לגדול כאשר הטעות גדולה מדי
    auto clampValue = [](float& value, float min_val, float max_val) {
        if (value > max_val) value = max_val;
        if (value < min_val) value = min_val;
    };

    clampValue(RollPID_I, -I_MAX, I_MAX);
    clampValue(PitchPID_I, -I_MAX, I_MAX);
    clampValue(YawPID_I, -I_MAX, I_MAX);
    // בעצם מונע מה i (האינטגרל) להמשיך לגדול ולגדול כאשר הטעות גדולה מדי 
}
