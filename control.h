#include <Arduino.h>
#include "MPU6050.h"
#include "dronRF.h"

extern MPU6050 mpu;
extern RXnrf dronRF;

class control {

// low pass filter  לD 
const float trust_old_data = 0.75;  // Low pass filter  (0.75 = 75% ישן, 25% חדש)

// סהכ הPID ולכל זוות הp i d  שלה 
/////////////////////////////////
float RollTotalPID = 0.0;

float RollPID_P = 0.0;
float RollPID_I = 0.0;
float RollPID_D = 0.0;

float RollPID_D_filtered = 0.0;  // D term לאחר סינון
//
float PitchTotalPID = 0.0;

float PitchPID_P = 0.0;
float PitchPID_I = 0.0;
float PitchPID_D = 0.0;

float PitchPID_D_filtered = 0.0;  // D term לאחר סינון
//
float YawTotalPID = 0.0;

float YawPID_P = 0.0;
float YawPID_I = 0.0;
float YawPID_D = 0.0;

float YawPID_D_filtered = 0.0;  // D term לאחר סינון
//
float ThrottleTotalPID = 0.0; 
/////////////////////////////////

float targetRoll = 0.0;
float targetPitch = 0.0;
float targetYaw = 0.0;
float targetThrottle = 0.0;

float currentRoll = 0.0;
float currentPitch = 0.0;
float currentYaw = 0.0;

float rollError = 0.0;
float pitchError = 0.0;
float yawError = 0.0;

float Rollprevious_error = 0.0;
float Pitchprevious_error = 0.0;
float Yawprevious_error = 0.0;

////////
// קבועי PID
// roll
float RollKp = 0.3; // Proportional gain
float RollKi = 0.03; // Integral gain
float RollKd = 0.08; // Derivative gain

// pitch
float PitchKp = 0.3; 
float PitchKi = 0.03; 
float PitchKd = 0.08; 

// yaw
float YawKp = 0.2; 
float YawKi = 0.01; 
float YawKd = 0.04; 
////////
// כמה לכל מנוע 
float motor_output0 = 0.0;
float motor_output1 = 0.0;
float motor_output2 = 0.0;
float motor_output3 = 0.0;

// זמן שעבר בין לופים
float elapsedTime = 0.0;

unsigned long previousTime = 0; // זמן הלופ הקודם 

public:
    void setup();
    void loop();
    void calculatePID();
    void MotorMix();
    void calculateError();
    void AntiWindup();

};