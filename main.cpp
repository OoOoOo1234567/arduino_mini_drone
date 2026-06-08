#include <Arduino.h>
#include "control.h"
#include <Wire.h>


///////////////////////////////////////////////////
// הגדרת הפינים של המנועים
//const int motors[] = {5, 6, 9, 3};
//const int numMotors = 4;
////////////////////////////////////////////////////


MPU6050 mpu;
RXnrf dronRF(2, 4);
control droneControl;


void setup() {

    Serial.begin(115200);

    delay(1000); // מספיק 1 שנייה 

    dronRF.setup();
    
    // לא עובד ולכן החלפתי את הפין של המנוע מ10 ל פין 3.
/////////
   // pinMode(10, OUTPUT);  // פין 10 הוא הSS SLAVE SELECT בארדואינו שלי ובגלל שחיברתי את המנוע עליו זה יצר בעיות ולכן צריך להגדיר אותו כאוטפוט ולבטל כמה שאפשר את הSS שלו. 
/////////

  // mpu.setup();
 //  droneControl.setup();

 /////////////////////////////////////////////
  //for (int i = 0; i < numMotors; i++) {
  //  pinMode(motors[i], OUTPUT);
  //}
 ///////////////////////////////////////
}

void loop() {

    dronRF.loop();
   // mpu.loop();
   // droneControl.loop();


////////////////////////////////////////////////////
//       הדפסות לבדיקות 
////////////////////////////////////////////////////
/*
 for (int i = 0; i < numMotors; i++) {
    analogWrite(motors[i], 255); 
    delay(500); // חצי שנייה עבודה
    
    // כיבוי
    analogWrite(motors[i], 0);
    delay(200); // הפסקה קצרה בין מנוע למנוע
  }

  for (int i = 0; i < numMotors; i++) {
    analogWrite(motors[i], 255);
  }
  
  delay(1500);

  // כיבוי של כולם
  for (int i = 0; i < numMotors; i++) {
    analogWrite(motors[i], 0);
  }

  delay(2000);
*/
/*
    Serial.print("Roll: ");
    Serial.print(  mpu.GET_ROLL());
    Serial.print("°  Pitch: ");
    Serial.print(  mpu.GET_PITCH());
    Serial.print("°  Yaw: ");
    Serial.println(  mpu.GET_YAW());

    delay(1000);

*/
/*
Serial.print("TXwantthrottle: "); Serial.println(dronRF.getTXwantthrottle()); 
Serial.print("TXwantroll: "); Serial.println(dronRF.getTXwantroll());
Serial.print("TXwantpitch: "); Serial.println(dronRF.getTXwantpitch());
Serial.print("TXwantyaw: "); Serial.println(dronRF.getTXwantyaw());

delay(1000);
*/

}
