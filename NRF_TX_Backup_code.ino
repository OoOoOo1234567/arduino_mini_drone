#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>

// הגדרת הפינים עבור ה-R4 (וודאי שזה תואם לחיבור הפיזי שלך)
RF24 radio(6, 7); 

byte addresses[][6] = {"1Node", "2Node"};

// מבנה הנתונים - חובה להשתמש ב-packed בגלל ה-R4
struct MyData {
  int16_t throttle;
  int16_t roll; 
  int16_t pitch; 
  int16_t yaw; 
} __attribute__((packed)); // שלא יהיה רווח 

MyData joystickData;

void setup() {
  Serial.begin(115200);
  Serial.println("Transmitter Ready - Sending Joystick Data");

  radio.begin();

  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_2MBPS); // מהירות גבוהה
  radio.setChannel(124);         // ערוץ 124

  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
}

void loop() {
  // 1. קריאת הערכים מהג'ויסטיקים
  joystickData.throttle = analogRead(A0);
  joystickData.roll     = analogRead(A1) - 512;
  joystickData.pitch    = analogRead(A2) - 512;
  joystickData.yaw      = analogRead(A3) - 512;

//  תיקון רעשים 
if (joystickData.throttle < 5) joystickData.throttle = 0;
if (joystickData.roll > -5 && joystickData.roll < 5) joystickData.roll = 0;
if (joystickData.pitch > -10 && joystickData.pitch < 10) joystickData.pitch = 0;
if (joystickData.yaw > -10 && joystickData.yaw < 10) joystickData.yaw = 0; 


  // 2. הפסקת האזנה לטובת שידור
  radio.stopListening(); 

  // 3. שליחת ה-struct המלא
  Serial.print("Sending -> T:"); Serial.print(joystickData.throttle);
  
  if (!radio.write( &joystickData, sizeof(MyData) )) {
    Serial.println(" | No acknowledgement - Fail");    
  } else {
    Serial.println(" | Sent OK!");
  }

  // 4. המתנה לתגובה מהרחפן (Ack Payload)
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
    Serial.println("No response from Drone (Timeout)");
  } else {
    unsigned char response;
    radio.read( &response, sizeof(unsigned char) );
    Serial.print("Drone says: ");
    Serial.println(response);
  }

  // קצב שידור שמתאים לרחפן - 50ms (20 פעמים בשנייה)
  delay(50); 
}





///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <Arduino.h>
#include <SPI.h>

// מנועים 
// ימין למעלה פין 6 
// ימין למטה פין 7ל 
// שמאל למעלה פין 8 
// שמאל למטה פין 9 

//  גויסטיקים 
// throttle - A0
// roll - A1
// pitch - A2
// yaw - A3

const uint8_t CE_PIN = 8; // Chip Enable pin אחראי על הפעלת השידור 
const uint8_t CSN_PIN = 9; // Chip Select Not pin 

// רגיסטורים של ה nRF24L01+
const uint8_t CONFIG = 0x00; //  רגיסטור לקונפיגריישן
const uint8_t RF_CH = 0x05; // RF Channel register
const uint8_t RF_SETUP = 0x06; //   RF Setup register הספק, מהירות העברה
const uint8_t STATUS = 0x07; // רגיסטור סטטוס. דגלים 
const uint8_t RX_ADDR_P0 = 0x0A; // כתובת קבלה צינור 0 
const uint8_t TX_ADDR = 0x10; // כתובת שידור
const uint8_t RX_PW_P0 = 0x11; // Payload width for pipe 0
const uint8_t FIFO_STATUS = 0x17; // FIFO status register האם מלא או ריק
const uint8_t W_TX_PAYLOAD = 0xA0; // פקודת כתיבת payload לשידור 


struct MyData {
int16_t throttle; //   מספר שלם (תופס 2 בייטים) 
int16_t roll; 
int16_t pitch; 
int16_t yaw; 
} __attribute__((packed)); // שלא יהיה רווח בין המידע שעובר וככה ישלח בצורה טובה יותר (בייט אחרי בייט בלי רווחים) 
MyData data;

// הגדרות SPI בעזרת הבנאי של ספריית SPI 
SPISettings mySettings(1000000, MSBFIRST, SPI_MODE0);

uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

const long loopInterval = 20; 
unsigned long lastSendTime = 0;

void writeRegister(uint8_t reg, uint8_t value);
void addres(uint8_t regist);
void sendData();

void setup() {

pinMode(53, OUTPUT); // שומר על ה-Mega במצב Master ל-SPI 

Serial.begin(115200);

pinMode(CE_PIN, OUTPUT); // המצב של הפינים 
pinMode(CSN_PIN, OUTPUT);

digitalWrite(CSN_PIN, HIGH); //  גבוה הפין לא פעיל 
digitalWrite(CE_PIN, LOW); //  נמוך הרדיו כבוי 

delay (10); // דילאי כי למה לא 

SPI.begin();

// ביטול הגדרות שמיותרות 
// writeRegister(0x01, 0x00); // Disable Auto-ACK 
// writeRegister(0x1C, 0x00); // Disable Dynamic Payload 
// writeRegister(0x1D, 0x00); // Disable Feature register (הגדרות נוספות) 
writeRegister(0x01, 0x3F);

// כתובת שידור וקבלה זהה 
addres(RX_ADDR_P0);
addres(TX_ADDR);

writeRegister(RF_CH, 0x4c); // ערוץ 76 

writeRegister(RF_SETUP, 0x26); // מהירות 250kbps, הספק מקסימלי 

writeRegister(RX_PW_P0, sizeof(MyData)); // כמה לצפות לקבל בכל שליחה גודל של שמונה בייטים 


//////////////////////////////////////////////////////////
SPI.beginTransaction(mySettings);
digitalWrite(CSN_PIN, LOW); // "פותח" את המודול להקשבה 

// אנחנו שולחים את כתובת הרגיסטר שאנחנו רוצים לקרוא (0x00)
SPI.transfer(CONFIG); 

// עכשיו אנחנו שולחים "בייט זבל" (0xFF) כדי לייצר פעימות שעון, 
// ובתמורה המודול מחזיר לנו את הערך ששמור אצלו ברגיסטר.
uint8_t check = SPI.transfer(0xFF); 

digitalWrite(CSN_PIN, HIGH); // "סוגר" את המודול
SPI.endTransaction();


// ניקוי זיכרון השידור 
SPI.beginTransaction(mySettings);
digitalWrite(CSN_PIN, LOW);

SPI.transfer(0xE1); // מנקה את זיכרון השידור (FLUSH_TX) 

digitalWrite(CSN_PIN, HIGH);
SPI.endTransaction(); 
delay(3);

writeRegister(CONFIG, 0x0E); //  מצב שידור 

delay(2.5); // המתנה לסיום הפעולה (על פי המיגע על הרכיב יש 1.5 מילישניות עד הוא באמת מתעורר ) 

///////////////////////////////////////////////////////
Serial.print("Check value: 0x");
Serial.println(check, HEX);
///////////////////////////////////////////////////////

}

void loop() {

  // קריאת ערכים מהג'ויסטיקים 
data.throttle = analogRead(A0);
data.roll = analogRead(A1) - 512;
data.pitch = analogRead(A2) - 512;
data.yaw = analogRead(A3) - 512;

//  תיקון רעשים 
if (data.throttle < 5) data.throttle = 0;
if (data.roll > -5 && data.roll < 5) data.roll = 0;
if (data.pitch > -5 && data.pitch < 5) data.pitch = 0;
if (data.yaw > -5 && data.yaw < 5) data.yaw = 0; 


///////////////////////////////////////////////////////////////////////////////
/*// הדפסת ערכי ג'ויסטיק למוניטור
  Serial.println("Joysticks -> T:"); Serial.print(data.throttle);
  Serial.println(" R:"); Serial.print(data.roll);
  Serial.println(" P:"); Serial.print(data.pitch);
  Serial.println(" Y:"); Serial.print(data.yaw);
  Serial.println();
*/
//delay(100); // דילאי קטן כדי לא להעמיס על המוניטור
///////////////////////////////////////////////////////////////////////////////

// כל 20 MS מידע ישלח 
unsigned long Time = millis();
if (Time - lastSendTime >= loopInterval) {
lastSendTime = Time;
sendData(); // שליחת המידע לרחפן 
}
}

// פונקציית שליחת המידע לרחפן 
void sendData() {

writeRegister(STATUS, 0x70);

SPI.beginTransaction(mySettings); // הגדרות לצינור SPI 
digitalWrite(CSN_PIN, LOW); // הפעלה של הפין 

SPI.transfer(W_TX_PAYLOAD); // אומר למודל שמה שאני שולח עכשיו זה לא פקודה אלא דברים שאני רוצה שתשדר לרחפן 

//מעביר את המידע לבייטים בודדים כי ככה עובד SPI  
uint8_t* ptr = (uint8_t*)&data; 

/* ספריית ה-SPI יודעת לעבוד רק עם בייטים בודדים (כמו סבל שיודע להרים רק ארגז אחד בכל פעם).
   לכן, כדי לשלוח מבנה נתונים (Struct) שכולל כמה שדות שונים, אנחנו צריכים לפרק אותו לבייטים בודדים.
הפקודה (uint8_t*)ptr אומרת למעבד: "אל תתייחס לזה כאל מבנה נתונים מורכב. תתייחס לזה כאל שורה ארוכה של בייטים בודדים שמתחילה בכתובת של המחסן הזה בזיכרון"
 */

// תעבור ביט ביט על Struct ותשלח כל בייט לבד 
for (uint8_t i = 0; i < sizeof(MyData); i++) {
SPI.transfer(ptr[i]); // שולח את ה-Struct בבייטים
}

digitalWrite(CSN_PIN, HIGH); // כיבוי של הפין
SPI.endTransaction(); // סיום העסקה 

// הפעלת השידור 
digitalWrite(CE_PIN, HIGH);

delayMicroseconds(20); 

digitalWrite(CE_PIN, LOW);

delayMicroseconds(5); // המתנה לסיום השידור 

////////////////////////////////////////////////////////////////////////
delay(1); 
SPI.beginTransaction(mySettings);
digitalWrite(CSN_PIN, LOW);
uint8_t status = SPI.transfer(0xFF); // קריאת הסטטוס מהמודול
digitalWrite(CSN_PIN, HIGH);
SPI.endTransaction();

if (status & 0x20) {
Serial.println(" | Status: Sent ok (drone got it)");
} else if (status & 0x10) {
Serial.println(" | Status: not working)");
} else {
Serial.print(" | Status: 0x"); Serial.println(status, HEX);
}
////////////////////////////////////////////////////////////////////////

SPI.beginTransaction(mySettings);
digitalWrite(CSN_PIN, LOW);

uint8_t currentStatus = SPI.transfer(0xFF); //  קריאת הסטטוס מהמודול

digitalWrite(CSN_PIN, HIGH);
SPI.endTransaction();

// ניקו דגלים 
writeRegister(STATUS, 0x70);


if (currentStatus & 0x01) { // ביט TX_FULL דלוק
    SPI.beginTransaction(mySettings);
    digitalWrite(CSN_PIN, LOW);
    SPI.transfer(0xE1); // FLUSH_TX
    digitalWrite(CSN_PIN, HIGH);
    SPI.endTransaction();
  }
}

// פונקציה שעוזרת לכתוב לרגיסטר מסוים ** המידע בגודל 8 ביט ** 
void writeRegister(uint8_t reg, uint8_t value) {

SPI.beginTransaction(mySettings); // הגדרות לצינור SPI 
digitalWrite(CSN_PIN, LOW); // הפעלה של הפין 

SPI.transfer(0x20 | reg); // כתיבה לרגיסטור 
SPI.transfer(value); // הערך לכתיבה 

digitalWrite(CSN_PIN, HIGH); // כיבוי של הפין
SPI.endTransaction(); // סיום העסקה 
delay(3); // המתנה לסיום הפעולה 
}

// פונקציה שעוזרת לכתוב לרגיסטר כתובת (5 בתים) 
void addres(uint8_t regist){

SPI.beginTransaction(mySettings);
digitalWrite(CSN_PIN, LOW); 

SPI.transfer(0x20 | regist); 
for (int i = 0; i < 5; i++){
SPI.transfer(address[i]);
}

digitalWrite(CSN_PIN, HIGH);
SPI.endTransaction();
delay(5);
}
