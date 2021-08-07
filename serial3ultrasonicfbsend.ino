// *******************************************************************
//  Arduino Nano 5V example snippet for sending via serial
//********************************************************************
// ########################## ULTRASONIC - DEFINES ##########################
#include "NewPing.h"

#define TRIGGER_PIN_1  4
#define ECHO_PIN_1     4
#define TRIGGER_PIN_2  5
#define ECHO_PIN_2     5
#define TRIGGER_PIN_3  6
#define ECHO_PIN_3     6
#define TRIGGER_PIN_4  7
#define ECHO_PIN_4     7
#define TRIGGER_PIN_5  8
#define ECHO_PIN_5     8
#define TRIGGER_PIN_6  9
#define ECHO_PIN_6     9

#define MAX_DISTANCE 20

NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);
NewPing sonar4(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE);
NewPing sonar5(TRIGGER_PIN_5, ECHO_PIN_5, MAX_DISTANCE);
NewPing sonar6(TRIGGER_PIN_6, ECHO_PIN_6, MAX_DISTANCE);

// Define Variables

float duration1; // Stores First HC-SR04 pulse duration value
float duration2; // Stores Second HC-SR04 pulse duration value
float duration3; // Stores Third HC-SR04 pulse duration value
float duration4; // Stores Third HC-SR04 pulse duration value
float duration5; // Stores Third HC-SR04 pulse duration value
float duration6; // Stores Third HC-SR04 pulse duration value

float distance1; // Stores calculated distance in cm for First Sensor
float distance2; // Stores calculated distance in cm for Second Sensor
float distance3; // Stores calculated distance in cm for Third Sensor
float distance4; // Stores calculated distance in cm for Third Sensor
float distance5; // Stores calculated distance in cm for Third Sensor
float distance6; // Stores calculated distance in cm for Third Sensor

int iterations = 3;



//3-1
int forward = 0;
int backward = 0;
int fminimum;
int bminimum;

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(2,3);        // RX, TX

typedef struct{
   signed long start;
   signed long ccmd1;
   signed long ccmd2;
   signed long cspeedR_meas;
   signed long cspeedL_meas;
   signed long cbatVoltage;
   signed long cboardTemp;
   signed long ccmdLed;
   signed long cchecksum;
} cSerialFeedback;
cSerialFeedback cFeedback;
//SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
}

// ########################## SEND ############################

void Send3(signed long ucmd1, signed long ucmd2, signed long uspeedR_meas, signed long uspeedL_meas, signed long ubatVoltage, signed long uboardTemp, signed long ucmdLed)
{
    cFeedback.start          = (signed long)START_FRAME;
    cFeedback.ccmd1           = (signed long)ucmd1;
    cFeedback.ccmd2           = (signed long)ucmd2;
    cFeedback.cspeedR_meas    = (signed long)uspeedR_meas;
    cFeedback.cspeedL_meas    = (signed long)uspeedL_meas;
    cFeedback.cbatVoltage     = (signed long)ubatVoltage;
    cFeedback.cboardTemp      = (signed long)uboardTemp;
    cFeedback.ccmdLed     = (signed long)ucmdLed;
    cFeedback.cchecksum   = (signed long )(cFeedback.start ^ cFeedback.ccmd1 ^ cFeedback.ccmd2 ^ cFeedback.cspeedR_meas ^ cFeedback.cspeedL_meas 
                                           ^ cFeedback.cbatVoltage ^ cFeedback.cboardTemp ^ cFeedback.ccmdLed);
  
     HoverSerial.write((uint8_t *)&cFeedback, sizeof(cFeedback));
}

// ########################## LOOP ##########################
void loop(void)
{   
  delay(1);  
  duration1 = sonar1.ping_median(iterations);
  delay(1);
  duration2 = sonar2.ping_median(iterations);  
  delay(1);
  duration3 = sonar3.ping_median(iterations);
  delay(1);
  duration4 = sonar4.ping_median(iterations);
  delay(1);
  duration5 = sonar5.ping_median(iterations);
  delay(1);
  duration6 = sonar6.ping_median(iterations); 
  

  // Calculate the distances for both sensors
  
  distance1 = (duration1 / 2) * 0.0343;
  distance2 = (duration2 / 2) * 0.0343;
  distance3 = (duration3 / 2) * 0.0343;
  distance4 = (duration4 / 2) * 0.0343;
  distance5 = (duration5 / 2) * 0.0343;
  distance6 = (duration6 / 2) * 0.0343;
  
  // Send results to Serial Monitor
  
    Serial.print("D 1: ");

    if (distance1 >= 400 || distance1 <= 2) {
    Serial.print("0 ");
    }
    else {
    Serial.print(distance1);
    Serial.print(" cm ");
    }
    
    Serial.print("D 2: ");

    if (distance2 >= 400 || distance2 <= 2) {
    Serial.print("0 ");
    }
    else {
    Serial.print(distance2);
    Serial.print(" cm");
    }
    
    Serial.print("D 3: ");

    if (distance3 >= 400 || distance3 <= 2) {
    Serial.print("0 ");
    }
    else {
    Serial.print(distance3);
    Serial.print(" cm");
    }
    
    Serial.print("D 4: ");

    if (distance4 >= 400 || distance4 <= 2) {
    Serial.print("0 ");
    }
    else {
    Serial.print(distance4);
    Serial.print(" cm");
    }
    
    Serial.print("D 5: ");

    if (distance5 >= 400 || distance5 <= 2) {
    Serial.print("0 ");
    }
    else {
    Serial.print(distance5);
    Serial.print(" cm");
    }
    
    Serial.print("D 6: ");

    if (distance6 >= 400 || distance6 <= 2) {
    Serial.print("0 ");
    }
    else {
    Serial.print(distance6);
    Serial.print(" cm");
    }


   // 3 -1
    Serial.print(" D F: ");
    if ((distance1 >= 400 || distance1 <= 2) && (distance2 >= 400 || distance2 <= 2) && (distance3 >= 400 || distance3 <= 2)){
    forward = 0;
    Serial.print(forward);
    }
    else {
    fminimum = max(max(distance1,distance2),distance3);
    //Serial.print(fminimum);
    forward = 1;
    Serial.print(forward);
    }

    
    Serial.print(" D B: ");
    if ((distance6 >= 400 || distance6 <= 2) && (distance5 >= 400 || distance5 <= 2) && (distance4 >= 400 || distance4 <= 2)){
    backward = 0;
    Serial.print(backward);
    }
    else {
    //bminimum = max(max(distance6,distance5),distance4);
    //Serial.print(bminimum);
    backward = 1;
    Serial.print(backward);
    }
    
  Serial.println(" ");
  
   Send3(forward, backward, distance3, distance4, distance5,distance6,100); //send 7 different  values from here..
  delay(10);
}

// ########################## END ###########################
