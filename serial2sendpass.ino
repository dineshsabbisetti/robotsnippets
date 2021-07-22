// *******************************************************************
//  Arduino Nano 5V example snippet for sending via serial
//********************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(2,3);        // RX, TX

typedef struct{
   signed long start;
   signed long bcmd1;
   signed long bcmd2;
   signed long bspeedR_meas;
   signed long bspeedL_meas;
   signed long bbatVoltage;
   signed long bboardTemp;
   signed long bcmdLed;
   signed long bchecksum;
} bSerialFeedback;
bSerialFeedback bFeedback;
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

void Send2(signed long  ucmd1, signed long  ucmd2, signed long uspeedR_meas, signed long uspeedL_meas, signed long ubatVoltage, signed long uboardTemp, signed long ucmdLed)
{
    bFeedback.start          = (signed long)START_FRAME;
    bFeedback.bcmd1           = (signed long)ucmd1;
    bFeedback.bcmd2           = (signed long)ucmd2;
    bFeedback.bspeedR_meas    = (signed long)uspeedR_meas;
    bFeedback.bspeedL_meas    = (signed long)uspeedL_meas;
    bFeedback.bbatVoltage     = (signed long)ubatVoltage;
    bFeedback.bboardTemp      = (signed long)uboardTemp;
    bFeedback.bcmdLed     = (signed long)ucmdLed;
    bFeedback.bchecksum   = (signed long )(bFeedback.start ^ bFeedback.bcmd1 ^ bFeedback.bcmd2 ^ bFeedback.bspeedR_meas ^ bFeedback.bspeedL_meas 
                                           ^ bFeedback.bbatVoltage ^ bFeedback.bboardTemp ^ bFeedback.bcmdLed);
  
     HoverSerial.write((uint8_t *)&bFeedback, sizeof(bFeedback));
}

// ########################## LOOP ##########################
void loop(void)
{ 
  Send2( 10, 0, 2100000000, 2100000000, 2100000000,2100000000,2100000000); //send 7 different  values from here..
  delay(10);
}

// ########################## END ###########################
