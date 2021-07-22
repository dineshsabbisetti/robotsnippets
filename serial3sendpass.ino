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
   Send3(0, 200000000, 2100000000, 2100000000, 2100000000,2100000000,2100000000); //send 7 different  values from here..
  delay(10);
}

// ########################## END ###########################
