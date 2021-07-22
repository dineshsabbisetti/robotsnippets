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
   signed long acmd1;
   signed long acmd2;
   signed long  aspeedR_meas;
   signed long  aspeedL_meas;
   signed long  abatVoltage;
   signed long  aboardTemp;
   signed long acmdLed;
   signed long achecksum;
} aSerialFeedback;
aSerialFeedback aFeedback;
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

void Send1(signed long  ucmd1, signed long  ucmd2, signed long uspeedR_meas, signed long uspeedL_meas, signed long ubatVoltage, signed long uboardTemp, signed long ucmdLed)
{
    aFeedback.start          = (signed long)START_FRAME;
    aFeedback.acmd1           = (signed long)ucmd1;
    aFeedback.acmd2           = (signed long)ucmd2;
    aFeedback.aspeedR_meas    = (signed long)uspeedR_meas;
    aFeedback.aspeedL_meas    = (signed long)uspeedL_meas;
    aFeedback.abatVoltage     = (signed long)ubatVoltage;
    aFeedback.aboardTemp      = (signed long)uboardTemp;
    aFeedback.acmdLed     = (signed long)ucmdLed;
    aFeedback.achecksum   = (signed long )(aFeedback.start ^ aFeedback.acmd1 ^ aFeedback.acmd2 ^ aFeedback.aspeedR_meas ^ aFeedback.aspeedL_meas 
                                           ^ aFeedback.abatVoltage ^ aFeedback.aboardTemp ^ aFeedback.acmdLed);
  
     HoverSerial.write((uint8_t *)&aFeedback, sizeof(aFeedback));
}

// ########################## LOOP ##########################
void loop(void)
{ 
  Send1(2100000000, 2100000000, 2100000000, 2100000000, 2100000000,2100000000,2100000000); //send 7 different  values from here..
  delay(10);
}

// ########################## END ###########################
