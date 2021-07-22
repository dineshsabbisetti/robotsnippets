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
   signed long cmd1;
   signed long cmd2;
   signed long  speedR_meas;
   signed long  speedL_meas;
   signed long  batVoltage;
   signed long  boardTemp;
   signed long cmdLed;
   signed long checksum;
} SerialFeedback;
SerialFeedback Feedback;
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

void Send(signed long  ucmd1, signed long  ucmd2, signed long uspeedR_meas, signed long uspeedL_meas, signed long ubatVoltage, signed long uboardTemp, signed long ucmdLed)
{
    Feedback.start          = (signed long)START_FRAME;
    Feedback.cmd1           = (signed long)ucmd1;
    Feedback.cmd2           = (signed long)ucmd2;
    Feedback.speedR_meas    = (signed long)uspeedR_meas;
    Feedback.speedL_meas    = (signed long)uspeedL_meas;
    Feedback.batVoltage     = (signed long)ubatVoltage;
    Feedback.boardTemp      = (signed long)uboardTemp;
    Feedback.cmdLed     = (signed long)ucmdLed;
    Feedback.checksum   = (signed long )(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas 
                                           ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);
  
     HoverSerial.write((uint8_t *)&Feedback, sizeof(Feedback));
}

// ########################## LOOP ##########################
void loop(void)
{ 
  Send(2100000000, 2100000000, 2100000000, 2100000000, 2100000000,2100000000,2100000000); //send 7 different  values from here..
  delay(10);
}

// ########################## END ###########################
