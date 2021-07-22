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
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
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

void Send(int16_t ucmd1, int16_t ucmd2, int16_t uspeedR_meas, int16_t uspeedL_meas, int16_t ubatVoltage, int16_t uboardTemp, uint16_t ucmdLed)
{
    Feedback.start          = (uint16_t)START_FRAME;
    Feedback.cmd1           = (int16_t)ucmd1;
    Feedback.cmd2           = (int16_t)ucmd2;
    Feedback.speedR_meas    = (int16_t)uspeedR_meas;
    Feedback.speedL_meas    = (int16_t)uspeedL_meas;
    Feedback.batVoltage     = (int16_t)ubatVoltage;
    Feedback.boardTemp      = (int16_t)ubatVoltage;
    Feedback.cmdLed     = (uint16_t)ucmdLed;
    Feedback.checksum   = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas 
                                           ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);
  
     HoverSerial.write((uint8_t *)&Feedback, sizeof(Feedback));
}

// ########################## LOOP ##########################
void loop(void)
{ 
  Send(0, 0, 100, 200, 300,400,500); //send 7 different  values from here..
  delay(10);
}

// ########################## END ###########################

