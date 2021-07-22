// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
// #define ADEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

//#include <SoftwareSerial.h>
//SoftwareSerial HoverSerial(2,3);        // RX, TX

// Global variables
uint8_t aidx = 0;                        // Index for new data pointer
uint16_t abufStartFrame;                 // Buffer Start Frame
byte *ap;                                // Pointer declaration for the new received data
byte aincomingByte;
byte aincomingBytePrev;

typedef struct{
   signed long start;
   signed long  acmd1;
   signed long  acmd2;
   signed long  aspeedR_meas;
   signed long  aspeedL_meas;
   signed long  abatVoltage;
   signed long  aboardTemp;
   signed long acmdLed;
   signed long achecksum;
} aSerialFeedback;
aSerialFeedback aFeedback;
aSerialFeedback aNewFeedback;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  Serial1.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
}

// ########################## RECEIVE ##########################
void Receive1()
{
    // Check for new data availability in the Serial buffer
    if (Serial1.available()) {
        aincomingByte    = Serial1.read();                                   // Read the incoming byte
        abufStartFrame = ((uint16_t)(aincomingByte) << 8) | aincomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef ADEBUG_RX
        Serial.print(aincomingByte);
        return;
    #endif

    // Copy received data
    if (abufStartFrame == START_FRAME) {                     // Initialize if new data is detected
        ap       = (byte *)&aNewFeedback;
        *ap++    = aincomingBytePrev;
        *ap++    = aincomingByte;
        aidx     = 2;  
    } else if (aidx >= 2 && aidx < sizeof(aSerialFeedback)) {  // Save the new received data
        *ap++    = aincomingByte; 
        aidx++;
    } 
    
    // Check if we reached the end of the package
    if (aidx == sizeof(aSerialFeedback)) {
        signed long achecksum;
        achecksum = (signed long)(aNewFeedback.start ^ aNewFeedback.acmd1 ^ aNewFeedback.acmd2 ^ aNewFeedback.aspeedR_meas ^ aNewFeedback.aspeedL_meas
                            ^ aNewFeedback.abatVoltage ^ aNewFeedback.aboardTemp ^ aNewFeedback.acmdLed);

        // Check validity of the new data
        if (aNewFeedback.start == START_FRAME && achecksum == aNewFeedback.achecksum) {
            // Copy the new data
            memcpy(&aFeedback, &aNewFeedback, sizeof(aSerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");   Serial.print(aFeedback.acmd1);
            Serial.print(" 2: ");  Serial.print(aFeedback.acmd2);
            Serial.print(" 3: ");  Serial.print(aFeedback.aspeedR_meas);
            Serial.print(" 4: ");  Serial.print(aFeedback.aspeedL_meas);
            Serial.print(" 5: ");  Serial.print(aFeedback.abatVoltage);
            Serial.print(" 6: ");  Serial.print(aFeedback.aboardTemp);
            Serial.print(" 7: ");  Serial.println(aFeedback.acmdLed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        aidx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    aincomingBytePrev = aincomingByte;
}

// ########################## LOOP ##########################

unsigned long iTimeSend1 = 0;

void loop(void)
{ 
  unsigned long timeNow1 = millis();

  // Check for new received data
  Receive1();
  
}

// ########################## END ##########################
