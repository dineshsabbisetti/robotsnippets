// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
// #define CDEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

//#include <SoftwareSerial.h>
//SoftwareSerial HoverSerial(2,3);        // RX, TX






// Global variables
uint8_t bidx = 0;                        // Index for new data pointer
uint16_t bbufStartFrame;                 // Buffer Start Frame
byte *bp;                                // Pointer declaration for the new received data
byte bincomingByte;
byte bincomingBytePrev;
//new 
 signed long  pulseCountl;       // Integer variable to store the pulse count
 signed long  pulseCountr;       // Integer variable to store the pulse count
//new

// Global variables 3 
//new
unsigned long receivecompass = 0;
//new
uint8_t cidx = 0;                        // Index for new data pointer
uint16_t cbufStartFrame;                 // Buffer Start Frame
byte *cp;                                // Pointer declaration for the new received data
byte cincomingByte;
byte cincomingBytePrev;


//struct 2

typedef struct{
   signed long start;
   signed long  bcmd1;
   signed long  bcmd2;
   signed long  bspeedR_meas;
   signed long  bspeedL_meas;
   signed long  bbatVoltage;
   signed long  bboardTemp;
   signed long bcmdLed;
   signed long bchecksum;
} bSerialFeedback;
bSerialFeedback bFeedback;
bSerialFeedback bNewFeedback;

//struct 3
typedef struct{
   signed long start;
   signed long  ccmd1;
   signed long  ccmd2;
   signed long  cspeedR_meas;
   signed long  cspeedL_meas;
   signed long  cbatVoltage;
   signed long  cboardTemp;
   signed long ccmdLed;
   signed long cchecksum;
} cSerialFeedback;
cSerialFeedback cFeedback;
cSerialFeedback cNewFeedback;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");
  
  Serial2.begin(HOVER_SERIAL_BAUD);
  Serial3.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
}

// ########################## RECEIVE2 ##########################
void Receive2()
{
    // Check for new data availability in the Serial buffer
    if (Serial2.available()) {
        bincomingByte    = Serial2.read();                                   // Read the incoming byte
        bbufStartFrame = ((uint16_t)(bincomingByte) << 8) | bincomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef BDEBUG_RX
        Serial.print(bincomingByte);
        return;
    #endif

    // Copy received data
    if (bbufStartFrame == START_FRAME) {                     // Initialize if new data is detected
        bp       = (byte *)&bNewFeedback;
        *bp++    = bincomingBytePrev;
        *bp++    = bincomingByte;
        bidx     = 2;  
    } else if (bidx >= 2 && bidx < sizeof(bSerialFeedback)) {  // Save the new received data
        *bp++    = bincomingByte; 
         bidx++;
    } 
    
    // Check if we reached the end of the package
    if (bidx == sizeof(bSerialFeedback)) {
        signed long bchecksum;
        bchecksum = (signed long)(bNewFeedback.start ^ bNewFeedback.bcmd1 ^ bNewFeedback.bcmd2 ^ bNewFeedback.bspeedR_meas ^ bNewFeedback.bspeedL_meas
                            ^ bNewFeedback.bbatVoltage ^ bNewFeedback.bboardTemp ^ bNewFeedback.bcmdLed);

        // Check validity of the new data
        if (bNewFeedback.start == START_FRAME && bchecksum == bNewFeedback.bchecksum) {
            // Copy the new data
            memcpy(&bFeedback, &bNewFeedback, sizeof(bSerialFeedback));

            // Print data to built-in Serial
//            Serial.print("1: ");   Serial.print(bFeedback.bcmd1);
//            Serial.print(" 2: ");  Serial.print(bFeedback.bcmd2);
//            Serial.print(" 3: ");  Serial.print(bFeedback.bspeedR_meas);
//            Serial.print(" 4: ");  Serial.print(bFeedback.bspeedL_meas);
//            Serial.print(" 5: ");  Serial.print(bFeedback.bbatVoltage);
//            Serial.print(" 6: ");  Serial.print(bFeedback.bboardTemp);
//            Serial.print(" 7: ");  Serial.println(bFeedback.bcmdLed);
            
            pulseCountr=bFeedback.bspeedR_meas;       // Integer variable to store the pulse count
            pulseCountl=bFeedback.bspeedL_meas;       // Integer variable to store the pulse count
  
            Serial.println();
            Serial.print(pulseCountl);                                                       // Display the pulse count
            Serial.print("\t");
            Serial.print(pulseCountr);    
        } else {
          Serial.println("Non-valid data skipped");
        }
        bidx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    bincomingBytePrev = bincomingByte;
}

// ########################## RECEIVE3 ##########################
void Receive3()
{
    // Check for new data availability in the Serial buffer
    if (Serial3.available()) {
        cincomingByte    = Serial3.read();                                   // Read the incoming byte
        cbufStartFrame = ((uint16_t)(cincomingByte) << 8) | cincomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef CDEBUG_RX
        Serial.print(cincomingByte);
        return;
    #endif

    // Copy received data
    if (cbufStartFrame == START_FRAME) {                     // Initialize if new data is detected
        cp       = (byte *)&cNewFeedback;
        *cp++    = cincomingBytePrev;
        *cp++    = cincomingByte;
        cidx     = 2;  
    } else if (cidx >= 2 && cidx < sizeof(cSerialFeedback)) {  // Save the new received data
        *cp++    = cincomingByte; 
        cidx++;
    } 
    
    // Check if we reached the end of the package
    if (cidx == sizeof(cSerialFeedback)) {
        signed long cchecksum;
        cchecksum = (signed long)(cNewFeedback.start ^ cNewFeedback.ccmd1 ^ cNewFeedback.ccmd2 ^ cNewFeedback.cspeedR_meas ^ cNewFeedback.cspeedL_meas
                            ^ cNewFeedback.cbatVoltage ^ cNewFeedback.cboardTemp ^ cNewFeedback.ccmdLed);

        // Check validity of the new data
        if (cNewFeedback.start == START_FRAME && cchecksum == cNewFeedback.cchecksum) {
            // Copy the new data
            memcpy(&cFeedback, &cNewFeedback, sizeof(cSerialFeedback));

            // Print data to built-in Serial
           // Serial.print("1: ");   Serial.print(cFeedback.ccmd1);
           // Serial.print(" 2: ");  Serial.print(cFeedback.ccmd2);
           // Serial.print(" 3: ");  Serial.print(cFeedback.cspeedR_meas);
           // Serial.print(" 4: ");  Serial.print(cFeedback.cspeedL_meas);
           // Serial.print(" 5: ");  Serial.print(cFeedback.cbatVoltage);
           // Serial.print(" 6: ");  Serial.print(cFeedback.cboardTemp);
           // Serial.print(" 7: ");  Serial.println(cFeedback.ccmdLed);
              Serial.println(" heading: "); 
             Serial.println(receivecompass);
            receivecompass = cFeedback.ccmdLed;
        } else {
          Serial.println("Non-valid data skipped");
        }
        cidx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    cincomingBytePrev = cincomingByte;
}

// ########################## LOOP ##########################

unsigned long iTimeSend3 = 0;
unsigned long iTimeSend2 = 0;

void loop(void)
{ 
  unsigned long timeNow2 = millis();
  unsigned long timeNow3 = millis();
  

  // Check for new received data
  Receive2();
  Receive3();
  
}

// ########################## END ##########################
