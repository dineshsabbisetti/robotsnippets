
// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define HOVER_SERIAL_BAUD2   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define START_FRAME2         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      200         // [-] Maximum speed for testing

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

// Global variables
uint8_t idx2 = 0;                        // Index for new data pointer
uint16_t bufStartFrame2;                 // Buffer Start Frame
byte *p2;                                // Pointer declaration for the new received data
byte incomingByte2;
byte incomingBytePrev2;

typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct {
  uint16_t start2;
  int16_t  left2;
  int16_t  right2;
  uint16_t checksum2;
} SerialCommand2;
SerialCommand2 Command2;
SerialCommand2 NewCommand2;

typedef struct {
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
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  Serial1.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial2.begin(HOVER_SERIAL_BAUD2);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial1.write((uint8_t *) &Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer

  if (Serial1.available()) {
    incomingByte    = Serial1.read();                                   // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
    p       = (byte *)&NewFeedback;
    *p++    = incomingBytePrev;
    *p++    = incomingByte;
    idx     = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++    = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      //   Serial.print("1: ");   Serial.print(Feedback.cmd1);
      //    Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
     // Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
     // Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
     // Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
     // Serial.print(" 6: ");  Serial.println(Feedback.boardTemp);
      //   Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);

    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}


// ########################## RECEIVE2 ##########################
void Receive2()
{
  // Check for new data availability in the Serial buffer

  if (Serial2.available()) {
    incomingByte2    = Serial2.read();                                   // Read the incoming byte
    bufStartFrame2 = ((uint16_t)(incomingByte2) << 8) | incomingBytePrev2;       // Construct the start frame
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte2);
  return;
#endif

  // Copy received data
  if (bufStartFrame2 == START_FRAME2) {                     // Initialize if new data is detected
    p2       = (byte *)&NewCommand2;
    *p2++    = incomingBytePrev2;
    *p2++    = incomingByte2;
    idx2     = 2;
  } else if (idx2 >= 2 && idx2 < sizeof(SerialCommand2)) {  // Save the new received data
    *p2++    = incomingByte2;
    idx2++;
  }

  // Check if we reached the end of the package
  if (idx2 == sizeof(SerialCommand2)) {
    uint16_t checksum2;
    checksum2 = (uint16_t)(NewCommand2.start2 ^ NewCommand2.left2 ^ NewCommand2.right2);

    // Check validity of the new data
    if (NewCommand2.start2 == START_FRAME2 && checksum2 == NewCommand2.checksum2) {
      // Copy the new data
      memcpy(&Command2, &NewCommand2, sizeof(SerialCommand2));

      // Print data to built-in Serial
      Serial.print(" 1: ");  Serial.print(Command2.left2);
      Serial.print(" 2: ");  Serial.print(Command2.right2);

    } else {
      Serial.println("Non-valid data skipped2");
      Serial.print(" 1: ");  Serial.print(Command2.left2);
    }
    idx2 = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev2 = incomingByte2;
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
unsigned long iTimeSend2 = 0;

void loop(void)
{
  unsigned long timeNow = millis();
  unsigned long timeNow2 = millis();

  // Check for new received data
  Receive();
  Receive2();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  //Send(0, SPEED_MAX_TEST - 2*abs(iTest));
  Send(0, 100);   // send two differernt values from here

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
}

// ########################## END ##########################
