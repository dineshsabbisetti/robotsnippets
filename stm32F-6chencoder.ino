/* BLDC Hall Sensor read and calculation program */

/***************************** Variables *********************************/

#define CWL            -1      // Assign a value to represent clock wise rotation
#define CCWL            1      // Assign a value to represent counter-clock wise rotation

#define CWR             1      // Assign a value to represent clock wise rotation
#define CCWR           -1      // Assign a value to represent counter-clock wise rotation

bool HSU_Vall = digitalRead(PB12);    // Set the Ul sensor value as boolean and read initial state
bool HSV_Vall = digitalRead(PB13);    // Set the Vl sensor value as boolean and read initial state 
bool HSW_Vall = digitalRead(PB14);    // Set the Wl sensor value as boolean and read initial state 

bool HSU_Valr = digitalRead(PB15);    // Set the Ur sensor value as boolean and read initial state
bool HSV_Valr = digitalRead(PA8);    // Set the Vr sensor value as boolean and read initial state 
bool HSW_Valr = digitalRead(PA11);    // Set the Wr sensor value as boolean and read initial state 

int directl = 1;       // Integer variable to store BLDC rotation direction
int pulseCountl;       // Integer variable to store the pulse count

int directr = 1;       // Integer variable to store BLDC rotation direction
int pulseCountr;       // Integer variable to store the pulse count

/***************************** Setup *********************************/

void setup()
{
// Set digital pins 2, 3 and 4 as inputs
//LEFT
  pinMode(PB12, INPUT);      
  pinMode(PB13, INPUT);      
  pinMode(PB14, INPUT);
//RIGHT
  pinMode(PB15, INPUT);      
  pinMode(PA8, INPUT);      
  pinMode(PA11, INPUT);

// Set digital pins 2, 3 and 4 as interrupts that trigger on rising and falling edge changes. Call a function (i.e. HallSensorU) on change
//LEFT
  attachInterrupt(digitalPinToInterrupt(PB12), HallSensorUl, CHANGE);      
  attachInterrupt(digitalPinToInterrupt(PB13), HallSensorVl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PB14), HallSensorWl, CHANGE);
//RIGHT
  attachInterrupt(digitalPinToInterrupt(PB15), HallSensorUr, CHANGE);      
  attachInterrupt(digitalPinToInterrupt(PA8), HallSensorVr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PA11), HallSensorWr, CHANGE);

// Initialize the print monitor and set baud rate to 9600 
  Serial.begin(115200);
}

/*************************** Main Loop ******************************/

void loop()
{
//  Serial.print("LEFT ");
//  Serial.print("\t");
//  Serial.print("RIGHT ");
//  Serial.println();
//
//  Serial.print(HSU_Vall); Serial.print(HSV_Vall); Serial.print(HSW_Vall);        // Display Hall Sensor Values
//  Serial.print("\t");
//  Serial.print(HSU_Valr); Serial.print(HSV_Valr); Serial.print(HSW_Valr);        // Display Hall Sensor Values
//  Serial.println();
//
//  Serial.print(directl);                                                           // Display direction of rotation
//  Serial.print("\t");
//  Serial.print(directr);                                                           // Display direction of rotation
  
  Serial.println();
  Serial.print(pulseCountl);                                                       // Display the pulse count
  Serial.print("\t");
  Serial.print(pulseCountr);                                                       // Display the pulse count
 // Serial.println();
}

/************************ Interrupt Functions ***************************/
//LEFT
void HallSensorWl()
{
  HSW_Vall = digitalRead(PB14);         // Read the current W hall sensor value
  HSV_Vall = digitalRead(PB13);           // Read the current V (or U) hall sensor value 
  directl = (HSW_Vall == HSV_Vall) ? CWL : CCWL;     // Determine rotation direction (ternary if statement)
  pulseCountl = pulseCountl + (1 * directl);       // Add 1 to the pulse count
}

void HallSensorVl()
{
  HSV_Vall = digitalRead(PB13);
  HSU_Vall = digitalRead(PB12);         // Read the current U (or W) hall sensor value 
  directl = (HSV_Vall == HSU_Vall) ? CWL : CCWL;
  pulseCountl = pulseCountl + (1 * directl);
}

void HallSensorUl()
{
//  startTime = millis();
  HSU_Vall = digitalRead(PB12);
  HSW_Vall = digitalRead(PB14);         // Read the current W (or V) hall sensor value    
  directl = (HSU_Vall == HSW_Vall) ? CWL : CCWL;
  pulseCountl = pulseCountl + (1 * directl);
}

//RIGHT
void HallSensorWr()
{
  HSW_Valr = digitalRead(PA11);         // Read the current W hall sensor value
  HSV_Valr = digitalRead(PA8);           // Read the current V (or U) hall sensor value 
  directr = (HSW_Valr == HSV_Valr) ? CWR : CCWR;     // Determine rotation direction (ternary if statement)
  pulseCountr = pulseCountr + (1 * directr);       // Add 1 to the pulse count
}

void HallSensorVr()
{
  HSV_Valr = digitalRead(PA8);
  HSU_Valr = digitalRead(PB15);         // Read the current U (or W) hall sensor value 
  directr = (HSV_Valr == HSU_Valr) ? CWR : CCWR;
  pulseCountr = pulseCountr + (1 * directr);
}

void HallSensorUr()
{
//  startTime = millis();
  HSU_Valr = digitalRead(PB15);
  HSW_Valr = digitalRead(PA11);         // Read the current W (or V) hall sensor value    
  directr = (HSU_Valr == HSW_Valr) ? CWR : CCWR;
  pulseCountr = pulseCountr + (1 * directr);
}
