#define WARNING_MESSAGE "AT$PD" // AT$PD=21;LDW:1<0x0D><0x0A>
#define DELAY_TIME 10 // unit: ms
#define BUFFER_SIZE 10 // unit: byte

// Input Pin
#define START_SWITCH_INPIN 2           // 開關

// Output Pin
#define EXECUTING_LED_OUTPIN 3

boolean initFlag = false;
boolean gpsFlag = false;
boolean directionFlag = false;

unsigned char warningCount = 0;

char recvBuffer[BUFFER_SIZE] = {0};
char gpsData1[6] = {'$', 'G', 'P', 'R', 'M', 'C'}; // "$GPRMC"
char gpsData2[6] = {'$', 'G', 'P', 'G', 'G', 'A'}; // "$GPGGA"

unsigned char tempPositiveVal = 0;
unsigned char tempNegativeVal = 0;
unsigned char positiveVal = 0;
unsigned char negativeVal = 0;

void setup() 
{
  setupInit();
}

void loop() 
{
  initFlow();

  if (digitalRead(START_SWITCH_INPIN) == LOW) // 開
  {
    /***** Arduino is running *****/
    digitalWrite(EXECUTING_LED_OUTPIN, HIGH); // Turn on LED
    
    /***** Receive Data *****/
    recvUART(); // 10 ms

    /***** Send Warning *****/
    sendWarning(); // 100 ms

    delay(DELAY_TIME); // 10 ms
  }
  else  // 關
  {
    resetStatus();
    delay(500); // 500 ms
  }
}

void setupInit()
{
  // Input Pin
  pinMode(START_SWITCH_INPIN, INPUT_PULLUP);

  // Output Pin
  pinMode(EXECUTING_LED_OUTPIN, OUTPUT);
  
  Serial.begin(9600);   // USB Debug
  Serial1.begin(9600);  // UART Communication
}

void initFlow()
{
  if (initFlag == false)
  {
    // Output Pin
    digitalWrite(EXECUTING_LED_OUTPIN, LOW);
    delay(200);
    
    for (int i = 0; i < 3 ; i++) 
    {
      digitalWrite(EXECUTING_LED_OUTPIN, HIGH);
      delay(200);
      digitalWrite(EXECUTING_LED_OUTPIN, LOW);
      delay(200);
    }

    initFlag = true;
  }
}

void resetStatus()
{ 
  // Output Pin reset
  digitalWrite(EXECUTING_LED_OUTPIN, LOW);

  // Count reset
  warningCount = 0;

  // Flag reset
  gpsFlag = false;
  directionFlag = false;

  // Other reset
  tempPositiveVal = 0;
  tempNegativeVal = 0;
  positiveVal = 0;
  negativeVal = 0;
}

void recvUART()
{
  // GPS: $GPRMC, $GPGGA
  // 方向燈：%0, ％1
  
  int count = 0;
  int gpsData1Length = sizeof(gpsData1) / sizeof(char);
  int gpsData2Length = sizeof(gpsData2) / sizeof(char);
  
  if (Serial1.available() > 0)
  {
    recvBuffer[count] = Serial1.read();
    Serial.write(recvBuffer[count]);
    
    if (recvBuffer[count] == '$') // GPS
    {
      count++;
      
      while (Serial1.available() > 0 && count < gpsData1Length)
      {
        recvBuffer[count] = Serial1.read();
        Serial.write(recvBuffer[count]);
        count++;
      }
      Serial.println();

      // "$GPRMC"
      count = 0;
      for (int i = 0; i < gpsData1Length; i++)
      {
        if (recvBuffer[i] == gpsData1[i])
        {
          count++;
        }
      }

      if (count == gpsData1Length) 
      {
        gpsFlag = true;
//        Serial.println("Success: receive GPS message");
      }

      // "$GPGGA"
      count = 0;
      for (int i = 0; i <gpsData2Length; i++)
      {
        if (recvBuffer[i] == gpsData2[i])
        {
          count++;
        }
      }
      
      if (count == gpsData2Length) 
      {
        gpsFlag = true;
//        Serial.println("Success: receive GPS message");
      }
      
    }
    else if (recvBuffer[count] == '%') // 方向燈
    {
      count++;

      if (Serial1.available() > 0)
      {
        recvBuffer[count] = Serial1.read();
        Serial.write(recvBuffer[count]);

        if (recvBuffer[count] == '0')
        {
          directionFlag = true;
        }
        else 
        {
          directionFlag = false;
        }
      }
    }
  }
}

void sendWarning() 
{
  warningCount++;

  if (gpsFlag && directionFlag && warningCount >= 10) // 10 * DELAY_TIME = 100ms
  {
    Serial.println(WARNING_MESSAGE);
    Serial1.println(WARNING_MESSAGE);

    // reset
    warningCount = 0;
    gpsFlag = false;
    directionFlag = false;
  }
}

