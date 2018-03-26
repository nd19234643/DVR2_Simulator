#define WARNING_MESSAGE "AT$PD" // AT$PD=21;LDW:1<0x0D><0x0A>
#define DELAY_TIME 10 // unit: ms
#define BUFFER_SIZE 10 // unit: byte

#define LOW_VALUE_OF_FREQUENCY 0.6  // unit: Hz
#define HIGH_VALUE_OF_FREQUENCY 1.5 // unit: Hz

// Input Pin
#define LEFT_DIRECTION_CHECK_INPIN 2   // 左方向燈
#define RIGHT_DIRECTION_CHECK_INPIN 3  // 右方向燈
#define SVIDEO_HPR_INPIN 4             // S 端子
#define SVIDEO_TVOUT_INPIN 5           // S 端子
#define NH25_SIGNAL_INPIN 6            // NH2.5-2P
#define START_SWITCH_INPIN 7           // 開關

// Output Pin
#define SVIDEO_NH25_LED_OUTPIN 8            // S 端子線, Powr 和 NH2.5-2P 線 是否導通

// Input Pin
#define VCC24V_INPIN 9                 // Power 24v
#define VCC12V_INPIN 10                // Power 12v

boolean initFlag = false;
boolean gpsFlag = false;
boolean directionFlag = false;

unsigned char ledCount = 0, warningCount = 0, checkFrequencyCount = 0;

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
    /***** Receive GPS *****/
    recvUART(); // 10 ms

    /***** Check Direction Light Frequency *****/
//    checkLeftDirection();  // for test
//    checkRightDirection(); // for test

    /***** Send Warning *****/
    sendWarning(); // 100 ms

    /***** Check S-Video and NH2.5-2P *****/
    checkSVideoAndNH25(); // 10 ms

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
  pinMode(LEFT_DIRECTION_CHECK_INPIN, INPUT);
  pinMode(RIGHT_DIRECTION_CHECK_INPIN, INPUT);
  pinMode(SVIDEO_HPR_INPIN, INPUT);
  pinMode(SVIDEO_TVOUT_INPIN, INPUT);
  pinMode(NH25_SIGNAL_INPIN, INPUT);
  pinMode(START_SWITCH_INPIN, INPUT_PULLUP);
  pinMode(VCC24V_INPIN, INPUT);
  pinMode(VCC12V_INPIN, INPUT);

  // Output Pin
  pinMode(SVIDEO_NH25_LED_OUTPIN, OUTPUT);
  
  Serial.begin(9600);   // USB Debug
  Serial1.begin(9600);  // UART Communication
}

void initFlow()
{
  if (initFlag == false)
  {
    // Output Pin
    digitalWrite(SVIDEO_NH25_LED_OUTPIN, LOW);
    delay(200);
    
    for (int i = 0; i < 3 ; i++) 
    {
      digitalWrite(SVIDEO_NH25_LED_OUTPIN, HIGH);
      delay(200);
      digitalWrite(SVIDEO_NH25_LED_OUTPIN, LOW);
      delay(200);
    }

    initFlag = true;
  }
}

void resetStatus()
{ 
  // Output Pin reset
  digitalWrite(SVIDEO_NH25_LED_OUTPIN, LOW);

  // Count reset
  ledCount = 0;
  checkFrequencyCount = 0;
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

void checkLeftDirection()
{
  int value = digitalRead(LEFT_DIRECTION_CHECK_INPIN); 
  checkFrequencyRange(LOW_VALUE_OF_FREQUENCY, HIGH_VALUE_OF_FREQUENCY, value); // 100 ms
}

void checkRightDirection()
{
  int value = digitalRead(RIGHT_DIRECTION_CHECK_INPIN); 
  checkFrequencyRange(LOW_VALUE_OF_FREQUENCY, HIGH_VALUE_OF_FREQUENCY, value); // 100 ms
}

void checkFrequencyRange(float lowValue, float highValue, int sensorVal)
{

  //1. (1 / 1.5) ~= 0.67 s
  //2. (0.67 s * 1000) = 670 ms
  //3. (670 ms / 10 ms) = 67 count (High + Low) (10ms 讀一次值)
  float lowerBound = ((1 / highValue) * 1000 / DELAY_TIME);

  //1. (1 / 0.6) ~= 1.67 s
  //2. (1.67 s * 1000) = 1670 ms
  //3. (1670 ms / 10 ms) = 167 count (High + Low) (10ms 讀一次值)
  float upperBound = ((1/ lowValue) * 1000 / DELAY_TIME);

  // Count
  if (sensorVal == LOW)
  {
    tempNegativeVal += 1;
    if (tempPositiveVal > 0)
    {
      positiveVal == tempPositiveVal;
      tempPositiveVal = 0;
    }
  }
  else // HIGH
  {
    tempPositiveVal += 1;
    if (tempNegativeVal > 0) 
    {
      negativeVal = tempNegativeVal;
      tempNegativeVal = 0;
    }
  }

  // Clear data
  if (tempNegativeVal > upperBound) // > 167 count
  {
    tempNegativeVal = 0;
    negativeVal = 0;
  }
  if (tempPositiveVal > upperBound) // > 167 count
  {
    tempPositiveVal = 0;
    positiveVal = 0;
  }

  // Check frequency range
  checkFrequencyCount++;
  if (checkFrequencyCount >= 10) // 100 ms 檢查一次
  {
    unsigned char sum = negativeVal + positiveVal;
    
    if ((sum >= lowerBound) && (sum <= upperBound))
    {
      Serial.print('%');
//      Serial.println(sum, DEC);
      Serial.println('0');
      directionFlag = true;
    }
    else
    {
      Serial.print('%');
//      Serial.println(sum, DEC);
      Serial.println('1');  
      directionFlag = false;
    }

    checkFrequencyCount = 0;
  }
}

void sendWarning() 
{
  if (gpsFlag && directionFlag)
  {
    warningCount++;

    if (warningCount >= 10) // 10 * DELAY_TIME = 100ms
    {
      Serial.println(WARNING_MESSAGE);
      Serial1.println(WARNING_MESSAGE);

      // reset
      warningCount = 0;
      gpsFlag = false;
      directionFlag = false;
    }
  }
}

void checkSVideoAndNH25()
{
  if (digitalRead(SVIDEO_HPR_INPIN) == HIGH && digitalRead(SVIDEO_TVOUT_INPIN) == HIGH && digitalRead(NH25_SIGNAL_INPIN) == HIGH && digitalRead(VCC24V_INPIN) == HIGH && digitalRead(VCC12V_INPIN) == HIGH)
  {
    ledCount++;

    if (ledCount >= 100) // 100 * DELAY_TIME = 1000 ms
    {
      digitalWrite(SVIDEO_NH25_LED_OUTPIN, HIGH); // 亮
      ledCount = 0;
    }
    else if (ledCount >= 25) // 25 * DELAY_TIME = 250 ms
    {
      digitalWrite(SVIDEO_NH25_LED_OUTPIN, LOW);  // 暗
    }
  }
  else
  {
    digitalWrite(SVIDEO_NH25_LED_OUTPIN, LOW);  // 暗
  }
}

