//This motor shield use Pin 6,5,7,4 to control the motor
// Simply connect your motors to M1+,M1-,M2+,M2-
// Upload the code to Arduino/Roboduino
// Through serial monitor, type 'a','s', 'w','d','x' to control the motor
// www.dfrobot.com
// Last modified on 24/12/2009


#include <EEPROM.h>


// This is the values to use with DFRduino MotorShield 
//int EN1 = 6;  
//int EN2 = 5;  //Roboduino Motor shield uses Pin 9
//int IN1 = 7;
//int IN2 = 4; //Latest version use pin 4 instead of pin 8

// This the values to use with real Arduino Motor Shield
int EN1 = 3;  
int EN2 = 11;  //Roboduino Motor shield uses Pin 9
int IN1 = 12;
int IN2 = 13; //Latest version use pin 4 instead of pin 8

int ANALOG_TOP_LEFT = 4;
int ANALOG_TOP_RIGHT = 5;
int ANALOG_BOTTOM_LEFT = 0;
int ANALOG_BOTTOM_RIGHT = 1;
int STORM_MODE = 2;
int LED = 4;

int g_Analog_Top_Left = 0;
int g_Analog_Top_Right = 0;
int g_Analog_Bottom_Left = 0;
int g_Analog_Bottom_Right = 0;
int g_Delta_Top_Left = 0;
int g_Delta_Top_Right = 0;
int g_Delta_Bottom_Left = 0;
int g_Delta_Bottom_Right = 0;
int g_Storm_mode = 0;
int g_transition = 0;
int g_ManualAzimuth = 0;
int g_ManualElevation = 0;
int g_IsNight = 0;
int g_IsTimeSet = 0;
int g_SunSleepTime = 0;
int g_SunRaiseTime = 0;
int g_LedSequence = 0x01;

#define MOTOR_LEVEL               255
#define MOTOR_OFF                 0
#define DIRECTION_AZIMUTH_WE      1    // Direction to turn from West to East
#define DIRECTION_AZIMUTH_EW      0    // Direction to turn from East to West
#define DIRECTION_ELEVATION_UP    0    // Direction to go Up
#define DIRECTION_ELEVATION_DOWN  1    // Direction to go Down
#define LOOP_DELAY                10   // Duration in ms of one cycle
#define FULL_RANGE_DURATION_AZIMUTH       460L  // Duration (in 0,1s) for a full extension
#define FULL_RANGE_DURATION_ELEVATION     390L
#define DELTA_MINUTES             3

#define DELTA(a, b)   (a > b ? (a-b) : (b-a))
#define DELTA_THRESHOLD           2 
#define NB_MIN_TRANSITION         2
#define INCR(a)                   a = ((a < 100) ? (a+1) : a)
#define DECR(a)                   a = ((a >= 1) ? (a-1) : a)


#define STATE_INIT_AZIMUTH        1
#define STATE_INIT_ELEVATION      2
#define STATE_RUNNING             0
#define STATE_STORM               3
#define STATE_STORM_MANUAL        4

#define LED_MODE_NIGHT            0x001
#define LED_MODE_DAY              0x00F
#define LED_MODE_NIGHT_NOTIME     0x009
#define LED_MODE_DAY_NOTIME       0x01B
#define LED_MODE_STORM            0x421
#define LED_MODE_STORM_MANUAL     0x0E7


int g_State = STATE_INIT_AZIMUTH;
int g_Tenths = 0;
int g_LastAzimuth = 0;
int g_StateAzimuthMotor = 0;
unsigned long g_EndAzimuthMotor = 0;
int g_LastElevation = 0;
int g_StateElevationMotor = 0;
unsigned long g_EndElevationMotor = 0;
unsigned long g_AbsoluteTime = 0;
int g_AutoAzimuth = 0;
int g_AutoElevation = 0;

#define NUMBER_OF_HOURS 24
#define NUMBER_PER_HOUR 6

#define FIRST_HOUR                6
#define DEFAULT_ELEVATION         20   // Temporary : Default elevation - To be replaced by an evaluation based on day of year
#define NIGHT_AZIMUTH             50   // Azimuth position for night hours
#define NIGHT_ELEVATION           80   // Elevation position for night hours

unsigned char g_TableAzimuth[NUMBER_OF_HOURS*NUMBER_PER_HOUR];
unsigned char  g_InitTableAzimuth[25] = { 0,0,0,0,0,0,0,0,0,0,0,10,25,50,50,75,90,100,100,100,100,100,100,100,100 }; 
int g_Elevation = DEFAULT_ELEVATION; 

int g_Hours = 15;
int g_Minutes = 21;
int g_Seconds = 0;
unsigned long nextTime;

unsigned char correlate(unsigned char first, unsigned char last, unsigned char step)
{
    unsigned char n = (last - first) / NUMBER_PER_HOUR;
    return (first + step*n);
}

void setup() 
{ 
    int i,j;
    int n=0;

    // Initialize TableAzimuth table
    for (i = 0 ; i < 24 ; i++)
        {
        for (j = 0 ; j < NUMBER_PER_HOUR ; j++)
            g_TableAzimuth[n++] = correlate(g_InitTableAzimuth[i], g_InitTableAzimuth[i+1], (unsigned char)j);  
        }
 
   // For Arduino Motor Shield
   pinMode(3, OUTPUT);
   pinMode(11, OUTPUT);
   pinMode(12, OUTPUT);
   pinMode(13, OUTPUT);
   pinMode(ANALOG_TOP_LEFT, INPUT);
   pinMode(ANALOG_TOP_RIGHT, INPUT);
   pinMode(ANALOG_BOTTOM_LEFT, INPUT);
   pinMode(ANALOG_BOTTOM_RIGHT, INPUT);
   pinMode(STORM_MODE, INPUT_PULLUP);
   pinMode(LED, OUTPUT);
   digitalWrite(LED, 0);
   
   Serial.begin(115200);
   Serial.println("SuiviSolaire - Version 2.0");
   Serial.println("Entering STATE_INIT_AZIMUTH");

   nextTime = millis() + 100;

   //loadData();
   
   readAnalog();
   g_IsNight = checkNight();

   // When STORM button is pressed at startup, keep time from predefined value 
   if (digitalRead(STORM_MODE) == 0)
      {
      g_IsTimeSet = true;
      g_LedSequence = LED_MODE_STORM;  
      }
   else
      g_LedSequence = g_IsNight ? LED_MODE_NIGHT_NOTIME : LED_MODE_DAY_NOTIME;  

   g_State = STATE_INIT_AZIMUTH;
   Serial.println("------ Switching to INIT_AZIMUTH mode ------");
} 

void setAzimuth(int azimuth)
{
  int direction;
  int update;

  if (azimuth > g_LastAzimuth)
  {
    direction = DIRECTION_AZIMUTH_WE;
    update = azimuth - g_LastAzimuth; 
  }
  else
  {
    direction = DIRECTION_AZIMUTH_EW;
    update = g_LastAzimuth - azimuth; 
  }
  unsigned long duration = FULL_RANGE_DURATION_AZIMUTH*update/100L;
  g_EndAzimuthMotor = g_AbsoluteTime + duration;
  g_StateAzimuthMotor = 1;
  g_LastAzimuth = azimuth;
  Motor1(MOTOR_LEVEL,direction);
  Serial.println("Starting Azimuth motor");
}

void setElevation(int elevation)
{
  int direction;
  int update;

  if (elevation > g_LastElevation)
  {
    direction = DIRECTION_ELEVATION_UP;
    update = elevation - g_LastElevation;
  }
  else
  {
    direction = DIRECTION_ELEVATION_DOWN;
    update = g_LastElevation - elevation;
  }
  unsigned long duration = FULL_RANGE_DURATION_ELEVATION*update/100L;
  g_EndElevationMotor = g_AbsoluteTime + duration;
  g_StateElevationMotor = 1;
  g_LastElevation = elevation;
  Motor2(MOTOR_LEVEL,direction);
}

void checkMotors()
{
    if (g_StateAzimuthMotor && (g_EndAzimuthMotor == g_AbsoluteTime))
    {
        g_StateAzimuthMotor = 0;
        Motor1(0,0);
        Serial.println(" Stopping Azimuth motor");
    }
    if (g_StateElevationMotor && (g_EndElevationMotor == g_AbsoluteTime))
    {
        g_StateElevationMotor = 0;
        Motor2(0,0);
        Serial.println(" Stopping Elevation motor");
    }
}

void setPosition()
{
  int n;
  int azimuth;
  int elevation;

  if (g_StateElevationMotor || g_StateAzimuthMotor)
      {
      checkMotors();
      return;
      }      
      
  if (g_State == STATE_STORM_MANUAL)
  {
    azimuth = g_ManualAzimuth;
    elevation = g_ManualElevation;
    if (azimuth != g_LastAzimuth)
      {
      Serial.print("New manual Azimuth : ");
      Serial.println(azimuth);
      setAzimuth(azimuth);
      }
    else if (elevation != g_LastElevation)
      {
      Serial.print("New manual Elevation : ");
      Serial.println(elevation);
      setElevation(elevation);
      }
        
    return;
  }

  if (g_IsNight || g_State == STATE_STORM || !g_IsTimeSet)
  {
    azimuth = NIGHT_AZIMUTH;
    elevation = NIGHT_ELEVATION;
    if (azimuth != g_LastAzimuth)
      {
      Serial.print("New Azimuth : ");
      Serial.println(azimuth);
      setAzimuth(azimuth);
      }
    else if (elevation != g_LastElevation)
      {
      Serial.print("New Elevation : ");
      Serial.println(elevation);
      setElevation(elevation);
      }
    return;
  }

  n = g_Hours*NUMBER_PER_HOUR + (g_Minutes * NUMBER_PER_HOUR / 60);
  g_AutoAzimuth = g_TableAzimuth[n];
  g_AutoElevation = g_Elevation;
      
  if (g_AutoAzimuth != g_LastAzimuth)
    {
    Serial.print("New Azimuth : ");
    Serial.println(g_AutoAzimuth);
    setAzimuth(g_AutoAzimuth);
    }
  else if (g_AutoElevation != g_LastElevation)
    {
    Serial.print("New Elevation : ");
    Serial.println(g_AutoElevation);
    setElevation(g_AutoElevation);
    }
}

void printTime()
{
     Serial.print("Time : ");
     Serial.print(g_AbsoluteTime);
     Serial.print(" - ");
     if (g_IsTimeSet)
         {
         Serial.print(g_Hours);
         Serial.print(":");
         Serial.print(g_Minutes);
         Serial.print(":");
         Serial.print(g_Seconds);
         }
     else
         Serial.print("??:??:??");
}

void printStatus(void)
{
    printTime();
    Serial.print("     TOP :  ");
    Serial.print(g_Analog_Top_Left);
    Serial.print("  -  ");
    Serial.print(g_Analog_Top_Right);
    Serial.print("  (");
    Serial.print(g_Delta_Top_Left);
    Serial.print("-");
    Serial.print(g_Delta_Top_Right);
    Serial.print(")  BOTTOM :  ");
    Serial.print(g_Analog_Bottom_Left);
    Serial.print("  -  ");
    Serial.print(g_Analog_Bottom_Right);
    Serial.print("  (");
    Serial.print(g_Delta_Bottom_Left);
    Serial.print("-");
    Serial.print(g_Delta_Bottom_Right);
    Serial.print(") ");
    switch(g_State)
    {
    case STATE_INIT_AZIMUTH:   Serial.print("STATE_INIT_AZIMUTH"); break;   
    case STATE_INIT_ELEVATION: Serial.print("STATE_INIT_ELEVATION"); break;
    case STATE_RUNNING:        Serial.print("STATE_RUNNING"); break;
    case STATE_STORM:          Serial.print("STATE_STORM"); break;   
    case STATE_STORM_MANUAL:   Serial.print("STATE_STORM_MANUAL"); break;
    default :                  Serial.print("STATE_INVALID"); break;
    }
    Serial.print("  Mode Tempete=");
    Serial.print(digitalRead(STORM_MODE));
    Serial.print(" Elevation=");
    Serial.print(g_Elevation);
    Serial.print(" isNight=");
    Serial.print(g_IsNight);
    Serial.print(" SunRaiseTime=");
    Serial.print((g_SunRaiseTime / 60));
    Serial.print("h");
    Serial.print((g_SunRaiseTime % 60));
    Serial.print(" SunSleepTime=");
    Serial.print((g_SunSleepTime / 60));
    Serial.print("h");
    Serial.println((g_SunSleepTime % 60));
}

int checkNight()
{
  if (!g_IsNight && g_Analog_Top_Left == 0 &&
      g_Analog_Top_Right == 0 &&
      g_Analog_Bottom_Left == 0 &&
      g_Analog_Bottom_Right == 0)
       return true;
  if (g_IsNight && g_Analog_Top_Left != 0 &&
      g_Analog_Top_Right != 0 &&
      g_Analog_Bottom_Left != 0 &&
      g_Analog_Bottom_Right != 0)
      return false;
      
  return g_IsNight;
}

void readAnalog()
{
    if (g_StateElevationMotor || g_StateAzimuthMotor)
    {
        g_Delta_Top_Left = 0;
        g_Delta_Top_Right = 0;
        g_Delta_Bottom_Left = 0;
        g_Delta_Bottom_Right = 0;
        return;
    }
    
    int analog_Top_Left = analogRead(ANALOG_TOP_LEFT);
    int analog_Top_Right = analogRead(ANALOG_TOP_RIGHT);
    int analog_Bottom_Left = analogRead(ANALOG_BOTTOM_LEFT);
    int analog_Bottom_Right = analogRead(ANALOG_BOTTOM_RIGHT);

    g_Delta_Top_Left = DELTA(analog_Top_Left, g_Analog_Top_Left);
    g_Delta_Top_Right = DELTA(analog_Top_Right, g_Analog_Top_Right);
    g_Delta_Bottom_Left = DELTA(analog_Bottom_Left, g_Analog_Bottom_Left);
    g_Delta_Bottom_Right = DELTA(analog_Bottom_Right, g_Analog_Bottom_Right);
    g_Analog_Top_Left = analog_Top_Left;
    g_Analog_Top_Right = analog_Top_Right;
    g_Analog_Bottom_Left = analog_Bottom_Left;
    g_Analog_Bottom_Right = analog_Bottom_Right;
}

void saveData()
{
    unsigned char data[5];
    int i;
    
    data[0] = (unsigned char)g_Elevation;
    data[1] = (unsigned char)(g_SunRaiseTime % 256);
    data[2] = (unsigned char)(g_SunRaiseTime / 256);
    data[3] = (unsigned char)(g_SunSleepTime % 256);
    data[4] = (unsigned char)(g_SunSleepTime / 256);
    for (i = 0 ; i < 5 ; i++)
      EEPROM.write(i, data[i]);
}

void loadData()
{
    int data[5];
    int i;

    for (i = 0 ; i < 5 ; i++)
      data[i] = EEPROM.read(i);
    g_Elevation = (int)data[0];
    g_SunRaiseTime = (int)data[1];
    g_SunRaiseTime += ((int)data[2])*256;
    g_SunSleepTime = (int)data[3];
    g_SunSleepTime += ((int)data[4])*256;

    Serial.print("Read from Flash memory : Elevation=");
    Serial.print(g_Elevation);
    Serial.print(" SunRaiseTime=");
    Serial.print((g_SunRaiseTime / 60));
    Serial.print("h");
    Serial.print((g_SunRaiseTime % 60));
    Serial.print(" SunSleepTime=");
    Serial.print((g_SunSleepTime / 60));
    Serial.print("h");
    Serial.println((g_SunSleepTime % 60));
}

void storeSunSleepTime()
{
    int n = (g_Hours * 60) + g_Minutes;
    int d = DELTA(n, g_SunSleepTime);
    if (d > DELTA_MINUTES)
        {
        g_SunSleepTime = n;  
        saveData();
        Serial.print("Sun is going to sleep : new time detected and saved ");
        Serial.print(g_Hours);
        Serial.print("h");
        Serial.println(g_Minutes);
        }
    else
        {
        Serial.print("Sun is going to sleep : almost same time detected - no update ");
        Serial.print(g_Hours);
        Serial.print("h");
        Serial.println(g_Minutes);
        }
}

void storeSunRaiseTime()
{
    int n = (g_Hours * 60) + g_Minutes;
    int d = DELTA(n, g_SunRaiseTime);
    if (d > DELTA_MINUTES)
        {
        g_SunRaiseTime = n;
        saveData();
        Serial.print("Sun is going to raise : new time detected and saved ");
        Serial.print(g_Hours);
        Serial.print("h");
        Serial.println(g_Minutes);
        }
    else
        {
        Serial.print("Sun is going to raise : almost same time detected - no update ");
        Serial.print(g_Hours);
        Serial.print("h");
        Serial.println(g_Minutes);
        }
}

void loop() 
{
  unsigned long currentTime = millis();
  int isNight;
  
  if (currentTime > nextTime)
      {
      nextTime += 100;
      }
  else
      {
      delay(LOOP_DELAY);
      return;
      }


  // g_AbsoluteTime is the time (0.1s multiplier)
  g_AbsoluteTime++;

  // Time management
  g_Tenths++;
  if (g_Tenths == 10)
    {
    g_Tenths = 0;
    g_Seconds++;
    if ((g_Seconds & 1) == 0)
        g_transition = 0;
        
    if (g_Seconds == 60)
    {
      g_Seconds = 0;
      g_Minutes++;
 
      if (g_Minutes == 60)
      {
        g_Minutes = 0;
        g_Hours++;
        if (g_Hours == 24)
          {
          g_Hours = 0;
          }
        if (g_IsTimeSet && g_Hours == FIRST_HOUR && g_State != STATE_STORM)
          g_State = STATE_INIT_AZIMUTH;
      }
    }
    if (g_Seconds == 0)  
      printStatus();
    }

   // LED management
   int state = g_LedSequence & (1 << (g_Tenths+10*(g_seconds&1));
   if (state)
      digitalWrite(LED, 1);
   else
      digitalWrite(LED, 0);
    
  switch(g_State)
  {
    case STATE_RUNNING :
      readAnalog();
      isNight = checkNight();
      if (isNight && !g_IsNight)
        {
        if (!g_IsTimeSet)
            {
            g_IsTimeSet = true;
            g_Hours = (g_SunSleepTime / 60);
            g_Minutes = (g_SunSleepTime % 60);
            g_Seconds = 0;  
            }
        g_LedSequence = LED_MODE_NIGHT;
        g_IsNight = true;
        storeSunSleepTime();  
        }
      if (!isNight && g_IsNight)
        {
        if (!g_IsTimeSet)
            {
            g_IsTimeSet = true;
            g_Hours = (g_SunRaiseTime / 60);
            g_Minutes = (g_SunRaiseTime % 60);  
            g_Seconds = 0;  
            }
        g_LedSequence = LED_MODE_DAY;
        g_IsNight = false;
        storeSunRaiseTime();  
        // Everyday, when sur raises, reinit position of verin
        g_State = STATE_INIT_AZIMUTH; 
        }
        
        if (digitalRead(STORM_MODE) == 0)
        {
            g_State = STATE_STORM;
            g_LedSequence = LED_MODE_STORM;
            Serial.println("------ Switching to STORM mode ------");
        }
        setPosition();
        break;
        
    case STATE_INIT_AZIMUTH:
      if (g_StateAzimuthMotor == 0)
      {
         Motor1(MOTOR_LEVEL,DIRECTION_AZIMUTH_EW);
         g_EndAzimuthMotor = g_AbsoluteTime + FULL_RANGE_DURATION_AZIMUTH;
         g_StateAzimuthMotor = 1;
         Serial.println("Starting Azimuth motor to go West");
      }
      else if (g_EndAzimuthMotor == g_AbsoluteTime)
      {
        g_StateAzimuthMotor = 0;
        Motor1(0,0);
        g_LastAzimuth = 0;
        Serial.println(" Stopping Azimuth motor - Init done on West");
        g_State = STATE_INIT_ELEVATION;
        Serial.println("------ Switching to INIT_ELEVATION mode ------");
      }
      break;
      
    case STATE_INIT_ELEVATION:
        if (g_StateElevationMotor == 0)
        {
            g_StateElevationMotor = 1;
            g_EndElevationMotor = g_AbsoluteTime + FULL_RANGE_DURATION_ELEVATION;
            Motor2(MOTOR_LEVEL,DIRECTION_ELEVATION_DOWN);
            Serial.println("Starting Elevation motor");
        }
        else if (g_EndElevationMotor == g_AbsoluteTime)
        {
            g_StateElevationMotor = 0;
            Motor2(0,0);
            g_LastElevation = 0;
            Serial.println(" Stopping Elevation motor - Init done on Down");
            g_State = STATE_RUNNING;
            if (g_IsTimeSet)
                g_LedSequence = g_IsNight ? LED_MODE_NIGHT : LED_MODE_DAY;
            else
                g_LedSequence = g_IsNight ? LED_MODE_NIGHT_NOTIME : LED_MODE_DAY_NOTIME;
            Serial.println("------ Switching to RUNNING mode ------");
        }
        break;

    case STATE_STORM :
        if (g_Tenths == 0)
        {
            readAnalog();
            if (g_Delta_Top_Left > DELTA_THRESHOLD &&
                g_Delta_Top_Right > DELTA_THRESHOLD &&
                g_Delta_Bottom_Left > DELTA_THRESHOLD &&
                g_Delta_Bottom_Right > DELTA_THRESHOLD)
            {
                g_transition++;
                Serial.print("Light Transition detected : ");
                Serial.println(g_transition);            
            }
            if (g_transition >= NB_MIN_TRANSITION)
            {
                g_State = STATE_STORM_MANUAL;
                g_LedSequence = LED_MODE_STORM_MANUAL;
                g_transition = 0; 
                g_ManualAzimuth = g_AutoAzimuth;
                g_ManualElevation = g_AutoElevation;
                Serial.println("------ Switching to Manual Mode ------");
            }
        }    
        if (digitalRead(STORM_MODE) == 1)
            {
            g_State = STATE_RUNNING;
            if (g_IsTimeSet)
                g_LedSequence = g_IsNight ? LED_MODE_NIGHT : LED_MODE_DAY;
            else
                g_LedSequence = g_IsNight ? LED_MODE_NIGHT_NOTIME : LED_MODE_DAY_NOTIME;
                
            Serial.println("------ Switching to RUNNING mode ------");
            }
        setPosition();
        break;

    case STATE_STORM_MANUAL :
        if (g_Tenths == 0)
        {
            readAnalog();
            if (g_Delta_Top_Left > DELTA_THRESHOLD)
                INCR(g_ManualElevation);
            if (g_Delta_Bottom_Left > DELTA_THRESHOLD)
                DECR(g_ManualElevation);
            //if (g_Delta_Top_Right > DELTA_THRESHOLD)
            //    INCR(g_ManualAzimuth);
            //if (g_Delta_Bottom_Right > DELTA_THRESHOLD)
            //    DECR(g_ManualAzimuth);
        }
        setPosition();
            
        if (digitalRead(STORM_MODE) == 1)
        {
            g_State = STATE_RUNNING;
            g_Elevation = g_ManualElevation;
            if (g_IsTimeSet)
                g_LedSequence = g_IsNight ? LED_MODE_NIGHT : LED_MODE_DAY;
            else
                g_LedSequence = g_IsNight ? LED_MODE_NIGHT_NOTIME : LED_MODE_DAY_NOTIME;
            saveData();
            Serial.println("------ Switching to RUNNING mode ------");
        }
        break;
        
    default:
      Serial.println("Software error : Invalid State");
      break;
  }
}

 
void Motor1(int pwm, boolean reverse)
{
   analogWrite(EN1,pwm); //set pwm control, 0 for stop, and 255 for maximum speed
   if(reverse)
          digitalWrite(IN1,HIGH);    
        else
          digitalWrite(IN1,LOW);    
}  
         
void Motor2(int pwm, boolean reverse)
{
    analogWrite(EN2,pwm);
    if(reverse)
          digitalWrite(IN2,HIGH);    
        else
          digitalWrite(IN2,LOW);    
}  
        
