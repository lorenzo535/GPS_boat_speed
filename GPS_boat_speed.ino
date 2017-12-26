
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Streaming.h>

#define GPS_TX 4
#define GPS_RX 12 //unused

#define SQUAREWAVE_OUT 2
#define MILESCOUNTER_OUT 3
#define SEC_TO_HOURS 0.000277778f
#define KNOTS_TO_HZ 17
#define OCR1A_1HZ 15624     //value of the timer counter for a 1Hz interrupt call
#define GPSECHO false
#define DISTANCE_INTEGRAL_STEP_S 3
#define S_TO_MS 1000.0f
#define MS_TO_S 0.001f
SoftwareSerial mySerial(GPS_TX, GPS_RX);

Adafruit_GPS GPS(&mySerial);
//char readbuffer [64] = {0};
//char c;

boolean usingInterrupt = true;
boolean display_GPS_data = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
float GPS_speed = 0;

void ComputeDistanceTravelled(float);
void SetCompareMatchRegisterForHertz(float);

int manual_speed;
int old_GPS_seconds = 0;
uint32_t timer = millis();
uint32_t odometer_timer = millis();
uint32_t odo_reset_time = millis();
uint32_t time_delta = 0;
uint32_t millis_;
boolean manual_commands;
float miles_travelled;
float old_miles_travelled;
float distance_travelled;
float delta_travelled;
float delta;
int steps, i;
int total_steps = 0;
float old_speed;
boolean flipflop; 
boolean manual_high_speed = false;

//#define GPSECHO 1
void setup()  
{
    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(9600);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(4800);
  //mySerial.begin (4800);
  
  useInterrupt(true);

  delay(1000);

  pinMode(SQUAREWAVE_OUT, OUTPUT);
  pinMode(MILESCOUNTER_OUT, OUTPUT);
  miles_travelled = 0.0;
  old_miles_travelled = 0.0;
  old_speed = 0.0;

  manual_speed = 1;
  flipflop = false;

  digitalWrite(MILESCOUNTER_OUT, true);

//
cli();//stop interrupts

//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 65535;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

sei();//allow interrupts
//


}


void SetCompareMatchRegisterForHertz(float hzsetpoint)
{

//compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1
// prescaler is 1024

//vcalibration tweak
if (hzsetpoint < 40)
  hzsetpoint = hzsetpoint * 0.9;

cli();
//OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
TCNT1  = 0;

if (hzsetpoint == -1)
  OCR1A = 65535;
else
  OCR1A = OCR1A_1HZ / hzsetpoint;

sei();

}


ISR(TIMER1_COMPA_vect)
{
  flipflop = !flipflop;      
  digitalWrite (SQUAREWAVE_OUT,flipflop);
  //Serial <<".";
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


void loop()                     // run over and over again
{

  
  // if a sentence is received, we can check the checksum, parse it...
   //char c = GPS.read();
   //if (c) Serial.print(c);

   //if (mySerial.available()>0)
   //{
   ///c = mySerial.read();
  // if (c) Serial.print(c);
   //}
   
 
  if ((GPS.newNMEAreceived())&&(!manual_commands)) {
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false    
     return;  // we can fail to parse a sentence in which case we should just wait for another      
     else
     {
      if (GPS.fix)      
      {
        GPS_speed = GPS.speed;
          
        if (GPS_speed >= 0.1)
        {
        
          if (GPS.seconds != old_GPS_seconds)
          {
            Serial << "Fix valid ; speed is " << GPS_speed << "\n";
            old_GPS_seconds = GPS.seconds;
            SetCompareMatchRegisterForHertz( GPS_speed * KNOTS_TO_HZ);        
            ComputeDistanceTravelled(GPS_speed);
          }
        }
        else
           SetCompareMatchRegisterForHertz( -1);
      }
    else
      SetCompareMatchRegisterForHertz( -1);
     }
             
  }

   if (manual_commands)
   ComputeDistanceTravelled(manual_speed);                     

       
  millis_ = millis();
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis_)  timer = millis_;
  if (odometer_timer > millis_)  odometer_timer = millis_;
  
  if (display_GPS_data)
    PrintStuff();

  ReadKeyboardCmds();
}

void ReadKeyboardCmds()
{
   if (Serial.available() > 0) 
   {  
      char rx_byte = Serial.read();       // get the character
    
      // check if test command
      if ((rx_byte == 't') || (rx_byte == 'T')) 
      {
        StepOdometerIncrement();
        
      }

      // check if test command
      if ((rx_byte == 'g') || (rx_byte == 'G')) 
      {
        display_GPS_data = !display_GPS_data;
        
      }

      if ((rx_byte == 'o') || (rx_byte == 'O')) 
      {
        ResetOdometer();        
      }

      if ((rx_byte == 'h') || (rx_byte == 'H')) 
      {
       manual_high_speed = ! manual_high_speed;
      }
      
      // check if manual command
      if ((rx_byte == 'x') || (rx_byte == 'X')) 
      {
        manual_commands =!manual_commands;
        Serial << "switched to " << (manual_commands ? "manual" : "programmed") << "controls \n";
        
      }
      if (manual_commands)
        {
          if ((rx_byte >= '1') && (rx_byte <= '9'))
          {
          manual_speed = int (rx_byte) - 48;
          if (manual_high_speed) manual_speed += 9;
          Serial << "//New speed " << manual_speed << " knots\\ \n";          
          SetCompareMatchRegisterForHertz( manual_speed * KNOTS_TO_HZ);
          }
          
        }
  
  }
}


void ComputeDistanceTravelled( float speed)
{
//Compute distance travelled
  time_delta = millis() - odometer_timer ;
    if (time_delta > DISTANCE_INTEGRAL_STEP_S * S_TO_MS) 
    { 
      odometer_timer = millis(); // reset the timer
      
      delta = (speed + old_speed) * time_delta * MS_TO_S * SEC_TO_HOURS /2.0f;  
      miles_travelled = miles_travelled  + delta;
      delta_travelled = miles_travelled - old_miles_travelled;
      
      old_speed= speed;

       
     //Serial << "time delta " <<_FLOAT (time_delta, 4) << "=== delta distance " << _FLOAT (delta, 4) << "\n";
     
     if ((delta_travelled) > 0.01)
     {
       steps = (int)(delta_travelled / 0.01);
       //Serial << "delta: "<< _FLOAT (delta_travelled, 4) << "  steps: "<< steps << "\n";
       for (i = 0; i < steps; i++)
       {
        StepOdometerIncrement();
        total_steps = total_steps +1;
       }

       old_miles_travelled = miles_travelled - (delta_travelled - steps * 0.01);
       Serial << "delta_travelled " <<_FLOAT (delta_travelled, 4);
       Serial << "miles travelled  " << _FLOAT (miles_travelled, 4) << " old_miles_travelled" << _FLOAT (old_miles_travelled,4) << "\n";       
       Serial << "miles are now " << miles_travelled << " speed is " << speed << " total steps sent: "<< total_steps <<"\n";
     }
     
     
    }
}

void ResetOdometer()
{
  Serial << " ### RESET ### \n";
  Serial << " Odometer was running since " << (millis () - odo_reset_time) * MS_TO_S<< " seconds, distance travelled:" << miles_travelled <<"\n" ;
  
  miles_travelled = 0.0;
  delta_travelled = 0.0;
  old_speed = 0.0;
  old_miles_travelled = 0.0;
  total_steps = 0;
  odo_reset_time = millis();
}

void StepOdometerIncrement()
{
        digitalWrite(MILESCOUNTER_OUT, true);
       delay (200);
       digitalWrite(MILESCOUNTER_OUT, false);
       delay (200);
       Serial << "step \n";
 

}

void PrintStuff()
{

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
   
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
  
}
