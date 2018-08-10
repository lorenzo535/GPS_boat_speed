
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Streaming.h>

#define GPS_TX 4
#define GPS_RX 11 //unused

#define SQUAREWAVE_OUT 2
#define MILESCOUNTER_OUT 3
#define SEC_TO_HOURS 0.000277778f
#define KNOTS_TO_HZ 17
#define OCR1A_1HZ 15624     //value of the timer counter for a 1Hz interrupt call
#define GPSECHO 0
#define DISTANCE_INTEGRAL_STEP_S 3
#define S_TO_MS 1000.0f
#define MS_TO_S 0.001f


#define LED_GPS_COMM 13
#define LED_GPS_VALID 12

SoftwareSerial mySerial(GPS_TX, GPS_RX);
Adafruit_GPS GPS(Serial);

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
uint32_t millissincelast = timer;
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


void setup()
{

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  mySerial.begin(9600);
  mySerial.println("GPS Boat speed display");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(4800);
  //mySerial.begin (4800);

  useInterrupt(true);

  delay(1000);

  pinMode(SQUAREWAVE_OUT, OUTPUT);
  pinMode(MILESCOUNTER_OUT, OUTPUT);
  pinMode(LED_GPS_COMM, OUTPUT);
  pinMode (LED_GPS_VALID, OUTPUT);
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
  digitalWrite (SQUAREWAVE_OUT, flipflop);
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
/*  
    //digitalWrite (LED_GPS_VALID, false);
    digitalWrite (SQUAREWAVE_OUT, true);
    Serial << "ping \n";
    delay (100);
    digitalWrite (SQUAREWAVE_OUT, false);
    Serial << "pong \n";
    delay (100);

    return;

    digitalWrite (LED_GPS_VALID, true);

    delay (1000);
    digitalWrite (LED_GPS_VALID, false);
    delay (1000);
  */

if (!manual_commands)
{
  if (GPS.newNMEAreceived())
  {

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    else
    {
      millissincelast = millis();
      //Serial << "decode ok\n";
      if (GPS.fix)
      {
        GPS_speed = GPS.speed;
        digitalWrite (LED_GPS_VALID, true);
        digitalWrite (LED_GPS_COMM, false);

        if (GPS_speed >= 0.5)
        {
          if (GPS.seconds != old_GPS_seconds)
          {
            mySerial << "Fix valid ; speed is " << GPS_speed << "\n";
            old_GPS_seconds = GPS.seconds;
            SetCompareMatchRegisterForHertz( GPS_speed * KNOTS_TO_HZ);
            ComputeDistanceTravelled(GPS_speed);
          }
        }
        else
          SetCompareMatchRegisterForHertz( -1);
      }
      else
      {
        SetCompareMatchRegisterForHertz( -1);
        digitalWrite (LED_GPS_VALID, true);
        digitalWrite (LED_GPS_COMM, true);
        //Serial << "signal not valid\n";
      }
    }

  }// end if received
  else
  {
    if (millis() - millissincelast > 3000)
    {
      digitalWrite (LED_GPS_COMM, true);
      digitalWrite (LED_GPS_VALID, false);
      //Serial << "no signal\n";
    }
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
  if (mySerial.available() > 0)
  {
    char rx_byte = mySerial.read();       // get the character

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
      manual_commands = !manual_commands;
      mySerial << "switched to " << (manual_commands ? "manual" : "programmed") << "controls \n";

    }
    if (manual_commands)
    {
      if ((rx_byte >= '1') && (rx_byte <= '9'))
      {
        manual_speed = int (rx_byte) - 48;
        if (manual_high_speed) manual_speed += 9;
        mySerial << "//New speed " << manual_speed << " knots\\ \n";
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

    delta = (speed + old_speed) * time_delta * MS_TO_S * SEC_TO_HOURS / 2.0f;
    miles_travelled = miles_travelled  + delta;
    delta_travelled = miles_travelled - old_miles_travelled;

    old_speed = speed;


    //Serial << "time delta " <<_FLOAT (time_delta, 4) << "=== delta distance " << _FLOAT (delta, 4) << "\n";

    if ((delta_travelled) > 0.01)
    {
      steps = (int)(delta_travelled / 0.01);
      //Serial << "delta: "<< _FLOAT (delta_travelled, 4) << "  steps: "<< steps << "\n";
      for (i = 0; i < steps; i++)
      {
        StepOdometerIncrement();
        total_steps = total_steps + 1;
      }

      old_miles_travelled = miles_travelled - (delta_travelled - steps * 0.01);
      mySerial << "delta_travelled " << _FLOAT (delta_travelled, 4);
      mySerial << "miles travelled  " << _FLOAT (miles_travelled, 4) << " old_miles_travelled" << _FLOAT (old_miles_travelled, 4) << "\n";
      mySerial << "miles are now " << miles_travelled << " speed is " << speed << " total steps sent: " << total_steps << "\n";
    }


  }
}

void ResetOdometer()
{
  mySerial << " ### RESET ### \n";
  mySerial << " Odometer was running since " << (millis () - odo_reset_time) * MS_TO_S << " seconds, distance travelled:" << miles_travelled << "\n" ;

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
  mySerial << "step \n";


}

void PrintStuff()
{

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    mySerial.print("\nTime: ");
    mySerial.print(GPS.hour, DEC); mySerial.print(':');
    mySerial.print(GPS.minute, DEC); mySerial.print(':');
    mySerial.print(GPS.seconds, DEC); mySerial.print('.');
    mySerial.println(GPS.milliseconds);
    mySerial.print("Date: ");
    mySerial.print(GPS.day, DEC); mySerial.print('/');
    mySerial.print(GPS.month, DEC); mySerial.print("/20");
    mySerial.println(GPS.year, DEC);
    mySerial.print("Fix: "); mySerial.print((int)GPS.fix);
    mySerial.print(" quality: "); mySerial.println((int)GPS.fixquality);
    if (GPS.fix) {
      mySerial.print("Location: ");
      mySerial.print(GPS.latitude, 4); mySerial.print(GPS.lat);
      mySerial.print(", ");
      mySerial.print(GPS.longitude, 4); mySerial.println(GPS.lon);
      mySerial.print("Location (in degrees, works with Google Maps): ");
      mySerial.print(GPS.latitudeDegrees, 4);
      mySerial.print(", ");
      mySerial.println(GPS.longitudeDegrees, 4);

      mySerial.print("Speed (knots): "); mySerial.println(GPS.speed);
      mySerial.print("Angle: "); mySerial.println(GPS.angle);
      mySerial.print("Altitude: "); mySerial.println(GPS.altitude);
      mySerial.print("Satellites: "); mySerial.println((int)GPS.satellites);
    }
  }

}
