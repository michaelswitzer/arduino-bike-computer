/*********************************************************************
Mike's Bike Computer


*********************************************************************/
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>


// Display
#define DISP_SCK A0
#define DISP_MOSI A1
#define DISP_SS A2

#define BLACK 0
#define WHITE 1

Adafruit_SharpMem display(DISP_SCK, DISP_MOSI, DISP_SS);

// GPS
#define GPS_RX 1
#define GPS_TX 2

SoftwareSerial mySerial(GPS_TX, GPS_RX);

Adafruit_GPS GPS(&mySerial);

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// 10DOF
#define DOF_SDA A4
#define DOF_SCL A5
//
//Adafruit_10DOF                dof   = Adafruit_10DOF();
//Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

void initSensors()
{
//  if(!mag.begin())
//  {
//    /* There was a problem detecting the LSM303 ... check your connections */
//    display.println("Ooops, no LSM303 detected ... Check your wiring!");
//    while(1);
//  }
}

void setup() {
  // start & clear the display
  display.begin();
  display.clearDisplay();
  initSensors();
  delay(2000);

  

  // initialize GPS
  GPS.begin(9600);

 // GPS.sendCommand(PMTK_CMD_COLD_START);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // Set 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet


  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
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

uint32_t timer = millis();

void loop() {
  // put your main code here, to run repeatedly:

    sensors_vec_t   orientation;
    sensors_event_t mag_event;

    if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    }

  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data 
   // mySerial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
  
    display.clearDisplay();
    display.setTextColor(BLACK);
    
    display.setCursor(0,0);    
    display.setTextSize(3);
    if (GPS.fix != 0) {
      display.setTextColor(WHITE, BLACK);
    }
    
    if (GPS.hour < 10) {
      display.setCursor(18,10);  
    } else {
      display.setCursor(0,10);
    }
    display.print(GPS.hour, DEC); display.print(':');
    display.print(GPS.minute, DEC);
    

//    display.setCursor(0,50);  
//    display.print("Date: ");
//    display.print(GPS.day, DEC); display.print('/');
//    display.print(GPS.month, DEC); display.print("/20");
//    display.print(GPS.year, DEC);
    
//

    display.setCursor(0,80); 
    display.setTextSize(1);
    display.print("Fix: "); display.print((int)GPS.fix);
    display.print("\nFix quality: "); display.println((int)GPS.fixquality);

//      /* Calculate the heading using the magnetometer */
//  mag.getEvent(&mag_event);
//  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
//  {
//    /* 'orientation' should have valid .heading data now */
//    display.print(F("Heading: "));
//    display.print(orientation.heading);
//    display.print(F("; "));
//  }
    
//    if (GPS.fix) {
//      display.print("Location: ");
//      display.print(GPS.latitude, 4); display.print(GPS.lat);
//      display.print(", "); 
//      display.print(GPS.longitude, 4); display.println(GPS.lon);
//      display.print("Location (in degrees, works with Google Maps): ");
//      display.print(GPS.latitudeDegrees, 4);
//      display.print(", "); 
//      display.println(GPS.longitudeDegrees, 4);
//      
//      display.print("Speed (knots): "); display.println(GPS.speed);
//      display.print("Angle: "); display.println(GPS.angle);
//      display.print("Altitude: "); display.println(GPS.altitude);
//      display.print("Satellites: "); display.println((int)GPS.satellites);
//    }
    display.refresh();
  }
}
