// ========= LIBRARY IMPORTS =========
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MAX31855.h>

// ========= PIN ASSIGNMENTS =========
// thermocouple digital pins
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

// temperature and pressure sensors
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

// SD card pins
#define SD_CS 53

// heaters
#define HEAT 9

// ========= SENSOR INITIALIZATION =========
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK); // temp and pressure sensor

HardwareSerial mySerial = Serial1; // GPS
Adafruit_GPS GPS(&Serial1);
#define GPSECHO  true
boolean usingInterrupt = false;


int max_altitude = 0;

// ========= SENSOR VALUE VARIABLES =========
int internal_temp = 0;
int external_temp = 0;
int pressure = 0;
int altitude = 0;
float altitude_feet = 0.0;
float latitude = 0.0; char lat = ' ';
float longitude = 0.0; char lon = ' ';

// ========= HELPER FUNCTION PROTOTYPES =========
void readSensors();
void regulateTemp();
void checkAltitude();
void deployParachute();
void writeHeaderToSD();
void writeDataToSD();

void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// ========= CONSTANTS =========
int TEMP_THRESHOLD = 0; // celcius
int ALTITUDE_DIFF_THRESHOLD = 100; // meters


// ========= SETUP FUNCTION =========
void setup() {
  Serial.begin(115200);

  // temp and pressure sensor test
  Serial.println("BMP280 test");
  if(!bme.begin(BMP_CS)){
    Serial.println("BMP initialization failed");
    return;
  }
  if(!SD.begin(SD_CS)){
    Serial.println("BMP initializaiton failed");
    return;
  }
  Serial.println("BMP initialization done.");

  // GPS setup
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status
  mySerial.println(PMTK_Q_RELEASE);

  pinMode(SD_CS, OUTPUT);
  writeHeaderToSD();
  
  delay(2000);
}

/*
 * MORE GPS SETUP
 */

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

// ========= MAIN LOOP FUNCTION =========
void loop() {
    readSensors();
    writeDataToSD();

    // run checks
    checkAltitude();
    regulateTemp();
    
    delay(1000);
}

// ========= HELPER FUNCTIONS =========

/* 
 * Reads all sensor values and stores them in global variables.
 */
void readSensors() {
    internal_temp = bme.readTemperature();
    
    pressure = bme.readPressure();
    
    altitude = bme.readAltitude(1011);
    altitude_feet = altitude * 3.28; // conversion meters to feet

    external_temp = thermocouple.readCelsius();

    // GPS
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    longitude = GPS.longitude;
    lon = GPS.lon;
    latitude = GPS.latitude;
    lat = GPS.lat;
}

/*
 * If temperature dips below a threshold, turn on heating pads.
 */
void regulateTemp() {
    if (internal_temp < TEMP_THRESHOLD) {
        // turn heaters on
    }
}

/* 
 * Checks if the payload is starting to fall. If so, deploy the parachute.
 */
void checkAltitude() {
    if (altitude > max_altitude) { // update max altitude
        max_altitude = altitude;
    }

    if (altitude <= (max_altitude - ALTITUDE_DIFF_THRESHOLD)) {
        deployParachute();
    }
}

/*
 * Heats up wire to cut the parachute cables.
 */
void deployParachute() {
    // fill me in!
}

/*
 * Writes CSV header row for data file.
 */
void writeHeaderToSD() {
    File myFile = SD.open("data.txt", FILE_WRITE);
    if(myFile) {
      myFile.println("INTERNAL_TEMP,EXTERNAL_TEMP,PRESSURE,ALTITUDE,ALTITUDE_FT,LONGITUDE,LATITUDE");
      Serial.println("wrote header");
      myFile.close();
    } else {
      Serial.println("error opening :(");
    }
}

/* 
 * Formats sensor readings into CSV and writes a new row
 * to data.txt on the SD card.
 */
void writeDataToSD() {
    File myFile = SD.open("data.txt", FILE_WRITE);
    if(myFile) {
        myFile.print(internal_temp);
        myFile.print(",");
        myFile.print(external_temp);
        myFile.print(",");
        myFile.print(pressure);
        myFile.print(",");
        myFile.print(altitude);
        myFile.print(",");
        myFile.print(altitude_feet);
        myFile.print(",");
        myFile.print(longitude + String(lon));
        myFile.print(",");
        myFile.print(latitude + String(lat));
        myFile.println("");
        Serial.println("wrote to file");
        myFile.close();
    } else {
        Serial.println("error opening :(");
    }  
}

