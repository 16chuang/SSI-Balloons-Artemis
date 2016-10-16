// ========= LIBRARY IMPORTS =========
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "Adafruit_MAX31855.h"

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

int max_altitude = 0;

// ========= SENSOR VALUE VARIABLES =========
int internal_temp = 0;
int external_temp = 0;
int pressure = 0;
int altitude = 0;
float latitude = 0.0;
float longitude = 0.0;

// ========= HELPER FUNCTION PROTOTYPES =========
void readAndLogSensors();
void regulateTemp();
void checkAltitude();
void deployParachute();
void sensorWriteToSD(float data, String sensorName);

// ========= CONSTANTS =========
int TEMP_THRESHOLD = 0; // celcius
int ALTITUDE_DIFF_THRESHOLD = 100; // meters


// ========= SETUP FUNCTION =========
void setup() {
  Serial.begin(9600);

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

  pinMode(SD_CS, OUTPUT);
  
  delay(2000);
}

// ========= MAIN LOOP FUNCTION =========
void loop() {
    readAndLogSensors();

    // run checks
    checkAltitude();
    regulateTemp();
    
    delay(1000);
}

// ========= HELPER FUNCTIONS =========

/* 
 * Reads all sensor values, stores them in global variables, 
 * and writes them to the SD card.
 */
void readAndLogSensors() {
    internal_temp = bme.readTemperature();
    sensorWriteToSD(internal_temp, "internal_temp");

    pressure = bme.readPressure();
    sensorWriteToSD(pressure, "pressure");

    altitude = bme.readAltitude(1011);
    sensorWriteToSD(altitude, "altitude_meters");
    float altitude_feet = altitude * 3.28; // conversion meters to feet
    sensorWriteToSD(altitude_feet, "altitude_feet"); 

    external_temp = thermocouple.readCelsius();
    sensorWriteToSD(external_temp, "external_temp");
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
 * Given a sensor reading and a name, the function formats 
 * and logs values to the appropriate files on the SD card.
 */
void sensorWriteToSD(float data, String sensorName){
    File myFile = SD.open(sensorName+".txt", FILE_WRITE);
    if(myFile) {
        myFile.print(data);
        Serial.print("wrote ");
        Serial.println(data);
        myFile.close();
        Serial.println("done.");
    } else {
        Serial.print("error opening ");
        Serial.println(sensorName);
    }  
}

