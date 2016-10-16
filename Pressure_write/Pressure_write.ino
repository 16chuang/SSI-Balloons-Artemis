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
void checkParachute();
void sensorWriteToSD(float data, String sensorName);

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

void loop() {
    readAndLogSensors();
    
    delay(1000);
}

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
    sensorWriteToSD(altitude, "altitude");

    external_temp = thermocouple.readCelsius();
    sensorWriteToSD(external_temp, "external_temp");
}

void regulateTemp() {
  
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

