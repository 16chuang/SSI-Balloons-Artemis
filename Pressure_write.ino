#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SD_CS 53
#define HEAT 9
Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
void sensorWrite(float data, String sensorName);

void setup() {
  Serial.begin(9600);
  Serial.println("BMP280 test");
  if(!bme.begin(BMP_CS)){
    Serial.println("initialization failed");
    return;
  }
  if(!SD.begin(SD_CS)){
    Serial.println("initializaiton failed");
    return;
  }
  Serial.println("initialization done.");
  pinMode(SD_CS, OUTPUT);
  delay(2000);
}

void loop() {
    Serial.println("sup");
    float temp = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(temp);
    sensorWrite(temp,"bmp.txt");
    Serial.println(" *C");

    float pressure = bme.readPressure();
    Serial.print(F("Pressure = "));
    Serial.print(pressure);
    sensorWrite(pressure,"bmp");
    Serial.println(F(" Pa"));

    float altitude = bme.readAltitude(1011);
    Serial.print(F("Approx altitude = "));
    Serial.print(altitude);
    sensorWrite(altitude,"bmp");
    Serial.println(F(" m"));
    
    Serial.println();
    delay(2000);
}

void sensorWrite(float data, String sensorName){
  Serial.println(sensorName);
  File myFile = SD.open(sensorName, FILE_WRITE);
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

