#include <SPI.h>
#include "Adafruit_MAX31855.h"

// ------- THERMOCOUPLE SETUP -------
// thermocouple digital pins
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5
// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

void setup() {
  while (!Serial); // wait for Serial on Leonardo/Zero, etc
  Serial.begin(9600);
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  // ------- THERMOCOUPLE READING -------
   Serial.println(thermocouple.readInternal()); // internal temp of thermocouple chip
   double c = thermocouple.readCelsius();
   if (!isnan(c)) { Serial.println(c); } // external temperature of thermocouple
 
   delay(1000);
}
