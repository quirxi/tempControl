
#include <SPI.h>
#include "Adafruit_MAX31855.h"

int thermoCLK = 3;
int thermoCS = 4;
int thermoDO = 5;
 
// Initialize the Thermocouple
Adafruit_MAX31855 kTC(thermoCLK, thermoCS, thermoDO);
 
void setup() {
  Serial.begin(9600);
  Serial.println("MAX31855 Init"); 
  delay(500); 
}
 
void loop() {
    
  Serial.print("C = "); 
  Serial.println(kTC.readCelsius());   
  //Serial.print("F = ");
  //Serial.println(kTC.readFarenheit());
  // delay so it doesn't scroll too fast.
  delay(1000);
 
}
