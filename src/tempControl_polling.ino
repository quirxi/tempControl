#include "Adafruit_MAX31855.h"
#include <PID_v1.h>

// @TODO: Error handling when readCelsius returns NAN

// set the pins for input, output and sample frequency here:
//const unsigned short PID_INPUT_PIN = 0; // pin from which sensor input is read
const unsigned short PID_RELAY_PIN = 5; // pin that is used to control relay
const unsigned int PID_CYCLE = 4000;    // a fixed period of time that determines a full PID on/off cycle (ms)

// specify the PID constants here:
double cProportional = 1000;          // the proportional constant
double cIntegral = 0;                 // the integral constant
double cDerivative = 0;               // the derivative constant

double pidSetpoint;                     // the desired target setpoint
double pidInput;                        // the input to the PID controller as read from the sensor (=PID_INPUT_PIN)
double pidOutput;                       // specifies how long the relay is switched on within a fixed period of time (=PID_CYCLE)
unsigned long pidStart;                 // variable that marks the start of each PID_CYCLE
// initialize the PID controller
PID myPID(&pidInput, &pidOutput, &pidSetpoint, cProportional, cIntegral, cDerivative, DIRECT);


// creating a thermocouple instance with hardware SPI on a given CS pin.
#define MAXCS   10
Adafruit_MAX31855 thermocouple(MAXCS);


////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // MAX31555 thermocouple and serial initialization
  Serial.begin(9600);
  while (!Serial) delay(1); // wait for serial on Leonardo/Zero, etc
  delay(500);  // wait for MAX chip to stabilize
  // PID and SSR initialization
  pinMode(PID_RELAY_PIN, OUTPUT);       // set the relay pin to output
  digitalWrite(PID_RELAY_PIN, LOW);     // initialize relay pin to low
  pidSetpoint = 50;                     // set the desired target value
  myPID.SetSampleTime(PID_CYCLE);       // tell the PID controller how frequently we will read a sample and calculate output
  myPID.SetOutputLimits(0, PID_CYCLE);  // tell the PID to range between 0 and the full window size
  myPID.SetMode(AUTOMATIC);             // turn the PID on
  pidStart = millis();                  // here our first pid cycle (=PID_CYCLE) starts
  pidInput = thermocouple.readCelsius();// read sensor input
  myPID.Compute();                      // compute pidOutput for the first time
  Serial.print("\nPid Cycle: "); Serial.print(PID_CYCLE); Serial.print(" | Setpoint: "); Serial.print(pidSetpoint); Serial.print(" | P: "); Serial.print(cProportional); Serial.print(" I: "); Serial.print(cIntegral); Serial.print(" D: "); Serial.println(cDerivative);
}
////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // compute pidOutput each PID_CYCLE
  if (millis() - pidStart >= PID_CYCLE)
  {
    pidStart = millis();                    // reset pidStart to begin of new PID_CYCLE
    pidInput = thermocouple.readCelsius();  // read sensor input
    // compute pidOutput
    if (!myPID.Compute()) Serial.println("Error in compute()!!");
    Serial.print(pidStart); Serial.print("\tTemperature C: "); Serial.print( pidInput); Serial.print(" | pidOutput: "); Serial.println(pidOutput);
    if (pidOutput > 0)
    {
      digitalWrite(PID_RELAY_PIN, HIGH);
      if (pidOutput < PID_CYCLE)            // only switch off relay when necessary, otherwise leave it on till next PID_CYCLE starts
      {
        delay(pidOutput);
        digitalWrite(PID_RELAY_PIN, LOW);
      }
    }
    else digitalWrite(PID_RELAY_PIN, LOW);  // set relay pin low when pidOutput = 0
  }
}
