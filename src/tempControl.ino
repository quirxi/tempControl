/*
   This program uses interrupts for (longer) timing tasks in the seconds range.

   @see: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/
   @see: https://arduino-projekte.webnode.at/registerprogrammierung/timer-interrupt/
   @see: http://www.gammon.com.au/interrupts
   @see: http://www.gammon.com.au/timers
   @see: http://www.instructables.com/id/Arduino-Timer-Interrupts/
   @see: http://maxembedded.com/2011/07/avr-timers-ctc-mode/
 * */
////////////////////////////////////////////////////////////////////////////////////////////
// MAX31855 k-type thermocouple:
//                creating a thermocouple instance with hardware SPI on a given CS pin.
////////////////////////////////////////////////////////////////////////////////////////////
#include "Adafruit_MAX31855.h"
#define MAXCS   10
Adafruit_MAX31855 thermocouple(MAXCS);

////////////////////////////////////////////////////////////////////////////////////////////
// PID
////////////////////////////////////////////////////////////////////////////////////////////
#include "PIDcontrol.h"

// set the pins for input, output and sample frequency here:
const unsigned short PID_INPUT_PIN = 0; // pin from which sensor input is read
const unsigned short PID_RELAY_PIN = 5; // pin that is used to control relay

// specify the PID constants here:
double cProportional = 1000;            // the proportional constant
double cIntegral = 0;                   // the integral constant
double cDerivative = 0;                 // the derivative constant

double pidSetpoint;                     // the desired target setpoint
volatile double pidInput;               // the input to the PID controller as read from the sensor (=PID_INPUT_PIN)
volatile double pidLastInput;           // the last input to be remembered in case the current reading is NAN (Not a Number) and cannot be used for computation.
volatile double pidOutput;              // specifies how long the relay is switched on within a fixed period of time (=sampleInterval)
unsigned short  sampleInterval;         // a fixed period of time that determines a full PID on/off cycle (ms) ATTENTION: sampleInterval [ms] is the same as INTERRUPT_INTERVAL[s] and cannot be greater than 4.1944 seconds
volatile bool   errorReading;           // is set when temperature reading returned NAN
// initialize the PID controller
PIDcontrol myPID(&pidInput, &pidOutput, &pidSetpoint, cProportional, cIntegral, cDerivative, DIRECT);

////////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT
////////////////////////////////////////////////////////////////////////////////////////////

// ATTENTION: Here TIMER 1 is used which is a 16-bit timer. Thus the compare value cannot be greater than 65535, otherwise the timer would overflow.
//            So the maximum time between two interrupts with a cpu frequency of 16000000 hz can only be:
//                OCR1A = CMP_VALUE = INTERRUPT_INTERVAL * TICKS_PER_SEC - 1
//                => INTERRUPT_INTERVAL = CMP_VALUE / (TICKS_PER_SEC - 1)
//                => INTERRUPT_INTERVAL = CMP_VALUE / (( CPU_FREQ / PRESCALER ) - 1)
//                => INTERRUPT_INTERVAL = 65535 / (( 16000000 / 1024 ) - 1)
//                => INTERRUPT_INTERVAL = 4.1945 seconds

#define CPU_FREQ 16000000L    // cpu clock from boards.txt
#define PRESCALER 1024        // cpu prescaler

// NOTE: we use seconds here as a basis because when using miliseconds the precision of the number would be inexact due to floating point calculations.
const unsigned short TICKS_PER_SEC = ( CPU_FREQ / PRESCALER );            // base for calculation of compare value for output compare register OCR1A
const unsigned short INTERRUPT_INTERVAL = 4;                              // interrupt every 4 seconds (=maximum interrupt intervall and maximum sample pid sample time)
const unsigned short CMP_VALUE = INTERRUPT_INTERVAL * TICKS_PER_SEC - 1;  // compare value for OCR1A: = amount of ticks in an interrupt intervall
const unsigned short MAX_CMP_VALUE = 65535 - 1;                           // maximum number in a 16-bit variable

volatile unsigned long counter = 0;
volatile unsigned long cntCmp = 0;

////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    // MAX31555 thermocouple and serial initialization
    Serial.begin(9600);
    while (!Serial) delay(1);           // wait for serial on Leonardo/Zero, etc
    delay(500);                         // wait for MAX chip to stabilize
    // SSR initialization
    pinMode(PID_RELAY_PIN, OUTPUT);     // set the relay pin to output
    digitalWrite(PID_RELAY_PIN, LOW);   // initialize relay pin to low
    //PID
    pinMode(PID_RELAY_PIN, OUTPUT);              // set the relay pin to output
    digitalWrite(PID_RELAY_PIN, LOW);            // initialize relay pin to low
    pidSetpoint = 55;                            // set the desired target value
    sampleInterval = INTERRUPT_INTERVAL * 1000;  // the sample time of the pid controller is the interrupt frequency in miliseconds
    myPID.SetSampleTime(sampleInterval);         // tell the PID controller how frequently we will read a sample and calculate output
    myPID.SetOutputLimits(0, sampleInterval);    // tell the PID to range between 0 and the full window size
    myPID.SetMode(AUTOMATIC);                    // turn the PID on
    pidInput = thermocouple.readCelsius();       // read temperature sensor input
    myPID.Compute();                             // compute pidOutput for the first time

    Serial.print("CPU_FREQ:            ");
    Serial.println(CPU_FREQ);
    Serial.print("PRESCALER:           ");
    Serial.println(PRESCALER);
    Serial.print("TICKS_PER_SEC:       ");
    Serial.println(TICKS_PER_SEC);
    Serial.print("CMP_VALUE:           ");
    Serial.println(CMP_VALUE);
    Serial.print("MAX_CMP_VALUE:       ");
    Serial.println(MAX_CMP_VALUE);
    Serial.print("INTERRUPT_INTERVAL: ");
    Serial.println(INTERRUPT_INTERVAL);

    Serial.print("sampleInterval: ");
    Serial.print(sampleInterval);
    Serial.print(" | Setpoint: ");
    Serial.print(pidSetpoint);
    Serial.print(" | P: ");
    Serial.print(cProportional);
    Serial.print(" I: ");
    Serial.print(cIntegral);
    Serial.print(" D: ");
    Serial.println(cDerivative);

    // INTERRUPTS
    cli();                               // disable global interrupts
    TCNT1 = 0;                           // delete timer counter register
    TCCR1A = 0;                          // delete TCCR1A-Registers
    TCCR1B = 0;                          // delete TCCR1B-Registers
    TCCR1B |= (1 << WGM12);              // CTC-Mode (Waveform Generation Mode): resets TCNT1 to0 after interrupt, makes OCR1A the leading compare register
    TCCR1B |= (1 << CS12) | (1 << CS10); // set prescaler to 1024: CS10 und CS12 (Clock Select)
    OCR1A = CMP_VALUE;                   // set compare value
    OCR1B = MAX_CMP_VALUE;               // a second comapare value in OCR1B can be used as long as its value is *lower* than that of OCR1A: here its higher so it wont be called in the beginning
    TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B); // enable interrupts: set output compare interrupt enable for 1A and 1B
    sei();                               // enable global interrupts
}

////////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPTS
////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER1_COMPA_vect)   // Interrupt Service Routine for timer 1 OCR1A
{
    pidInput = thermocouple.readCelsius(); // read temperature sensor input
    if (!isnan(pidInput)) pidLastInput = pidInput;
    else
    {
        pidInput = pidLastInput;
        errorReading = true;
    }
    myPID.Compute();                       // compute pidOutput

    if (pidOutput > 0)
        digitalWrite(PID_RELAY_PIN, HIGH);
    else
        digitalWrite(PID_RELAY_PIN, LOW);

    // trigger interrupt b to turn off relay within the sample time (=sampleInterval)
    if ( pidOutput > 0 && pidOutput < sampleInterval )
        OCR1B = ((pidOutput * TICKS_PER_SEC) / 1000) - 1;
    else
        OCR1B = MAX_CMP_VALUE;    // we set OCR1B higher than OCR1A so that the second b interrupt never gets called

    ++counter;                    // flag that signals the start of a new cycle to the main loop
}

// Interrupt Service Routine for timer 1 OCR1B
ISR(TIMER1_COMPB_vect)
{
    digitalWrite(PID_RELAY_PIN, LOW);
}

////////////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
    if (cntCmp != counter)    // use != for comparison and not < because it could give problems when unsigned long overflows
    {
        Serial.print(counter);
        Serial.print(": ");
        Serial.print(millis());
        if (errorReading)   // for error codes see: https://cdn-shop.adafruit.com/datasheets/MAX31855.pdf#page=10 
        {
            Serial.print(":\tERROR: ");
            uint8_t errorCode = thermocouple.readError();
            switch( errorCode )
            {
            case 1:
                Serial.println("(1) Thermocouple has no connection (open circuit)!");
                break;
            case 2:
                Serial.println("(2) Thermocouple short-circuited to GND!");
                break;
            case 4:
                Serial.println("(4) Thermocouple short-circuited to VCC!");
                break;
            default:
                Serial.print("(");
                Serial.print(errorCode);
                Serial.println(") Unkown error code!");
            }
            errorReading=false;    // IMPORTANT: reset error flag !!
        }
        else
        {
            Serial.print(":\tpidInput: ");
            Serial.print( pidInput);
            Serial.print(" | pidOutput: ");
            Serial.print(pidOutput);
            Serial.print(" | OCR1A: ");
            Serial.print(OCR1A);
            Serial.print(" | OCR1B: ");
            Serial.println(OCR1B);
        }
        cntCmp = counter;
    }
}
