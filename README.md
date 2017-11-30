# **Temperature Control**


A basic temperature control example that uses to read in a temperature via a K-Type thermocouple sensor (TODO).
The volt output of the sensor is processed by an MAX31855 board (TODO) and read out by an Arduino Nano (TODO).
The so gained temperature value is given out through the serial console of the Arduino Nano.
On the output side a mosfet/relay is connected to the arduino board on the PWM pins, thus controlling a connected heater till a given target temperature is reached.

## Setup

* TODO: photo from setup
* TODO: wiring
* TODO: test

## License

Distributed under the GNU LGPL v.3.0.

## References

* [MAX31855 K-Type thermoelement breakout board for arduino.](https://www.banggood.com/de/MAX31855-K-Type-Thermocouple-Breakout-Board-Temperature-Measurement-Module-For-Arduino-p-1086523.html?currency=EUR)
* [Adafruit's MAX31855 tutorial.](https://learn.adafruit.com/connecting-the-max31855-thermocouple-amplifier-breakout-to-an-electric-imp/breadboarding-the-circuit)
* [Adafruit-MAX31855-library](https://github.com/adafruit/Adafruit-MAX31855-library)
* [Newer version of SPI lib.](https://github.com/arduino/Arduino/tree/master/hardware/arduino/avr/libraries/SPI)
* LINK: Arduino Nano
* LINK: Arduino Anleitung
* LINK: MAX31855 kaufseite + specs
* LINK: K-Type Thermocouple specs
* LINK: MOSFET

## Authors:

* quirxi (https://github.com/quirxi)
* Cub3master (https://github.com/Cub3master)
 
