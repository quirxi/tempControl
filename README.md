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

* [henrysbench arduino-max31855 tutorial](http://henrysbench.capnfatz.com/henrys-bench/arduino-temperature-measurements/max31855-arduino-k-thermocouple-sensor-manual-and-tutorial/)
* [Adafruit-MAX31855-library](https://github.com/adafruit/Adafruit-MAX31855-library)
* [Newer version of SPI lib](https://github.com/arduino/Arduino/tree/master/hardware/arduino/avr/libraries/SPI)
* [MAX31855 specs and datasheet](https://www.maximintegrated.com/en/products/analog/sensors-and-sensor-interface/MAX31855.html)
* [MAX31855 K-Type thermoelement breakout board for arduino](https://www.banggood.com/de/MAX31855-K-Type-Thermocouple-Breakout-Board-Temperature-Measurement-Module-For-Arduino-p-1086523.html?currency=EUR)
* [ATmega328 Datasheet](http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf)
* [Arduino Nano Pinout Diagram](http://www.pighixxx.com/test/wp-content/uploads/2014/11/nano.png)
* LINK: Arduino Nano
* LINK: Arduino Anleitung
* LINK: K-Type Thermocouple specs
* LINK: MOSFET



## Authors:

* quirxi (https://github.com/quirxi)
* Cubemast3r (https://github.com/Cubemast3r)
 
