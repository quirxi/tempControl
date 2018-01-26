# **Temperature Control**


A basic temperature control example that uses to read in a temperature via a K-Type thermocouple sensor (TODO).
The volt output of the sensor is processed by an MAX31855 board (TODO) and read out by an Arduino Nano (TODO).
The so gained temperature value is given out through the serial console of the Arduino Nano.
On the output side a mosfet/relay is connected to the arduino board on the PWM pins, thus controlling a connected heater till a given target temperature is reached.

## Instructions

### What you need:

* [Thermocouple](https://www.banggood.com/de/Universal-K-Type-EGT-Thermocouple-Temperature-Sensors-For-Exhaust-Gas-Probe-p-1011377.html?rmmds=search&cur_warehouse=CN)
* [MAX31855 K-Type thermoelement breakout board for arduino](https://www.banggood.com/de/MAX31855-K-Type-Thermocouple-Breakout-Board-Temperature-Measurement-Module-For-Arduino-p-1086523.html?currency=EUR)
* [Arduino Nano clone](https://www.banggood.com/de/ATmega328P-Arduino-Compatible-Nano-V3-Improved-Version-No-Cable-p-959231.html?gmcCountry=AT&currency=EUR&createTmp=1&utm_source=googleshopping&utm_medium=cpc_elc&utm_content=zouzou&utm_campaign=pla-at-arduino-pc&gclid=EAIaIQobChMIm8_gxOD-1gIVBl8ZCh32wQj4EAQYAiABEgIKwfD_BwE&cur_warehouse=CN)
* you will also need a soldering iron, jumper cables, breadbord etc.

### Setup

1. Resolder the plug onto the other side of the MAX31855K breakout board, so that it can be easily attach it to the breadboard.

<img src="docs/img/MAX31855K_Breakout_front.jpg" alt="Original MAX31855K breakout board front" height="250">
<img src="docs/img/MAX31855K_Breakout_back.jpg" alt="Original MAX31855K breakout board back" height="250">
<img src="docs/img/MAX31855K_Breakout_resoldered.jpg" alt="MAX31855K breakout board resoldered" height="250">

2. Download and install the newest Arduino IDE from https://www.arduino.cc/en/Main/Software. 

3. We downloaded the newest version of the SPI library from https://github.com/arduino/Arduino/tree/master/hardware/arduino/avr/libraries/SPI and copied it to the Arduino libraries folder in /usr/share/arduino/libraries/SPI/SPI.h. 

4. Connect the Arduino nano with the MAX31855K breakout board according to following circuit diagram.

<img src="docs/img/MAX31855K_Breakout_ArduinoNano_wiring.jpg" alt="Breadboard wiring" height="250">
<img src="docs/img/tempControl_wiring.png" alt="Wiring diagram" height="500">

* You can find the Fritzing wiring diagram file (tempControl.fzz) in the docs folder.



### Explanation

1. K Type Thermocouple current changes with temperature (Peltier-effect)

2. MAX31855K Pinouts

3. SPI protocol

4. Atmel ATmega nano pinouts

5. programming

* TODO: photo from setup
* TODO: wiring
* TODO: test

## License

Distributed under the GNU LGPL v.3.0.

## References

* [Thermocouple](https://www.banggood.com/de/Universal-K-Type-EGT-Thermocouple-Temperature-Sensors-For-Exhaust-Gas-Probe-p-1011377.html?rmmds=search&cur_warehouse=CN)
* [MAX31855 K-Type thermoelement breakout board for arduino](https://www.banggood.com/de/MAX31855-K-Type-Thermocouple-Breakout-Board-Temperature-Measurement-Module-For-Arduino-p-1086523.html?currency=EUR)
* [Arduino Nano clone](https://www.banggood.com/de/ATmega328P-Arduino-Compatible-Nano-V3-Improved-Version-No-Cable-p-959231.html?gmcCountry=AT&currency=EUR&createTmp=1&utm_source=googleshopping&utm_medium=cpc_elc&utm_content=zouzou&utm_campaign=pla-at-arduino-pc&gclid=EAIaIQobChMIm8_gxOD-1gIVBl8ZCh32wQj4EAQYAiABEgIKwfD_BwE&cur_warehouse=CN)
* [henrysbench arduino-max31855 tutorial](http://henrysbench.capnfatz.com/henrys-bench/arduino-temperature-measurements/max31855-arduino-k-thermocouple-sensor-manual-and-tutorial/)
* [Adafruit-MAX31855-library](https://github.com/adafruit/Adafruit-MAX31855-library)
* [Newer version of SPI lib](https://github.com/arduino/Arduino/tree/master/hardware/arduino/avr/libraries/SPI)
* [MAX31855 specs and datasheet](https://www.maximintegrated.com/en/products/analog/sensors-and-sensor-interface/MAX31855.html)
* [ATmega328 Datasheet](http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf)
* [Arduino Nano Pinout Diagram](http://www.pighixxx.com/test/wp-content/uploads/2014/11/nano.png)
* LINK: Arduino Nano
* LINK: Arduino Anleitung
* LINK: K-Type Thermocouple specs
* LINK: MOSFET



## Authors:

* quirxi (https://github.com/quirxi)
* Cubemast3r (https://github.com/Cubemast3r)
 
