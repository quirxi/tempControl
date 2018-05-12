# **Temperature Control**


A basic temperature control example that reads in a temperature via a K-Type thermocouple sensor.
The volt output of the sensor is processed by an MAX31855 board and read out by an Arduino Nano.
The so gained temperature value is given out to the serial console of the Arduino Nano.
On an output pin of the arduino nano a Solid State Relay (SSR) is connected. 
A PID controller is switching the SSR till the given target temperature is reached.


## Instructions


### What you need:

* [Thermocouple](https://www.banggood.com/de/Universal-K-Type-EGT-Thermocouple-Temperature-Sensors-For-Exhaust-Gas-Probe-p-1011377.html?rmmds=search&cur_warehouse=CN)
* [MAX31855 K-Type thermoelement breakout board for arduino](https://www.banggood.com/de/MAX31855-K-Type-Thermocouple-Breakout-Board-Temperature-Measurement-Module-For-Arduino-p-1086523.html?currency=EUR)
* [Arduino Nano clone](https://www.banggood.com/de/ATmega328P-Arduino-Compatible-Nano-V3-Improved-Version-No-Cable-p-959231.html?gmcCountry=AT&currency=EUR&createTmp=1&utm_source=googleshopping&utm_medium=cpc_elc&utm_content=zouzou&utm_campaign=pla-at-arduino-pc&gclid=EAIaIQobChMIm8_gxOD-1gIVBl8ZCh32wQj4EAQYAiABEgIKwfD_BwE&cur_warehouse=CN)
* [Solid State Relay](https://www.banggood.com/80A-SSR-80DA-Solid-State-Relay-Module-DC-To-AC-24V-380V-Output-p-1097188.html)
* you will also need a soldering iron, jumper cables, breadbord etc.


### Setup

__Hardware__

1. Resolder the plug onto the other side of the MAX31855K breakout board, so that it can be easily attach it to the breadboard.

<img src="docs/img/MAX31855K_Breakout_front.jpg" alt="Original MAX31855K breakout board front" height="250">
<img src="docs/img/MAX31855K_Breakout_back.jpg" alt="Original MAX31855K breakout board back" height="250">
<img src="docs/img/MAX31855K_Breakout_resoldered.jpg" alt="MAX31855K breakout board resoldered" height="250">

2. Connect the Arduino nano with the MAX31855K breakout board according to following circuit diagram.

<img src="docs/img/MAX31855K_Breakout_ArduinoNano_wiring.jpg" alt="Breadboard wiring" height="250">
<img src="docs/img/tempControl_wiring.png" alt="Wiring diagram" height="500">

* You can find the Fritzing wiring diagram file (tempControl.fzz) in the docs folder.

__Software__

1. Download and install the newest Arduino IDE from https://www.arduino.cc/en/Main/Software. 

2. If you need help using Arduino have a look at www.arduino.cc/en/Guide/HomePage. 
    
3. Install the newest Arduino IDE from https://www.arduino.cc/en/main/software (version 1.8.5 at time of writing). 
    
4. Open "Menu->Sketch->Include Libraries->Manage Libraries.." in the Arduino IDE and search for "Adafruit MAX31855" and "PID" by Brett Beauregard (version 1.2.0 at time of writing) and install it. 


### Programming

1. Open the "tempControl.ino" file that you can find here, "Verify" and "Upload" it to your Arduino board via USB connection
    
2. Once the code upload finishes verifying, open the serial monitor (found in the ‘Tools’ menu). You should be able to read the temperature your thermocouple is detecting on the serial monitor in real time. If it isn’t working, make sure you have assembled the circuit and uploaded the code to your board correctly. 


### Explanation

__1. K Type Thermocouple__

A thermocouple is a kind of temperature sensor, simply made by welding together two metal wires. Because of a physical effect of two joined metals, there is a slight but measurable voltage across the wires that increases with temperature (Seebeck-Effekt). The main advantage of using a thermocouple is the high temperature range (our K type: -200°C to 1200°C). A difficulty in using them is that the voltage to be measured is very small, with changes of about 50 uV per °C. To solve this we are using a thermocouple interface chip (MAX31855K) to measure the voltage between the wires. If you find that the thermocouple temperature goes down instead of up when heated, when connecting the K type thermocouples with the MAX31855K, try swapping the red and yellow wires.

__2. MAX31855K__

The MAX31855K is a chip that converts the voltage output of the thermocouple to a digital temperature value that is read out via SPI 
protocol. The first assembly step is creating a reliable, electrical connection from the MAX31855K breakout board to your Arduino. We chose 
to use a breakout board because we wanted to connect it to a bread board. The hookup is fairly straightforward. Connect the Arduino, MAX31855K and thermocouple as described in the Fritzing wiring diagram.

__3. Arduino nano__

The Arduino nano communicates with the MAX31855K through SPI protocol. For further details see next point.

__4. SPI protocol__

The SPI is a fast synchronous protocol that tipically uses uses 4 pins for communication, wiz. MISO, MOSI, SCK, and SS. These pins are directly related to the SPI bus interface.

    1. MISO – MISO stands for Master In Slave Out. MISO is the input pin for Master AVR, and output pin for Slave AVR device. Data transfer from Slave to Master takes place through this channel.
    2. MOSI – MOSI stands for Master Out Slave In. This pin is the output pin for Master and input pin for Slave. Data transfer from Master to Slave takes place through this channel.
    3. SCK – This is the SPI clock line (since SPI is a synchronous communication).
    4. SS – This stands for Slave Select. This pin would be discussed in detail later in the post.
    
__5. PID controller__

PID Controller is a most common control algorithm used in industrial automation & applications. PID controllers are used for precise and accurate control of various parameters. Most often these are used for the regulation of temperature, pressure, speed, flow and other process variables. For in depth information about how PID controller work see reference section.

## License

Distributed under the GNU LGPL v.3.0.

## References

* [Thermocouple](https://www.banggood.com/de/Universal-K-Type-EGT-Thermocouple-Temperature-Sensors-For-Exhaust-Gas-Probe-p-1011377.html?rmmds=search&cur_warehouse=CN)
* [MAX31855 K-Type thermoelement breakout board for arduino](https://www.banggood.com/de/MAX31855-K-Type-Thermocouple-Breakout-Board-Temperature-Measurement-Module-For-Arduino-p-1086523.html?currency=EUR)
* [Arduino Nano clone](https://www.banggood.com/de/ATmega328P-Arduino-Compatible-Nano-V3-Improved-Version-No-Cable-p-959231.html?gmcCountry=AT&currency=EUR&createTmp=1&utm_source=googleshopping&utm_medium=cpc_elc&utm_content=zouzou&utm_campaign=pla-at-arduino-pc&gclid=EAIaIQobChMIm8_gxOD-1gIVBl8ZCh32wQj4EAQYAiABEgIKwfD_BwE&cur_warehouse=CN)
* [Solid State Relay](https://www.banggood.com/80A-SSR-80DA-Solid-State-Relay-Module-DC-To-AC-24V-380V-Output-p-1097188.html)
* [henrysbench arduino-max31855 tutorial](http://henrysbench.capnfatz.com/henrys-bench/arduino-temperature-measurements/max31855-arduino-k-thermocouple-sensor-manual-and-tutorial/)
* [MAX31855 specs and datasheet](https://datasheets.maximintegrated.com/en/ds/MAX31855.pdf)
* [ATmega328 Datasheet](http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf)
* [Arduino Nano Pinout Diagram](http://www.pighixxx.com/test/wp-content/uploads/2014/11/nano.png)
* [Arduino Anleitung](www.arduino.cc/en/Guide/HomePage)
* [K-Type Thermocouple specs](https://www.thermocoupleinfo.com/type-k-thermocouple.htm)
* [PID diagrams](http://ww1.microchip.com/downloads/en/AppNotes/Atmel-2558-Discrete-PID-Controller-on-tinyAVR-and-megaAVR_ApplicationNote_AVR221.pdf)
* [Working of a PID controller](https://www.elprocus.com/the-working-of-a-pid-controller/)
* [PID controller](http://manuals.chudov.com/Servo-Tuning/PID-without-a-PhD.pdf)
* [PID controlling algorithm](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
* [PID Arduino library](https://github.com/br3ttb/Arduino-PID-Library)
* [PID for dummies](https://www.csimn.com/CSI_pages/PIDforDummies.html)


## TODOs

* Solid State Relais
* GUI
* [PID tuning](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
* Change Fritzing pictures and table for hardware SPI.
* Pictures of new setpup with water kettle


## Authors

* quirxi (https://github.com/quirxi)
* Cubemast3r (https://github.com/Cubemast3r)
 
