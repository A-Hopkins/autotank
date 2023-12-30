# Autotank
This is an AVR project to be used as a hw/sw testbed for prototyping lidar sensor algorithms, autonomous motor control, and connections between SOC and microcontrollers

## Goals
The goals for this project will be to have a small tracked vehicle controlled autonomously.

### Software
**Environment**:
The software environment is vscode with WSLGUI. The used toolchain is:
* avr-g++
* avrdude
* cmake

**C++**:
The embedded code will be written in c++ utilizing the feature rich usage of c++ for modern embedded development.

### Hardware
* ATMega328P (Arduino Uno R3)
* L293D motor controller with latch (adafruit L293D motor shield)
* Slamtec RPLIDAR A1M8
* Raspberry Pi Compute Module 4 and IO board

## How to Use
This is a software project with the goal to be slightly general with the intent of scaling hardware, but ultimately there is a degree of coupling that will happen. So there is no guaruntee that anything here will work for any other hardware than listed in the hardware section.

### Build
Using cmake to build an elf and hex file after avr-g++ and avrdude is installed. The cmake extension for vscode will provide the build. The workspace also has the build task configured to create.

### Deploy
The CMakeList also includes the avrdude command to upload the hex to the atmega328p. There are also a few other handy commands, like burning eeprom, fuses, and stripping the atmega328p. The CMakeList has some documentation to guide settings of variables a few to note would be the port, mcu, f_cpu, and programmer. Because I am using the arduino uno r3 I can just use the arduino programmer and upload over serial.