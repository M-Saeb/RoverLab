# Race Rover
## Description
A project for an embedded esp32 chip that runs a mini rover robot for a racing challenge

## The race challenge
To write a procedural algorithm that takes the coordinates of the rover and of the "target" as input, and covers the following point
- direct the rover into the "target"
- avoid any obstacles along the way


## Project setup
The used hardware was an ESP32 chip that is wired with:
1. 2 motors for moving (left and right)
2. a distance sensor for detecting objects that are front or next to the rover

The used protocol for getting the coordinates of the rover and the "target" is an AWS MQTT Broker

## Code Structure
*Used PlatformIO & VScode for ease of development*

### motorDriver.h & motorDriver.cpp
For handling the motors

### sensorDriver.h & sensorDriver.cpp
For handling the distance sensor reading

### AWS.h & AWS.cpp
For handling the connection with the AWS MQTT Broker

### secrets.h
For saving all the confidentials values in one place

### main.cpp
The code the main project algorithms