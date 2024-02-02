# Tamagotchi Project - Virtual Pet Simulation

## Overview

This repository contains the project developed for the "Introduntion to the computer systems" course at the University of Oulu. The project is implemented in C and is designed for Texas Instruments SensorTag. The aim for this project was gain experience with embedded systems. The project aims to recreate the experience of nurturing a virtual pet through interactive and engaging means.

## Project Description 

The Tamagotchi lives within course's IoT backend system, and user interact with it using SensorTag. Actions such as feeding, playing and petting are executed through specific movemets. After recognized movement SensorTag sends command to backend based on movement. SensorTag also receives messages from backend.

## Key Features:

#### Multiple tasks Using RTOS:
There are separate tasks for handling MPU and UART (Universal Asynchronous Receiver-Transmitter) communication.
#### Finite State Machine Implementation:
The code uses an enumeration-based finite state machine for managing different states of the program and the virtual pet's activities. This allows for organized and efficient state transitions based on user interactions and sensor data.
#### Button and LED Handling:
The code includes configurations for buttons and LEDs, allowing user interaction through physical hardware. Buttons are used to change the state of the program, and LEDs provide visual feedback.
#### Sensor Interaction:
The code interacts with an MPU9250, which is a motion tracking device. It reads acceleration and gyroscope data, crucial for determining the virtual pet's actions based on physical movements of the device.
#### Feedback System:
The program gives feedback via buzzer based on the pet's responses.
#### UART Communication:
The communication between the SensorTag and the backend system is facilitated through UART (Universal Asynchronous Receiver-Transmitter), ensuring real-time data transfer and responsiveness.
