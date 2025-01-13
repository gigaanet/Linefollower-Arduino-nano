# Line Follower Bot with Arduino Nano

This documentation will guide you through building a line follower bot using an Arduino Nano, QTR sensors, a motor driver, and a few other components. The guide is beginner-friendly and provides a comprehensive understanding of how each component works together to create a fully functional line follower bot.

# Table of Contents

1. Introduction


2. Components Needed


3. Circuit Diagram


4. Wiring and Connections


5. Software Setup


6. Code Explanation


7. Testing and Calibration


8. Troubleshooting


9. Conclusion



# 1. Introduction

A line follower bot is a basic and popular project in robotics, designed to follow a predefined path or line on the floor. The bot uses sensors to detect the line and adjusts its movements to stay on track. This project is ideal for beginners to learn about sensors, motor control, and basic control algorithms like PID (Proportional, Integral, Derivative).

# 2. Components Needed

# Essential Components

Arduino Nano: The microcontroller that acts as the brain of the bot, executing the code and controlling all components.

QTR Sensors (8-sensor array): These sensors detect the line by measuring the reflectance of the surface. The bot uses these readings to determine its position relative to the line.

Motor Driver (L298N or similar): A motor driver is needed to control the DC motors' speed and direction, as the Arduino cannot supply enough current directly to the motors.

DC Motors (2): These motors drive the wheels of the bot, enabling movement.

Wheels and Chassis: The structure of the bot, including wheels for movement and a chassis to hold all components.

Buck Converter: Converts the battery voltage down to 5V for the Arduino and sensors.

Boost Converter: Steps up the voltage to 12V to power the motors efficiently.

Battery: Powers the entire bot, typically a 7.4V Li-ion battery pack.

LEDs and Push Buttons: Used for feedback and interaction during calibration and operation.


# Miscellaneous

Wires: For connections between components.

Resistors: For current limiting, especially for LEDs.

Capacitors: To smooth out voltage fluctuations, if needed.


# 3. Circuit Diagram

The detailed circuit diagram for this bot is provided in a separate file named Circuit-Diagram. This diagram illustrates how all the components are interconnected. It is crucial to follow this diagram accurately to ensure the bot functions correctly.

# 4. Wiring and Connections

# Arduino Nano

5V Pin: Connects to the 5V output from the buck converter.

GND Pin: Connects to the ground of the power supply.

Digital Output Pins (2-7): Used to control the motor driver inputs (AIN1, AIN2, BIN1, BIN2) and enable pins (PWMA, PWMB).

Analog Pins (A0-A5): Connected to the QTR sensor array for reading sensor values.


# Motor Driver (L298N or similar)

Input Pins (IN1, IN2, IN3, IN4): Connected to the Arduino digital output pins for controlling motor directions.

Enable Pins (ENA, ENB): Connected to PWM pins of the Arduino for controlling motor speed.

Motor Output Pins: Connected to the DC motors.


# QTR Sensors

Power Pins: Connect to the 5V and GND from the Arduino.

Sensor Pins: Connect to the analog input pins (A0-A5) of the Arduino to read sensor values.


# Buck and Boost Converters

Buck Converter: Input from the battery, output set to 5V for powering the Arduino and sensors.

Boost Converter: Input from the battery, output set to 12V for powering the motors through the motor driver.


# Push Buttons and LEDs

Calibration Button: Connected to a digital input pin with a pull-up resistor for starting sensor calibration.

Start Button: Connected to another digital input pin to start the bot.

LEDs: Used for indicating the calibration and operation status, connected with current-limiting resistors.


# 5. Software Setup

Installing the Arduino IDE

Before uploading the code to the Arduino Nano, you'll need the Arduino IDE. Download and install it from the official Arduino website.

Libraries

Ensure you have the QTRSensors library installed. You can do this through the Arduino Library Manager by searching for "QTRSensors."

Uploading the Code

Open the Arduino IDE, copy the provided code into a new sketch,
and upload it to the Arduino Nano. Ensure the correct board and COM port are selected under the "Tools" menu.

# 6. Code Explanation

# Initialization

The code initializes the pins for the motors, sensors, and buttons.
It also sets up the serial communication for debugging purposes.

# Calibration

When the calibration button is pressed, the bot enters a calibration mode where it rotates in place to calibrate the QTR sensors to the environment.

# PID Control

The PID_Control() function calculates the error between the desired line position and the actual position. It then adjusts the motor speeds to correct the bot's path using the PID algorithm.

# Loop Function

The loop() function continuously calls PID_Control() to keep the bot on the line. It checks the sensor readings and adjusts motor speeds accordingly.

# 7. Testing and Calibration

1. Initial Test: After uploading the code, place the bot on a simple line and observe its behavior.


2. Calibration: Press the calibration button and allow the bot to calibrate its sensors by rotating in place.


3. Fine-Tuning: Adjust the PID constants (Kp, Ki, Kd) in the code for optimal performance based on how the bot reacts to the line.



# 8. Troubleshooting

Bot doesn't move: Check all power connections and ensure the motor driver is correctly wired.

Bot veers off the line: Fine-tune the PID constants and ensure the sensors are correctly placed and functioning.

Uneven speed: Ensure both motors are identical and the offsets in the code are adjusted correctly.


# 9. Conclusion

Building a line follower bot is an excellent way to learn about robotics, control systems, and sensor integration. By following this guide, you should have a fully functional bot capable of following a line accurately. 
Experiment with different line patterns and environments to challenge your bot's capabilities and improve its performance.

