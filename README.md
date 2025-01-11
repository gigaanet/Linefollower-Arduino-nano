Line-Following Bot with PID Control Using Arduino nano
Overview
This project is a comprehensive implementation of a line-following robot using a PID (Proportional-Integral-Derivative) control algorithm to achieve smooth and accurate navigation along a predefined line. The robot is designed to use QTR (QTRSensors) sensors to detect the position of the line and SparkFun TB6612FNG motor drivers to control the motors. The PID controller adjusts the motor speeds based on the line's position relative to the robot, ensuring precise tracking and minimal deviation.

Key Components
1. QTR Sensors: A set of eight infrared sensors from Pololu, which are capable of detecting the contrast between the line (usually black) and the surrounding surface (usually white or a lighter color). These sensors provide real-time feedback on the robot's position relative to the line.
2. SparkFun TB6612FNG Motor Driver: This dual-channel H-bridge motor driver allows for independent control of two DC motors. It supports PWM (Pulse Width Modulation) for speed control and can handle both direction and braking functions.
3. Arduino Microcontroller: The brain of the robot, responsible for reading sensor values, executing the PID control algorithm, and sending commands to the motor driver.

Features
PID Control: The robot uses a PID controller to dynamically adjust motor speeds based on the error between the desired position (centered on the line) and the actual position (as detected by the sensors). This approach minimizes oscillations and provides smoother movement.
Sensor Calibration: Before beginning the line-following task, the robot undergoes a calibration process to adapt the sensors to the specific lighting and surface conditions. This ensures that the sensors accurately distinguish between the line and the background.
Button-Controlled Operations: Two buttons are included—one for initiating sensor calibration and the other for starting the line-following task. This allows for easy operation without the need for continuous serial communication.
Adjustable Parameters: Key parameters, such as the PID constants (Kp, Ki, Kd) and motor speed limits, are defined in the code and can be easily adjusted to optimize the robot's performance under different conditions.
Project Setup

Hardware Requirements
Arduino Microcontroller: Any Arduino board that supports the required number of digital and analog I/O pins can be used.
QTR Sensors: Eight QTR sensors connected to the microcontroller for line detection.
SparkFun TB6612FNG Motor Driver: For controlling two DC motors.
DC Motors: Two motors for the robot's movement.
Buttons: Two push buttons for user input (calibration and start).

Miscellaneous: Wires, resistors (if necessary), and a breadboard or PCB for connections.

Hardware Connections
1. QTR Sensors:
Connected to analog pins A0 to A5 and digital pins 10 and 9 on the Arduino.
Optional: Use an additional pin for emitter control if needed.

2. Motor Driver:
PWMA_PIN (6) and PWMB_PIN (5): PWM pins for motor speed control.
AIN1_PIN (7), AIN2_PIN (4), BIN1_PIN (3), and BIN2_PIN (2): Digital pins for motor direction control.
STBY_PIN (12): Standby control pin to enable or disable the motor driver.

3. Buttons:
BUTTON_CALIBRATION_PIN (8): Button for initiating the sensor calibration process.
BUTTON_START_PIN (11): Button for starting the line-following operation.

4. Power Supply: Ensure a suitable power supply is provided for both the Arduino and the motors.
Software Requirements
Arduino IDE: Used to write and upload the code to the Arduino board.
QTRSensors Library: Provides functions for reading and calibrating QTR sensors.
SparkFun_TB6612 Library: Facilitates the control of the TB6612FNG motor driver.

Code Explanation:
Initialization
In the setup() function:
The pin modes for the motors and buttons are set.
Serial communication is initialized for debugging purposes.
The QTR sensors are configured, and a calibration process is initiated by pressing the calibration button. The robot spins in both directions to ensure the sensors capture a wide range of values for accurate calibration.
PID Control Algorithm

The PID_Control() function:
Reads the current position of the line using the QTR sensors.
Calculates the error as the difference between the target position (usually the center of the sensor array) and the actual position.
Uses the PID formula to compute the correction needed:

Proportional: Corrects based on the current error.

Integral: Accounts for past errors to eliminate residual steady-state error.

Derivative: Predicts future errors based on the rate of change of the error.
Adjusts the motor speeds accordingly, ensuring the robot stays on the line.
Motor Speed Control

The mspeed() function:
Takes two arguments (speed values for the left and right motors) and sends these values to the motor driver to adjust the motor speeds.
The motor speeds are constrained within predefined limits to prevent excessive speed or reverse movements beyond the intended range.


Main Loop
The loop() function:
Continuously calls the PID_Control() function to adjust the robot's trajectory in real-time.
Ensures that the robot can adapt to changes in the line's path, such as curves or intersections.

Usage Instructions
1. Hardware Assembly: Assemble the robot as per the hardware connections described.
2. Code Upload: Upload the provided code to the Arduino using the Arduino IDE.
3. Calibration: Place the robot on the line, press the calibration button, and wait for the robot to complete its calibration routine.
4. Line Following: Press the start button to initiate the line-following operation. The robot should now follow the line smoothly, adjusting its speed and direction as necessary.

Tuning the PID Controller
To optimize the robot's performance:
Kp (Proportional Gain): Increase to make the robot respond more aggressively to deviations, but too high a value may cause oscillations.
Ki (Integral Gain): Increase to reduce steady-state error, but too high a value may cause overshooting or oscillations.
Kd (Derivative Gain): Increase to dampen oscillations, but too high a value may slow down the robot's response to changes.

Experiment with these values to achieve the desired behavior for your specific environment and line configuration.

Conclusion
This project provides a robust and adaptable platform for building a line-following robot using PID control. 
With the ability to calibrate the sensors and tune the PID parameters, this robot can be tailored to a wide range of environments and
challenges. It's an excellent project for learning about robotics, control systems, and embedded programming, and it can serve as a foundation for more advanced robotic applications, 
such as maze-solving or autonomous navigation tasks.

