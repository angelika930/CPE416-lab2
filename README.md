# CPE416

Lab 3 Part 1:

Line-Following Robot with PID Control

Description:
This project implements a line-following robot utilizing a PID (Proportional-Integral-Derivative) controller to navigate along predefined paths. The robot continuously reads sensor values and adjusts its motor speeds accordingly to maintain its course.

Features
PID Control: The robot uses a PID controller to minimize the error between the left and right sensors.

Motor Control: Individual motor speed control allows for precise navigation.

Pause Functionality: A button can pause the robot without needing to disconnect the hardware.

Real-time Feedback: The current motor speeds are displayed on an LCD for monitoring.

Lab 3 part 2: 

Robot Controller with Neural Network and Proportional Control

This project implements a robot controller that combines neural network learning with proportional control for navigating environments based on sensor inputs. The controller is designed to be used within the Webots simulation environment, allowing for real-time training and inference.

Features

Neural Network Training: The controller employs a simple feedforward neural network that learns from sensor data to adjust motor speeds dynamically.

Proportional Control: A proportional controller is implemented to maintain balance and follow lines based on readings from ground color sensors.

Real-time Input Handling: The controller can respond to keyboard inputs for manual operation while simultaneously training its neural network.

Modular Design: The code is structured to facilitate easy modifications and extensions, with clear separation of functions for training, control, and sensor normalization.
