
# Final Demo

The goal of the final demo was to traverse a path set by the location of different Aruco markers. The robot needed to search for markers, and then drive to them in a particular order. To accomlish this we use a combination of Arduino and Raspberry PI control defined below.

## System Architecture

A few considerations needed to be made when developing the system architecture for this project. One is that the update rate of the Pi is significantly slower than that of the Arduinos. The Arduino needs to have some persistence as to what the location of where the ArUco marker is. We achieved this by defining everything in a world coordinate space. The Arduino, through localization with the encoders, keeps track as to what its position is in World space. It sends this data to the PI over I2C. Once the PI sees a marker, It uses the relative location of the marker, using code from demo 1, and adds it to the current world location of the robot. This marker world location is then sent back to the Arduino over the same I2C line. The Arduino then computes its angular and positional error to that point and feeds that information into the PI loops that run the motors. An FSM then controls moving and waiting between Markers

## Overview of I2C Communication

Before we discuss what data was sent, I think it is important to discuss the purpose that I2C served in this Demo. To understand this, we will need to cover how the robot localization works. Our robot has two location systems that work in tandem, the Arduino has encoder data and PID control, which provide an accurate physical local distance traveled by the robot. The Raspberry Pi has computer vision which can detect the robot's position based on proximity to markers in three-dimensional space. If we combine these two relative positions, we can create a global position for the robot. This is where our I2C came in to create a communication pathway for sharing this location data. 

 

The I2C packets being sent by each device were as follows:

 

| Raspberry Pi | Size | Datatype |
|--|--|--|
| Target Point X | 4 Bytes | Float   |
| Target Point Y | 4 Bytes | Float   |
| Mode           | 1 Byte  | Integer |

Table 1.1 I2C Values coming from the PI

|  Arduino | Size | Datatype |
|--|--|--|
| Robot X     | 4 Bytes | Float |
| Robot Y     | 4 Bytes | Float |
| Robot Angle | 4 Bytes | Float |
| Status Word | 4 Bytes | Float |

Table 1.2 I2C Values coming from the Arduino 

We used both of these data types to calculate Positional and Angular errors, as well as a global position via integration on the Pi. Since we know the location of a marker in relation to the robot (via OpenCV) and we know the location of the robot from the origin (via Encoders), it's only a matter of simple addition to create a current global position for all markers and the robot. The status word gets used so that the PI know when it has reached a marker.

## Arduino PID Controller 

This section provides operational information on the implementation of our robot controller. The system utilizes two PI loops to provide optimum control of the robot. The first bit of the program just defines the necessary criteria to run the controller. The actual control comes from the loop () function.  

The first condition in the loop is examining how close the robot is to the desired target; if the robot is within an expectable range, the PI loops are turned off and the motors are killed. However, if the robot is not at its target, there are two states: rotating or moving. Each of these states are controlled by independent PI controllers. The values of which may be found in Table 2. 

|  PID TABLE    | KP            | KI            | KD            |
| ------------- | ------------- | ------------- | ------------- |
| Rotation PI   | 0.8           | 0.000014      | 0.0           |
| Moving PI     | 0.013         | 0.000013      | 0.0           |

Table 2. PI Controller Gain Values 

Regardless of which controller is actively controlling the robot, every 10 milliseconds the system is sampled, and the error is calculated.  Positional and angular error may be found with the following two formulas:

## Python Vision System

The goal of our Vision system was to correctly determin the angle and distance away from our robot. Our existing miniProject vision system was already able to provide the corrdanites of markers relative to the robot. Using a linear transform, we are able to convert the coordanites relative to the robot to a coordanite plane flat flat to the ground. We are able to compute the needed transform using a set of calibration markers. The world and realtive coordanites of these calibration marker are known, so the program within matrixCalibrator.py is able to solve for the matrix needed to get between the two. The code in calibratedPolar.py can then use this matrix to solve for the polar coordanites. 

## Finite State Machine

A Finite State Machine running on the Raspberry PI ulitmentley controls the action of the Robot. The FSM keeps track of the current markers, and which ones have and havent been found. An internal array keeps track of the position of all the markers. The FSM is designed to search for a marker, drive to it, wait 5 seconds, and then repeat. A GUI interface displays status and allows for Go/Stop control.