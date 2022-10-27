
# Demo 1

The purpose of Demo one was to both demonstrate both a movment scheme and an ArUco localiztion scheme. Our code is both able to drive to a point set using Polar coordanites, and to find the polar coordanites of an ArUco marker infront of the robot


## Arduino Controller Documentation 
Found within Code>Arduino>Demo1Driver

This section provides operational information on the implementation of our robot controller. The system utilizes two PI loops to provide optimum control of the robot. The first bit of the program just defines the necessary criteria to run the controller. The actual control comes from the loop () function.  

The first condition in the loop is examining how close the robot is to the desired target; if the robot is within an expectable range, the PI loops are turned off and the motors are killed. However, if the robot is not at its target, there are two states: rotating or moving. Each of these states are controlled by independent PI controllers. The values of which may be found in Table 1. 

|  PID TABLE    | KP            | KI            | KD            |
| ------------- | ------------- | ------------- | ------------- |
| Rotation PI   | 0.8           | 0.000014      | 0.0           |
| Moving PI     | 0.013         | 0.000013      | 0.0           |

Table 1. PI Controller Gain Values 

Regardless of which controller is actively controlling the robot, every 10 milliseconds the system is sampled, and the error is calculated.  Positional and angular error may be found with the following two formulas:  
## Python Vision Documentation 
Found within Code>Python

The goal of our Vision system was to correctly determin the angle and distance away from our robot. Our existing miniProject vision system was already able to provide the corrdanites of markers relative to the robot. Using a linear transform, we are able to convert the coordanites relative to the robot to a coordanite plane flat flat to the ground. We are able to compute the needed transform using a set of calibration markers. The world and realtive coordanites of these calibration marker are known, so the program within matrixCalibrator.py is able to solve for the matrix needed to get between the two. The code in calibratedPolar.py can then use this matrix to solve for the polar coordanites. 