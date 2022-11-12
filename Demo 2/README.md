
# Demo 2

The goal for Demo 2 was to locate an Aruco marker, and drive to it once found. The drive and vison code from Demo 1 were combined using I2C logic to create a "search and destroy" style control system.
## System Architecture

A few considerations needed to be made when developing the system architecture for this project. One is that the update rate of the Pi is significantly slower than that of the Arduinos. The Arduino needs to have some persistence as to what the location of where the ArUco marker is. We achieved this by defining everything in a world coordinate space. The Arduino, through localization with the encoders, keeps track as to what its position is in World space. It sends this data to the PI over I2C. Once the PI sees a marker, It uses the relative location of the marker, using code from demo 1, and adds it to the current world location of the robot. This marker world location is then sent back to the Arduino over the same I2C line. The Arduino then computes its angular and positional error to that point and feeds that information into the PI loops that run the motors.
## Demo 2 I2C Breakdown

 

### Overview of Communication

 

Before we discuss what data was sent, I think it is important to discuss the purpose that I2C served in this Demo. To understand this, we will need to cover how the robot localization works. Our robot has two location systems that work in tandem, the Arduino has encoder data and PID control, which provide an accurate physical local distance traveled by the robot. The Raspberry Pi has computer vision which can detect the robot's position based on proximity to markers in three-dimensional space. If we combine these two relative positions, we can create a global position for the robot. This is where our I2C came in to create a communication pathway for sharing this location data. 

 

The I2C packets being sent by each device were as follows:

 

| Raspberry Pi | Size | Datatype |
|--|--|--|
| Target Point X | 4 Bytes | Float   |
| Target Point Y | 4 Bytes | Float   |
| Mode           | 1 Byte  | Integer |

 

|  Arduino | Size | Datatype |
|--|--|--|
| Robot X     | 4 Bytes | Float |
| Robot Y     | 4 Bytes | Float |
| Robot Angle | 4 Bytes | Float |

 

We used both of these data types to calculate Positional and Angular errors, as well as a global position via integration on the Pi. Since we know the location of a marker in relation to the robot (via OpenCV) and we know the location of the robot from the origin (via Encoders), it's only a matter of simple addition to create a current global position for all markers and the robot.

 

### Issues and Resolutions:

 

The most difficult problem to solve for this Demo was sending floats over I2C. Arduino and Python have different default float types (32 bit and 64 bit respectively) which means we needed to not only use a custom float definition (based on Numpy Float32), we also had to send the data over four separate bits and then reconstruct the numbers on either end. This involved a lot of byte arrays and creating custom Structs on the Arduino side. It took some trial and error, but we were able to get good numbers, with an error value of around +/- 0.0005.

 

### Other Thoughts on I2C:

 

Since the localization of the markers were based on updated positions from the Pi-Cam, once the markers got too close to the robot, they would disappear from the sight of the camera. To account for this, we allowed the Arduino to decide if we had reached the marker or not. This caused a small problem, because we were also controlling the Arduino state via the Raspberry Pi, meaning the Arduino would decide when we reached the marker, but the Pi would decide when we entered a stop state. This led to a gridlock in the Finite State Machine, since the Arduino would end it's state and the Raspberry Pi didn't know whether the Arduino thought it had reached the marker or not. In the future, we will be using I2C to send this data as well, and we will use a consensus approach to decide whether the robot has reached it's target or not, and if it can move on to the next marker. 
## Finite State Machine

To allow for both searching, stopping, and driving to the marker, we developed a psuedo FSM in the Raspberry PI Python code. Do to the simplicity of this control loop, it was done with flags and if statments. Essentailly this system boiled down to only a 3 state machine. For the Final Demo we will rewrite this code to use control and status words and revolve around define statments.