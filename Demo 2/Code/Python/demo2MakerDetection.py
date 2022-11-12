# SEED Lab - Team 3
# Demo 2 Driving Script - Raspberry Pi
# Fall 2022

# Goal: Calculate the World Coordanites of an ArUco marker, and then send 
# this data to an Arduio over I2C. Also send a mode varaible that indicates 
# what operation the Arduino should perform. The marker postion is calculated 
# using OpenCV, Matrix Math, and Trig.

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import math 
import struct
from smbus2 import SMBus, i2c_msg
import sys
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

print("Welcome to Team 3, Demo 2, ArUco Detection and Localization Script")

#Going to this marker
markerID = 1
runI2C = True

#Load calibration file
calibrationFileLoad = "camera_calibration_2.yaml"
cv_file_load = cv2.FileStorage(calibrationFileLoad, cv2.FILE_STORAGE_READ)
mtx_load = cv_file_load.getNode("camera_matrix").mat()
dist_load = cv_file_load.getNode("dist_coeff").mat()
newcameramtx_load = cv_file_load.getNode("new_camera_matrix").mat()
cv_file_load.release()

#Load transformation file
matrixFileLoad = "transformation_matrix4.yaml"
cv_file_load = cv2.FileStorage(matrixFileLoad, cv2.FILE_STORAGE_READ)
transformation = cv_file_load.getNode("transformation_matrix").mat()
cv_file_load.release()

#Constants for ArUco detection
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
#arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

#Set camera parameters
camera = PiCamera()
camera.resolution = (1296, 976)
rawCapture = PiRGBArray(camera, size=(1296, 976))

#Camera settings
#camera.exposure_mode = 'sports'
camera.shutter_speed = 5000
camera.image_denoise = False
camera.iso = 0
camera.brightness = 58
camera.contrast = 10

#I2C Varaibles
#Robot Position Variables - From Arduino
robot_X = 0
robot_Y = 0
robotAngle = 0
#Marker Location and State - To Arduino
targetPointX = 0
targetPointY = 0
mode = 0

#Setup for FSM
targetMarkerVisible = False
wasFound = False
keyPressed = False
modePrevious = 0

# allow the camera to warmup
print("Starting PiCamera.....")
time.sleep(0.1)

#LCD Setup
if runI2C:
    lcd_columns = 16
    lcd_rows = 2
    i2c = busio.I2C(board.SCL, board.SDA)
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.color = [255, 0, 0]
    smile = chr(0b10111100)
    lcd.message = " " + smile + "    Demo 2   " + smile + " \n     Testing     "
    time.sleep(2)
    lcd.clear()
    redColorIter = 0
    blueColorIter = 127
    greenColorIter = 255

#I2C Setup
if runI2C:
    bus = SMBus(1)
    addr = 0x8
    return_data = 0

    def writeData(value):
        bus.write_i2c_block_data(addr, 0, value)
        return -1

    def readData():
        number = bus.read_i2c_block_data(addr, 0, 12)
        return number
        

for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    ############################################################################
    ########                    I2C REQUEST LOGIC                      #########
    ############################################################################
    if runI2C:
        try:
            #Read the floats
            receivedData = readData()
        except:
            print("I2C Read failed")
        robot_X_Bytes = receivedData[:4]
        robot_Y_Bytes = receivedData[4:8]
        robotAngle_Bytes = receivedData[-4:]
        aa = bytearray(robot_X_Bytes)
        ab = bytearray(robot_Y_Bytes)
        ac = bytearray(robotAngle_Bytes)
        robot_X_tuple = struct.unpack('<f', aa)
        robot_Y_tuple = struct.unpack('<f', ab)
        robotAngle_tuple = struct.unpack('<f', ac)
        robot_X = round(robot_X_tuple[0], 4)
        robot_Y = round(robot_Y_tuple[0], 4)
        robotAngle = round(robotAngle_tuple[0], 4)
        #print("Received I2C Data")
        #print(robot_X)
        #print(robot_Y)
        #print(robotAngle)
    
    ############################################################################
    ########                 CAPTURE AND ARUCO LOGIC                   #########
    ############################################################################
    #Get image and convert
    image = frame.array
    grayOff = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.undistort(grayOff, mtx_load, dist_load, None, newcameramtx_load)
    #Run ArUco detection
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    #First check if any marker was detected
    if len(corners) > 0:
        #Check if the marker we want was found
        if (markerID in ids) :
            #Tell the FSM we have the marker we want
            targetMarkerVisible = True
            #Find what the index of the needed marker is
            indexMarker = np.where(ids == markerID)[0][0]
            #Calculate the pose of all markers
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx_load, dist_load)
            (rvec-tvec).any() 
            
            #Run calculations on the needed marker
            cornerInfo = corners[indexMarker]
            #Calculate center of marker
            cX = int((cornerInfo[0][0][0] + cornerInfo[0][2][0]) / 2.0)
            cY = int((cornerInfo[0][0][1] + cornerInfo[0][2][1]) / 2.0)
            
            ###### CALCULATE RELATIVE POSTION OF MARKER #####
            point = tvec[indexMarker][0]
            fix = np.matmul(point ,transformation)
            ###### CALCULATE WORLD POSTION OF MARKER #####
            partialXA = math.cos(robotAngle) * fix[1] #Normal are both marker Y
            partialXB = math.cos(robotAngle - 1.5708) * fix[0] #Weird subtract are both marker X
            partialYA = math.sin(robotAngle) * fix[1]
            partialYB = math.sin(robotAngle - 1.5708) * fix[0]
            targetPointX = round((partialXA - partialXB + robot_X), 3)
            targetPointY = round((partialYA - partialYB + robot_Y), 3)
            
            #Debug Print
            #print("I am at X: %7.2f  Y: %7.2f  Phi: %7.2f , Marker at X: %7.2f  Y:%7.2f " % (robot_X, robot_Y, robotAngle, targetPointX, targetPointY))

            #Display all the found markers
            for i in range(rvec.shape[0]):
                cv2.drawFrameAxes(gray, mtx_load, dist_load, rvec[i, :, :], tvec[i, :, :], 0.03)
                #cv2.drawDetectedMarkers(gray, corners)
    else:
        targetMarkerVisible = False

    #Show the frame
    cv2.imshow("ArUco 3D Detection", gray)
    #Capture any key press
    key = cv2.waitKey(1) & 0xFF

    
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `d' key was pressed, break from the loop
    if key == ord("d"):
        cv2.destroyAllWindows()
        break
        
    if key == ord("s"):
        keyPressed = True


    ############################################################################
    ########                   FINITE STATE MACHINE                    #########
    ############################################################################
        #Need better approch, its fine for this this demo
    if targetMarkerVisible:
        wasFound = True
    if keyPressed:
        if wasFound:
            mode = 2
            #Found (Green)
        else:
            mode = 1
            #Turning (Blue)
    else:
        mode = 0
        #LCD is Red

    #Change colors on the LCD
    if(modePrevious != mode):
        modePrevious = mode
        if(mode == 0):
            lcd.color = [255, 0, 0]
        elif(mode == 1):
            lcd.color = [0, 0, 255]
        elif(mode == 2):
            lcd.color = [0, 255, 0]
        else:
            print("Could not set LCD Color")


    ############################################################################
    ########                      I2C SEND LOGIC                       #########
    ############################################################################
    if runI2C:
        #print("Sent over I2C, Mode = " + str(mode) + ", X = " + str(targetPointX) + ", Y = " + str(targetPointY))
        #Cast values from 64bit to 32 (for arduino float compatibility)
        targetPointX_32 = np.float32(targetPointX)
        targetPointY_32 = np.float32(targetPointY)
        
        #Convert to byte array
        posBytes = bytearray(struct.pack("f", targetPointX_32))
        posYBytes = bytearray(struct.pack("f", targetPointY_32))
        modeByte = mode.to_bytes(1, 'big')
        
        #Combine bytes
        posBytes.extend(posYBytes)
        posBytes.extend(modeByte)
        
        #Debug prints statements
        #print(posBytes)
        #print(targetPointX)
        #print(targetPointY)
        #print(mode)

            #Send the floats
        try:
            #Read the floats
            sentData = writeData(posBytes)
        except:
            print("I2C Failed")
        #time.sleep(0.1)
        
        ############################################################################
        ########                       VISUALIZATION                       #########
        ############################################################################
            
        # ------------------ Geoff's Makeshift Minimap --------------------- #
        try:
            screenSizeX = 23
            screenSizeY = 40
            inchesPerPixel = 6
            screenSizeMiddleX = round(screenSizeX/2)
            screenSizeMiddleY = round(screenSizeY/2)
            screenSizeInches = screenSizeX*inchesPerPixel
            
            #Convert Origin
            #Formula  (1/inchesPerPixel)*(relativeWorldPostion)+screenSizeMiddleX
            originX = round((1/inchesPerPixel)*(robot_X*1)+screenSizeMiddleX)
            originY = round((-1/inchesPerPixel)*(robot_Y*-1)+screenSizeMiddleY)
            targetX = round((1/inchesPerPixel)*((robot_X-targetPointX)*1)+screenSizeMiddleX)
            targetY = round((-1/inchesPerPixel)*((robot_Y+targetPointY)*-1)+screenSizeMiddleY)
            
            #Rotation
            #Determine where and what arrow to print using robotAngle
            #↖ ⬆ ↗
            #⬅ · ➡
            #↙ ⬇ ↘
            robotAngleDraw = round((robotAngle+3.1415)*1.273)
            printAngle = "'"
            if (robotAngleDraw == 0):
                rotationX = screenSizeMiddleX+1
                rotationY = screenSizeMiddleY
                printAngle = "⬇"
            elif (robotAngleDraw == 1):
                rotationX = screenSizeMiddleX+1
                rotationY = screenSizeMiddleY-1
                printAngle = "↙"
            elif (robotAngleDraw == 2):
                rotationX = screenSizeMiddleX
                rotationY = screenSizeMiddleY-1
                printAngle = "⬅"
            elif (robotAngleDraw == 3):
                rotationX = screenSizeMiddleX-1
                rotationY = screenSizeMiddleY-1
                printAngle = "↖"
            elif (robotAngleDraw == 4):
                rotationX = screenSizeMiddleX-1
                rotationY = screenSizeMiddleY
                printAngle = "⬆"
            elif (robotAngleDraw == 5):
                rotationX = screenSizeMiddleX-1
                rotationY = screenSizeMiddleY+1
                printAngle = "↗"
            elif (robotAngleDraw == 6):
                rotationX = screenSizeMiddleX
                rotationY = screenSizeMiddleY+1
                printAngle = "➡"
            elif (robotAngleDraw == 7):
                rotationX = screenSizeMiddleX+1
                rotationY = screenSizeMiddleY+1
                printAngle = "↘"
            elif (robotAngleDraw == 8):
                rotationX = screenSizeMiddleX+1
                rotationY = screenSizeMiddleY
                printAngle = "⬇"
            else:
                rotationX = 1
                rotationY = 1
            
            #Print Logic    
            for x in range(screenSizeX):
                for y in range(screenSizeY):
                    #Corner
                    if ((x == 0 and y == 0) or (x == (screenSizeX-1) and y == 0) or (x == 0 and y == (screenSizeY-1)) or (x == (screenSizeX-1) and y == (screenSizeY-1))):
                        print("#",end="")
                    #edge top/bottom
                    elif ((x == 0) or (x == (screenSizeX-1))):
                        print("-",end="")
                    #edge left/right
                    elif ((y == 0) or (y == (screenSizeY-1))):
                        print("|",end="")
                    #Robot
                    elif(x == screenSizeMiddleX and y == screenSizeMiddleY):
                        print("R",end="")
                    #Rotation Marker
                    elif(x == rotationX and y == rotationY):
                        print(printAngle,end="")
                    #Marker
                    elif(x == targetX and y == targetY):
                        print("X",end="")
                    #Origin
                    elif(x == originX and y == originY):
                        print("⊙",end="")
                    #Screen
                    else:
                        print("`",end="")
                print("")
        except:
            pass
            
            
    
