#   Eric Sobczak
#   9/12/2022
#   SEED Lab Python Assignment 2, Excercise 6

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import math 
from smbus2 import SMBus
import sys
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


#Load calibration file
calibrationFileLoad = "camera_calibration.yaml"
cv_file_load = cv2.FileStorage(calibrationFileLoad, cv2.FILE_STORAGE_READ)
mtx_load = cv_file_load.getNode("camera_matrix").mat()
dist_load = cv_file_load.getNode("dist_coeff").mat()
newcameramtx_load = cv_file_load.getNode("new_camera_matrix").mat()
cv_file_load.release()

#Constants for ArUco detection
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()

font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
fontBold = cv2.FONT_HERSHEY_DUPLEX #font for displaying text (below)

#Set camera parameters
camera = PiCamera()
camera.resolution = (1296, 976)
#camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(1296, 976))

# allow the camera to warmup
print("Starting PiCamera.....")
time.sleep(0.1)

#For Miniproject
quadrant = 0
quadrantPrev = 0
quadrantNew = False

centerX = 1296/2
centerY = 976/2

#LCD Setup
lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [0, 127, 255]
smile = chr(0b10111100)
lcd.message = " " + smile + "    ArUco   " + smile + " \n     Testing     "
time.sleep(2)
lcd.clear()
redColorIter = 0
blueColorIter = 127
greenColorIter = 255

#I2C Setup
bus = SMBus(1)
addr = 0x8
return_data = 0

#The loop starts here
for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    
    #Get image and convert
    image = frame.array
    grayOff = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.undistort(grayOff, mtx_load, dist_load, None, newcameramtx_load)
    #flip the image
    #gray = cv2.flip(gray, 1)

    #Run ArUco detection
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    #Check if any marker was found  
    if len(corners) > 0:
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx_load, dist_load)
        (rvec-tvec).any() 
        
        #Look at only the first marker
        index = 0
        
        #Grab corner data
        cornerInfo = corners[index]
            
        #Calculate center of marker
        cX = int((cornerInfo[0][0][0] + cornerInfo[0][2][0]) / 2.0)
        cY = int((cornerInfo[0][0][1] + cornerInfo[0][2][1]) / 2.0)
            
        ###### DRAW QUADRANT DATA #####
        if (cX < centerX) & (cY < centerY):
            quadrant = 2
        elif (cX > centerX) & (cY < centerY):
            quadrant = 1
        elif (cX < centerX) & (cY > centerY):
            quadrant = 3
        else:
            quadrant = 4
            
        if (quadrant == quadrantPrev):
            quadrantNew = 0
        else:
            quadrantNew = 1
        quadrantPrev = quadrant
        
        
        cv2.putText(gray, str(quadrant), (cX+4,cY-4), font, 1, (255,255,255),1,cv2.LINE_AA)
            
        for i in range(rvec.shape[0]):
            cv2.drawFrameAxes(gray, mtx_load, dist_load, rvec[i, :, :], tvec[i, :, :], 0.03)
            #cv2.drawDetectedMarkers(gray, corners)

    #Draw functions to make output look pretty
    #Draw lines:
    cv2.line(gray, (648,0), (648,976), (0, 255, 0), 3) 
    cv2.line(gray, (0,488), (1296,488), (0, 255, 0), 3) 
    #Draw Numbers
    cxb = 640
    cyb = 500
    hob = 40
    wob = 30
    cv2.putText(gray, "1", (cxb+wob,cyb-hob), font, 1, (255,255,255),1,cv2.LINE_AA)
    cv2.putText(gray, "2", (cxb-wob,cyb-hob), font, 1, (255,255,255),1,cv2.LINE_AA)
    cv2.putText(gray, "3", (cxb-wob,cyb+hob), font, 1, (255,255,255),1,cv2.LINE_AA)
    cv2.putText(gray, "4", (cxb+wob,cyb+hob), font, 1, (255,255,255),1,cv2.LINE_AA)
    
    #Draw Values
    cx = 640
    cy = 500
    ho = 240
    wo = 230
    cv2.putText(gray, "0", (cx+wo,cy-ho), fontBold, 2, (255,255,255),1,cv2.LINE_AA)
    cv2.putText(gray, "pi/2", (cx-wo,cy-ho), fontBold, 2, (255,255,255),1,cv2.LINE_AA)
    cv2.putText(gray, "pi", (cx-wo,cy+ho), fontBold, 2, (255,255,255),1,cv2.LINE_AA)
    cv2.putText(gray, "3pi/2", (cx+wo,cy+ho), fontBold, 2, (255,255,255),1,cv2.LINE_AA)

    cv2.imshow("ArUco 3D Detection", gray)

    #Capture any key press
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `d' key was pressed, break from the loop
    if key == ord("d"):
        cv2.destroyAllWindows()
        break

    #Geoff, Place you serial code here, this point will run as a loop
    #The vraibale you want is quadrant, and quadrantNew
    #quadrant will give a value between 1 and 4 for the quadrant
    #quadrantNew will be true for one loop cycle if there is a new quadrant
    
    if (quadrantNew):
        #Code here if you want the single exectuation
        try:
            pi = chr(0b11110111)
            bus.write_byte_data(addr, 0, quadrant)
            if(quadrant == 1):
                pos = "0"
                lcd.color = [255, 0, 0]
            elif (quadrant == 2):
                pos = pi + "/2"
                lcd.color = [255, 255, 0]
            elif (quadrant == 3):
                pos = pi
                lcd.color = [0, 255, 0]
            elif (quadrant == 4):
                pos = "3" + pi + "/2"
                lcd.color = [0, 127, 255]
            else:
                lcd.message = "Sent: " + str(quadrant) + "\nPosition: " + pos + "      "
        except:
            print("I2C no longer go brr")
        quadrantNew = 0
        #lcd.color = [redColorIter, greenColorIter, blueColorIter]
        lcd.message = "Sent: " + str(quadrant) + "\nPosition: " + pos + "      "
    #Ill clean my code up a little so please excuse the mess
    #No worries, its not that bad :)


#https://www.geeksforgeeks.org/python-opencv-cv2-line-method/
