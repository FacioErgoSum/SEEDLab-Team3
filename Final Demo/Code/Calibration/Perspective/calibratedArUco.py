#   Eric Sobczak
#   9/12/2022
#   SEED Lab Python Assignment 2, Excercise 6

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import math 


#Load calibration file
calibrationFileLoad = "calibration_test.yaml"
cv_file_load = cv2.FileStorage(calibrationFileLoad, cv2.FILE_STORAGE_READ)
mtx_load = cv_file_load.getNode("camera_matrix").mat()
dist_load = cv_file_load.getNode("dist_coeff").mat()
newcameramtx_load = cv_file_load.getNode("new_camera_matrix").mat()
cv_file_load.release()

#Constants for ArUco detection
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()

font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

#Set camera parameters
camera = PiCamera()
camera.resolution = (640, 480)
#camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
print("Starting PiCamera.....")
time.sleep(0.1)

#For drawing phi
fD = 0

for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    #Get image and convert
    image = frame.array
    grayOff = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.undistort(grayOff, mtx_load, dist_load, None, newcameramtx_load)

    #Run ArUco detection
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    #Check if any marker was found
    if len(corners) > 0:
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx_load, dist_load)
        (rvec-tvec).any() 
        #Loop through all the markers
        for index, cornerInfo in enumerate(corners):
            #Print corner id
            #print(ids[index][0], end=", ")
            
            #Calculate center of marker
            cX = int((cornerInfo[0][0][0] + cornerInfo[0][2][0]) / 2.0)
            cY = int((cornerInfo[0][0][1] + cornerInfo[0][2][1]) / 2.0)
            
            ###### DRAW PHI #####
            phi = math.degrees(math.atan((tvec[index][0][0])/(tvec[index][0][2])))
            phiWrite =str((round(phi*10))/10)
            fD += 1
            #phi = str((tvec[0][0][0])/(tvec[0][0][2]))
            #Hoz tvec[0][0][0]
            #Depth tvec[0][0][2]
            #math.degrees(math.atan(1.18))
            cv2.putText(gray, phiWrite, (cX+4,cY-4), font, 1, (255,255,255),1,cv2.LINE_AA)
            
        for i in range(rvec.shape[0]):
            cv2.drawFrameAxes(gray, mtx_load, dist_load, rvec[i, :, :], tvec[i, :, :], 0.03)
            #cv2.drawDetectedMarkers(gray, corners)


    cv2.imshow("ArUco 3D Detection", gray)

    #Capture any key press
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `d' key was pressed, break from the loop
    if key == ord("d"):
        cv2.destroyAllWindows()
        break


#https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
#https://stackoverflow.com/questions/10057854/inverse-of-tan-in-python-tan-1
