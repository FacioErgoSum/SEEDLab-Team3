#Seed Lab Team 3

from fileinput import filename
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
from numpy.linalg import inv
#For breakinng out, replace with return
import sys

#Constatnts
fileName = "planarCalibrationImage.jpg"
calibrationFileLoad = "camera_calibration_2.yaml"
transformationMatrixFile = "transformation_matrix4.yaml"
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
arucoParams = cv2.aruco.DetectorParameters_create()
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
fontBold = cv2.FONT_HERSHEY_DUPLEX #font for displaying text (below)
markerIDS = [0, 1, 2]
#X, Y, Z
#pointA = np.array([-10, 100, 0.5])
#pointB = np.array([0, 117.5, 14.8])
#pointC = np.array([10, 105, 5.3])
pointA = np.array([-12, 12, 0]) #0
pointB = np.array([0, 5.75, 0])  #1
pointC = np.array([12, 24, 0])  #2
Iterations = 20

#Initialize and warmup the camera
camera = PiCamera()
rawCapture = PiRGBArray(camera)
camera.resolution = (1296, 976)
time.sleep(0.1)

#Open the calibration file
cv_file_load = cv2.FileStorage(calibrationFileLoad, cv2.FILE_STORAGE_READ)
mtx_load = cv_file_load.getNode("camera_matrix").mat()
dist_load = cv_file_load.getNode("dist_coeff").mat()
newcameramtx_load = cv_file_load.getNode("new_camera_matrix").mat()
cv_file_load.release() 


#Numpy matrix to hold temp point data
Ball = np.array([[0,0,0],[0,0,0],[0,0,0]])

#Iteration varaiable
i = 0
while (i < Iterations): 
    #Take a photo
    print("Info: Capturing image for camera calibration, " + str(i))
    try:
        rawCapture = PiRGBArray(camera)
        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array
        print("Info: Image Captured")

        #Save the image
        print("Info: Saving image "+fileName)
        cv2.imwrite(fileName, image)
        print("Info: Image Saved")


        #Modify the captured image
        grayOff = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.undistort(grayOff, mtx_load, dist_load, None, newcameramtx_load) 
    
        #Perform Aruco detection
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)   
    
        #Check detection for needed markers
        if ((markerIDS[0] in ids) and (markerIDS[1] in ids) and (markerIDS[2] in ids)):
            print("Info: Captured Image contains needed markers, ")
            
            #Calcualte the 3D positon of the markers
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx_load, dist_load)
            (rvec-tvec).any()   
    
            #Calcualte index
            indexA = np.where(ids == markerIDS[0])[0][0]
            indexB = np.where(ids == markerIDS[1])[0][0]
            indexC = np.where(ids == markerIDS[2])[0][0]
            #print(str(indexA) + " " + str(indexB)+ " " + str(indexC))
    
            #Get the positions
            posA = tvec[indexA][0]
            posB = tvec[indexB][0]
            posC = tvec[indexC][0]
            #print(str(posA) + " " + str(posB)+ " " + str(posC))
            Btemp = np.matrix([posA, posB, posC])
            print("Appending: " + str(Btemp))
            Ball = Ball + Btemp
            print(Ball)
        #The captured image did not contain the required ArUco markers
        else:
            print("Error: Captured image did not contain the necessary ArUco markers for calibration, " + str(i))
            i = i - 1
    except:
        print("Error: FAILED #" + str(i))
        i = i - 1
    i = i + 1
    time.sleep(0.5)


## Perform the Linear Transform Calculation ##
#Generate the B & C matrix
print(Ball)
B = Ball/Iterations
print(B)
np.average(Ball, axis=0)
print(Ball)
C = np.matrix([pointA, pointB, pointC])
#Find inverse of B
Binverse = inv(B)
#A = Binverse * C
A = np.matmul(Binverse, C)
print("Info: Transformation Matrix Solved")

#save as a calibration file
cv_file=cv2.FileStorage(transformationMatrixFile, cv2.FILE_STORAGE_WRITE)
cv_file.write("transformation_matrix", A)
cv_file.release()
print("Info: Matrix Saved?")

#https://numpy.org/doc/stable/reference/generated/numpy.linalg.inv.html

