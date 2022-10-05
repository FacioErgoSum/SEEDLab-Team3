#   Eric Sobczak
#   9/6/2022
#   SEED Lab Python Assignment 2, Excercise 1-5

#The following code run through excercsies 1 through 5. 
# Select the excercise to run by entering a value on the
# keyboard and hitting enter. All these excercseis use 
# Open CV to process the image, and the PICamera libraries 
# to capture the image. Open CV is also used as File 
# mangament, keyboard capture, and image preview.

from calendar import c
from fileinput import filename
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2

#Constants for ArUco detection
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()

######### EXCERCISE 1 #########
def cv_excercise1():
    #Have user enter a filename
    fileName = input("Enter a Filename: ") + ".jpg"

    #Initialize and warmup the camera
    camera = PiCamera()
    rawCapture = PiRGBArray(camera)
    time.sleep(0.1)

    #Configure camera resolutiona and white balance
    camera.resolution = (1280, 720)
    camera.awb_mode = 'off'
    camera.awb_gains = (1.3, 2)

    #Take a photo
    print("Capturing an Image")
    try:
        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array
    except:
        print("Failed to Capture")

    #Save the image
    print("Saving image "+fileName)
    try:
        cv2.imwrite(fileName, image)
    except:
        print("Couldn't save "+fileName)

    #Display the image
    cv2.imshow("Image Capture", image)
    cv2.waitKey(0)

######### EXCERCISE 2 #########
def cv_excercise2(chosenFile):
    #Open the user selected file
    try:
        image = cv2.imread(chosenFile, cv2.IMREAD_COLOR)
    except:
        print("Could not open file")
        
    #Get dimensions of image
    dimensions = image.shape
    
    #Calculate half dimensions
    dimensionsHalf = (round(dimensions[1]/2),round(dimensions[0]/2))
    
    #Perform the resize
    imageHalfed = cv2.resize(image, dimensionsHalf, interpolation = cv2.INTER_AREA)
    
    #Return the resized image
    return imageHalfed

######### EXCERCISE 3 #########
def cv_excercise3(givenImage):  
     convertedImage = cv2.cvtColor(givenImage, cv2.COLOR_BGR2GRAY)
     return convertedImage
    
######### EXCERCISE 4 & 5 #########
def cv_excercise45(givenImage, fileName):
    #Run ArUco Detection  
    (corners, ids, rejected) = cv2.aruco.detectMarkers(cv2.cvtColor(givenImage, cv2.COLOR_BGR2GRAY), arucoDict, parameters=arucoParams)
        
    #Check if any marker was found
    if len(corners) > 0:
        #Start print statement
        print("Markers with ids", end=" ")
        
        #Loop through all the markers
        for index, cornerInfo in enumerate(corners):
            #Print corner id
            #print(ids[index][0], end=", ")
            
            #Calculate center of marker
            cX = int((cornerInfo[0][0][0] + cornerInfo[0][2][0]) / 2.0)
            cY = int((cornerInfo[0][0][1] + cornerInfo[0][2][1]) / 2.0)
            
            #Add dot to center of marker image
            cv2.circle(givenImage, (cX, cY), 4, (0, 0, 255), -1)
            
            #Write the ID of the marker on the image
            cv2.putText(givenImage, str(ids[index][0]), (cX+10, cY-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
        #End Print Statement
        #print("\b\b were detected in " + fileName + "\n")
        print("Corners:")
        print (corners)
        print ("Ids:")
        print (ids) 
        print ("--")   
        
    else:
        #No DICT_6X6_250 markers were detected in the image
        #print("No markers detected in " + fileName + "\n")
        
        #Write on image
        cv2.putText(givenImage, "NO MARKERS FOUND", (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        #cv2.putText(givenImage, "NO MARKERS DETECTED", (3, 28), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
    #Return image
    return givenImage

    
#Initialization
runMe = input("Which excercise (1-5) should I run?  => ")

#Excercise 1
if ("1"==runMe):
    print("Open CV Excercise 1 | Capture image to file")
    cv_excercise1()


#Excercise 2
if ("2"==runMe):
    print("Open CV Excercise 2 | Scale a file")
    #Get a file name from user
    scaleInput = input("Choose a file to scale: ")
    #Scale and show the image
    cv2.imshow("Scaled Image", cv_excercise2(scaleInput))
    cv2.waitKey(0)
    
    
#Excercise 3
if ("3"==runMe):
    print("Open CV Excercise 3 | Change Color Space")
    #Get a file to work with
    fileToGrey = input("Choose a file to covert to greyscale: ")
    #Load that file
    try:
        imageToGrey = cv2.imread(fileToGrey, cv2.IMREAD_COLOR)
    except:
        print("Could not open file")
    #Send image to function and process it, then show it
    cv2.imshow("Scaled Image", cv_excercise3(imageToGrey))
    cv2.waitKey(0)
    
    
#Excercise 4
if ("4"==runMe):
    print("Open CV Excercise 4 | Aruco within still image")
    
    #Files to loop through
    allFiles = ["worm.jpg", "image2.jpg", "image3.jpg"]
    #Loop through each file
    for markerFile in allFiles:
        #Open file
        try:
            detectInMe = cv2.imread(markerFile, cv2.IMREAD_COLOR)
        except:
            print("Could not open file")
            
        #Perform ArUco Detection and show the image
        cv2.imshow(markerFile, cv_excercise45(detectInMe, markerFile))
        cv2.waitKey(0)
        

#Excercise 5
if ("5"==runMe):
    print("Open CV Excercise 5 | Aruco within video")
    #Setup Camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    
    # allow the camera to warmup
    time.sleep(0.1)
    
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
        imageVideo = frame.array
        
        #Perform the ArUco detection and display the video        
        cv2.imshow("Frame", cv_excercise45(imageVideo, "Video"))
        key = cv2.waitKey(1) & 0xFF
        
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break



#Excercise 1
#   Getting Started with Picamera
#       https://pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/
#   Configure White Balance
#       https://raspberrypi.stackexchange.com/questions/22975/custom-white-balancing-with-picamera
#   take_picture.py example file on Canvas
#   Cool info on camera config for future
#       https://python.hotexamples.com/examples/picamera/PiCamera/awb_gains/python-picamera-awb_gains-method-examples.html
#
#Excercise 2
#   Opening an Image file
#       https://www.geeksforgeeks.org/reading-image-opencv-using-python/
#   Using the resize function
#       https://www.tutorialkart.com/opencv/python/opencv-python-resize-image/
#   Getting image parameters
#       https://www.tutorialkart.com/opencv/python/opencv-python-get-image-size/
#
#Excercise 3
#   How to use the cv2.cvtcolor
#       https://www.geeksforgeeks.org/python-opencv-cv2-cvtcolor-method/
#
#Excercise 4
#   Great tutorial on ArUco detection
#       https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
#   Gave me a stronger understanding of how ArUco markers have types
#       https://pyimagesearch.com/2020/12/28/determining-aruco-marker-type-with-opencv-and-python/
#
# No additional resources used for excercsie 5
