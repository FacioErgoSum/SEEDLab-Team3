#   Eric Sobczak
#   9/6/2022
#   SEED Lab Python Assignment 2+

# Peforming camera calibration is vital to accuratley find the location of 
# ArUco markers in 3D space. This script generates a .yaml calibration file 
# that contains calibration data for any PiCamera. This scripts begins by 
# running a photo application to collect calibration data. Simply aim the 
# camera at a checkerboard and press "c" multiple times from different angles. 
# After atleast 6 images, press the d key. The program will look for the 
# checkboard in each image and note the locationsa of all the corners. 
# Afterwards opencv camera calibration script will run and generate a new 
# calibration. This is save to a yaml file, and then realoaded. A preview of 
# the calibration results is then shown to the user.


from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2

# Set the parameters for finding sub-pixel corners, max 30 cycles, max error tolerance 0.001
subPixelCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#Set Size of Chessboard
gridW = 9   # 10 - 1
gridH = 6   # 7  - 1
checkerSpacing = 18.1 #Size of checkerboard points in mm
#Define the checkerboard in world coordanites
checkerPoints = np.zeros((gridW*gridH, 3 ), np.float32)  #Create numpy array full of zeros for all checkerboard points
checkerPoints[:,: 2 ] = np .mgrid[ 0 :gridW, 0 :gridH].T.reshape( -1 , 2 )  #Fill the grid with coordainted of points 
checkerPoints = checkerPoints * checkerSpacing #Multiply out grid by spacing

#Create holders 3D and 2D Coordanites
worldCordChess = [] # 3D points in the world coordinate system
imgCordChess = []   # 2D points in the image plane


#Capture images for calibration
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
print("Starting PiCamera.....")
time.sleep(0.1)

#Index for taking images
photoIndex = 10  #Increase this to skip taking images if you already have them

print("Press 'c' to take a photo")
print("Press 'd' to finish and calibrate")

#Capture photos for calibration
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    imageVideo = frame.array

    #Display the video        
    cv2.imshow("Frame", imageVideo)
    key = cv2.waitKey(1) & 0xFF
        
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
        
    # if the `d' key was pressed, break from the loop
    if key == ord("d"):
        cv2.destroyAllWindows()
        break
    
    # if the 'c' key was pressed, capture an image
    if key == ord("c"):
        fileName = "calibrationImage" + str(photoIndex) + ".jpg"
        cv2.imwrite(fileName, imageVideo)
        print ("Saving " + fileName)
        photoIndex += 1

#Loop through all the captured photos and perform calibration steps
calibrateIndex = 0
while calibrateIndex < photoIndex:
    #Load from captured photo
    fileName = "calibrationImage" + str(calibrateIndex) + ".jpg"
    img = cv2.imread(fileName)
    
    #Get image parameters
    imgHeight, imgWidth = img.shape[0], img.shape[1]
    
    #Get grayscale image
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
    #Perform scan for checkerbaord
    ret, corners = cv2.findChessboardCorners(gray, (gridW,gridH),None)
    
    if ret == True:
        print("Checkerboard found in " + fileName + ", running subpixel correction")
        #Use subpixel constraints to get more accurate corners
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),subPixelCriteria)
        
        #Add corners to lists from earlier
        worldCordChess.append(checkerPoints) #? - Im still confused why we are doing this as checkerPoints has no data from the image
        imgCordChess.append(corners)
        
        #Draw and display the corners on the chessbaord
        cv2.drawChessboardCorners(img, (gridW,gridH), corners, ret)
        cv2.namedWindow(('CheckerView - ' + fileName), cv2.WINDOW_NORMAL)
        cv2.resizeWindow(('CheckerView - ' + fileName), 640, 480)
        cv2.imshow(('CheckerView - ' + fileName),img)
        cv2.waitKey(2000)
        
    else:
        print("No checkerboard found in " + fileName)
        
    calibrateIndex += 1
cv2.destroyAllWindows()

#Generate the calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(worldCordChess, imgCordChess, gray.shape[::-1], None, None)
#Generate a new camera matrix to account for a different frame size with the distortion 
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (imgWidth, imgHeight), 1, (imgWidth, imgHeight))

#Save calibration to file
calibrationFile = "calibration_test1.yaml"
cv_file=cv2.FileStorage(calibrationFile, cv2.FILE_STORAGE_WRITE)
cv_file.write("camera_matrix", mtx)
cv_file.write("dist_coeff", dist)
cv_file.write("new_camera_matrix", newcameramtx)
cv_file.release()

#Load calibration file
calibrationFileLoad = "calibration_test1.yaml"
cv_file_load = cv2.FileStorage(calibrationFileLoad, cv2.FILE_STORAGE_READ)
mtx_load = cv_file_load.getNode("camera_matrix").mat()
dist_load = cv_file_load.getNode("dist_coeff").mat()
newcameramtx_load = cv_file_load.getNode("new_camera_matrix").mat()
cv_file_load.release()

#Start video capture again
print("Running preview of distortion correction, press 'd' to exit")
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    imageVideo = frame.array
    
    dst1 = cv2.undistort(imageVideo, mtx_load, dist_load, None, newcameramtx_load)
    
    #Perform the ArUco detection and display the video        
    cv2.imshow("Corrected", dst1)
    cv2.imshow("Uncorrected", imageVideo)
    key = cv2.waitKey(1) & 0xFF
        
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
        
    # if the `d' key was pressed, break from the loop
    if key == ord("d"):
        cv2.destroyAllWindows()
        break

# The following tutorial provide info on how to perform these calibration steps.
# I also utilized the resources from the other documents
#https://stackoverflow.com/questions/39432322/what-does-the-getoptimalnewcameramatrix-do-in-opencv
#https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
