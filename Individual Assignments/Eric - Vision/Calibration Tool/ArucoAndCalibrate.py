import numpy as np
import time
import cv2
import cv2.aruco as aruco
from imutils.video import VideoStream
import argparse
import imutils



# mtx = np.array([
#         [2946.48,       0, 1980.53],
#         [      0, 2945.41, 1129.25],
#         [      0,       0,       1],
#         ])
# #When my mobile phone takes a picture of the chessboard, the picture size is 4000 x 2250
# #The ip camera is set to 1920 x 1080 when shooting video, and the aspect ratio is the same,
# #Pay attention when setting the resolution of ip camera
#
#
# dist = np.array( [0.226317, -1.21478, 0.00170689, -0.000334551, 1.9892] )


#Camera correction parameters

# dist=np.array(([[-0.51328742,  0.33232725 , 0.01683581 ,-0.00078608, -0.1159959]]))
#
# mtx=np.array([[464.73554153, 0.00000000e+00 ,323.989155],
#  [  0.,         476.72971528 ,210.92028],
#  [  0.,           0.,           1.        ]])
dist=np.array(([[ 0.15015287, -1.01745458,  0.00274141,  0.0074918,   1.86666574]]))
newcameramtx=np.array([[1.58943762e+03, 0.00000000e+00, 6.54862606e+02],
 [0.00000000e+00, 1.64224829e+03, 3.50408762e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
mtx=np.array([[1.30947713e+03, 0.00000000e+00, 6.60636668e+02],
 [0.00000000e+00, 1.39791876e+03, 2.99447789e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
 


 
 



print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)

font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

#num = 0
while True:
    frame = vs.read()
    h1, w1 = frame.shape[:2]
    # Read the camera picture
    # Correct distortion
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h1, w1), 0, (h1, w1))
    dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    x, y, w1, h1 = roi
    dst1 = dst1[y:y + h1, x:x + w1]
    frame=dst1


    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    '''
    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
    '''

    #Use aruco The detectmarkers() function can detect the marker and return the ID and the coordinates of the four corners of the sign board
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

#    If you can't find it, type id
    if ids is not None:

        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        # Estimate the attitude of each marker and return the values rvet and tvec --- different
        # from camera coeficcients
        (rvec-tvec).any() # get rid of that nasty numpy value array error

#        aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1) #Draw axis
#        aruco.drawDetectedMarkers(frame, corners) #Draw a square around the mark

        for i in range(rvec.shape[0]):
            aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
            aruco.drawDetectedMarkers(frame, corners)
        ###### DRAW ID #####
        cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


    else:
        ##### DRAW "NO IDS" #####
        cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


    # Display result frame
    cv2.imshow("frame",frame)

    key = cv2.waitKey(1)

    if key == 27:         # Press esc to exit
        print('esc break...')
        cap.release()
        cv2.destroyAllWindows()
        break

    if key == ord(' '):   # Press the spacebar to save
#        num = num + 1
#        filename = "frames_%s.jpg" % num  # Save an image
        filename = str(time.time())[:10] + ".jpg"
        cv2.imwrite(filename, frame)
