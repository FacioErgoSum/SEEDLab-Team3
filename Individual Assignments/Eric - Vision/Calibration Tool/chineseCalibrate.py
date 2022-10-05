import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# 找棋盘格角点标定并且写入文件

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) # 阈值
#棋盘格模板规格
w = 9   # 10 - 1
h = 6   # 7  - 1
# 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵
objp = np.zeros((w*h,3), np.float32)
objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
objp = objp*18.1  # 18.1 mm

# 储存棋盘格角点的世界坐标和图像坐标对
objpoints = [] # 在世界坐标系中的三维点
imgpoints = [] # 在图像平面的二维点

images = glob.glob('./CalImages/*.jpg')  #   拍摄的十几张棋盘图片所在目录

i = 1
for fname in images:

    img = cv2.imread(fname)
    # 获取画面中心点

    h1, w1 = img.shape[0], img.shape[1]
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    u, v = img.shape[:2]
    # 找到棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, (w,h),None)
    # 如果找到足够点对，将其存储起来
    if ret == True:
        print("i:", i)
        i = i+1

        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        # 将角点在图像上显示
        cv2.drawChessboardCorners(img, (w,h), corners, ret)
        cv2.namedWindow('findCorners', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('findCorners', 640, 480)
        cv2.imshow('findCorners',img)
        cv2.waitKey(200)
cv2.destroyAllWindows()
#%% 标定
print('正在计算')
ret, mtx, dist, rvecs, tvecs = \
    cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
cv_file=cv2.FileStorage("camera.yaml",cv2.FILE_STORAGE_WRITE)
cv_file.write("camera_matrix",mtx)
cv_file.write("dist_coeff",dist)
# 请注意，*释放*不会关闭（）FileStorage对象

cv_file.release()

print("ret:",ret  )
print("mtx:\n",mtx)      # 内参数矩阵
print("dist畸变值:\n",dist   )   # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
print("rvecs旋转（向量）外参:\n",rvecs)   # 旋转向量  # 外参数
print("tvecs平移（向量）外参:\n",tvecs  )  # 平移向量  # 外参数
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (u, v), 0, (u, v))
print('newcameramtx外参',newcameramtx)
camera=cv2.VideoCapture(0)

# dist=np.array(([[-0.3918239532375715, 0.1553689004591761, 0.001069066277469635, 2.175204930902934e-06, -0.02850420360197434]]))
# # newcameramtx=np.array([[1.85389837e+04 ,0.00000000e+00, 5.48743017e+02]
# #  ,[  0 ,2.01627296e+04 ,4.52759577e+02]
# #  ,[0, 0, 1]])
# mtx=np.array([[379.1368428730273, 0, 312.1210537268028],
#  [  0, 381.6396537294123, 242.492484246843],
#  [  0.,           0.,           1.        ]])



while True:
    (grabbed,frame)=camera.read()
    h1, w1 = frame.shape[:2]
    #打开标定文件
    cv_file = cv2.FileStorage("camera.yaml", cv2.FILE_STORAGE_READ)
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_matrix = cv_file.getNode("dist_coeff").mat()
    cv_file.release()

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_matrix, (u, v), 0, (u, v))
    # 纠正畸变
    dst1 = cv2.undistort(frame, camera_matrix, dist_matrix, None, newcameramtx)
    #dst2 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    mapx,mapy=cv2.initUndistortRectifyMap(camera_matrix,dist_matrix,None,newcameramtx,(w1,h1),5)
    dst2=cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)


    # 裁剪图像，输出纠正畸变以后的图片
    x, y, w1, h1 = roi
    dst1 = dst1[y:y + h1, x:x + w1]


    cv2.imshow('dst1',dst1)
    #cv2.imshow('dst2', dst2)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # 按q保存一张图片
        cv2.imwrite("../u4/frame.jpg", dst1)
        break

camera.release()
cv2.destroyAllWindows()



import cv2 
import numpy as np 
import glob



# Find chessboard corners 
# Set the parameters for finding sub-pixel corners, the stopping criteria used are the maximum number of cycles 30 and the maximum error tolerance 0.001
 criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30 , 0.001 ) # threshold 
# chessboard Grid template specification
 w = 9    # 10 - 1
 h = 6    # 7 - 1 
# Checkerboard points in world coordinates, e.g. (0,0,0), (1,0,0), (2,0,0 ) ....,(8,5,0), remove the Z coordinate and record it as a two-dimensional matrix
objp = np.zeros((w*h, 3 ), np.float32) 
objp[:,: 2 ] = np .mgrid[ 0 :w, 0 :h].T.reshape( -1 , 2 ) 
objp = objp* 18.1   # 18.1 mm

# Store the world coordinate and image coordinate pairs of the corner points of the checkerboard
 objpoints = [] # 3D points in the world coordinate system
 imgpoints = [] # 2D points in the image plane #Load 
all jpg images in the pic folder
 images = glob.glob( './*.jpg' )   # The directory where the dozens of chessboard pictures taken are located

i= 0 
for fname in images:

    img = cv2.imread(fname) # Get the center point of the screen # Get the length and width of the image     h1, w1 = img.shape[ 0 ], img.shape[ 1 ]     gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)     u, v = img.shape[: 2 ] # find the checkerboard corners     ret, corners = cv2.findChessboardCorners(gray, (w,h), None ) # if enough pairs are found, store them if ret == True :         print( "i:" , i)         i = i+ 1 # Find sub-pixel corner points based on the original corner points         cv2.cornerSubPix(gray,corners,( 11 , 11 ),( -1 , -1 ),criteria)
    
    



    

    
    


        

        #Append into world 3D points and plane 2D points
         objpoints.append(objp) 
        imgpoints.append(corners) # Display the corners on the image         cv2.drawChessboardCorners(img, (w,h), corners, ret)         cv2. namedWindow( 'findCorners' , cv2.WINDOW_NORMAL)         cv2.resizeWindow( 'findCorners' , 640 , 480 )         cv2.imshow( 'findCorners' ,img)         cv2.waitKey( 200 ) cv2.destroyAllWindows() #%% Calibration print( ' Calculating' ) #Calibration ret, mtx, dist, rvecs, tvecs = \     cv2.calibrateCamera(objpoints, imgpoints, gray.shape[:: -1
        










], None , None )


print( "ret:" ,ret ) 
print( "mtx:\n" ,mtx)       # Internal parameter matrix
 print( "dist distortion value:\n" ,dist )    # distortion coefficient distortion cofficients = (k_1,k_2,p_1, p_2,k_3)
 print( "rvecs rotation (vector) external parameter:\n" ,rvecs)    # rotation vector # external parameter
 print( "tvecs translation (vector) external parameter:\n" ,tvecs )   # translation vector # external parameter
 newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (u, v), 0 , (u, v)) 
print( 'newcameramtx external parameter' ,newcameramtx) #Open 
camera
 camera=cv2.VideoCapture( 0 ) 
while  True : 
    (grabbed,frame)=camera.read()
    h1, w1 = frame.shape[: 2 ] 
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (u, v), 0 , (u, v)) # correct distortion     dst1 = cv2.undistort(frame, mtx, dist, None , newcameramtx) #dst2 = cv2.undistort(frame, mtx, dist, None, newcameramtx)     mapx,mapy=cv2.initUndistortRectifyMap(mtx,dist, None ,newcameramtx,(w1,h1), 5 )     dst2=cv2 .remap(frame,mapx,mapy,cv2.INTER_LINEAR) # Crop the image and output the corrected image     x, y, w1, h1 = roi     dst1 = dst1[y:y + h1, x:x + w1]
    

    


    



    #cv2.imshow('frame',dst2) #cv2.imshow('dst1',dst1)     cv2.imshow( 'dst2' , dst2) if cv2.waitKey( 1 ) & 0xFF == ord( 'q' ):   # Press q to save a picture         cv2.imwrite( "../u4/frame.jpg" , dst1) break
    

    

        

camera.release() 
cv2.destroyAllWindows()