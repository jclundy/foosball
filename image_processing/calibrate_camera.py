import numpy as np
import cv2 as cv
import glob
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('*.jpg')

path = "/home/joe/Videos/Webcam/"
fileName = "2022-05-04-214101.webm"
filePath = path + fileName
capture = cv.VideoCapture(filePath)

while(capture.isOpened()):
	(grabbed, img) = capture.read()

	if grabbed == True:
		gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
		# Find the chess board corners

		ret, corners = cv.findChessboardCorners(gray, (9,6), cv.CALIB_CB_FAST_CHECK)
		# If found, add object points, image points (after refining them)
		if ret == True:
			objpoints.append(objp)
			corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
			imgpoints.append(corners)
			# Draw and display the corners
			cv.drawChessboardCorners(img, (7,6), corners2, ret)

		cv.imshow("gray", gray)
		cv.imshow('img', img)

		# Press Q on keyboard to  exit
		key = cv.waitKey(10) & 0xFF

		# if the 'q' key is pressed, stop the loop
		if key == ord("q"):
			break
	else:
		break
		
capture.release()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("ret")
print(ret)
print("mtx")
print(mtx)
print("dist")
print(dist)
print("rvecs")
print(rvecs)
print("tvecs")
print(tvecs)

cv.destroyAllWindows()
