import argparse
import datetime
import time
import cv2
import numpy as np
import os

import yaml

calibrationFile = open('camera.yaml', 'r')
calibrationData = yaml.safe_load(calibrationFile)

camera_matrix = calibrationData['camera_matrix']
numColumns = camera_matrix['cols']
numRows =  camera_matrix['rows']
cameraMatrix = np.array(camera_matrix['data'])
cameraMatrix = np.reshape(cameraMatrix, (numRows, numColumns))

distortion_coefficients = calibrationData['distortion_coefficients']
numColumns = distortion_coefficients['cols']
numRows =  distortion_coefficients['rows']
distortionCoefficients = np.array(distortion_coefficients['data'])
distortionCoefficients = np.reshape(distortionCoefficients, (numRows, numColumns))

filePath = "/home/joe/Videos/Webcam/2021-12-05-144129.webm"
camera = cv2.VideoCapture(filePath)

#camera = cv2.VideoCapture(3)

fourcc = cv2.VideoWriter_fourcc(*'DIVX')


whiteLower = (0, 0, 0)
whiteUpper = (0, 0, 255)

yellowLower = (255, 255, 240)
yellowUpper = (204,204,0)

greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

redLower = (17, 15, 100)
redUpper = (50, 56, 200)

lower_blue = np.array([60,50,50])
upper_blue = np.array([130,255,255])

pinkLower = (140, 10, 225)
pinkUpper = (180, 118, 255)

upper = pinkUpper
lower = redUpper

firstFrame = None

(grabbed, frame) = camera.read()
height, width = frame.shape[:2]

output_h = 240
output_w = 320

ct = datetime.datetime.now()
timestamp = ct.strftime("%m-%d-%Y_%H:%M:%S")
fileName1 = 'frame_output_' + timestamp + '.avi'
fileName2 = 'original_' + timestamp + '.avi'

path = "recordings/" + timestamp
os.makedirs(path, exist_ok = True)

videoWriters = {}
fileNames = ['original', 'masked', 'eroded', 'dilated', 'tracker']

for name in fileNames:
	fileNameOutput = path + '/' + name + '_' + timestamp + '.avi'
	videoWriters[name] = cv2.VideoWriter(fileNameOutput, fourcc, 20.0, (output_w, output_h))

print("image height and width", height, width)

while True:
	(grabbed, frame) = camera.read()
	if not grabbed:
		camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
		continue


	original = frame.copy()

	height, width = frame.shape[:2]
	frame = cv2.undistort(frame, cameraMatrix, distortionCoefficients) 

	undistorted = frame.copy()
	frame = cv2.resize(frame,(output_w, output_h), interpolation = cv2.INTER_CUBIC)

	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	cv2.imshow("hsv", hsv)
	if firstFrame is None:
		firstFrame = hsv
		continue
		# compute the absolute difference between the current frame and
	# first frame
	#frameDelta = cv2.absdiff(firstFrame, gray)

	mask = cv2.inRange(hsv, lower, upper)
	cv2.imshow("Mask", mask)
	mask_to_save = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

	mask = cv2.erode(mask, None, iterations=2)

	eroded = mask.copy()
	eroded_to_save = cv2.cvtColor(eroded,cv2.COLOR_GRAY2BGR)
	cv2.imshow("Eroded", eroded)
	
	mask = cv2.dilate(mask, None, iterations=2)
	dilated = mask.copy()
	dilated_to_save = cv2.cvtColor(dilated,cv2.COLOR_GRAY2BGR)
	cv2.imshow("Dilated", dilated)

	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None
 
	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
		# only proceed if the radius meets a minimum size
		if( radius > 5):
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			#print("radius: %f\n",radius)	
 

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	
	cv2.imshow("undistorted", undistorted) 
	
	cv2.imshow("original", original)
	
	framesToSave = {'original':original, 'masked':mask_to_save,'eroded':eroded_to_save, 'dilated':dilated_to_save, 'tracker':frame}
	for name in fileNames:
		videoWriters[name].write(framesToSave[name])
	
	key = cv2.waitKey(1) & 0xFF
 
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
 
# cleanup the camera and close any open windows
camera.release()

for name in fileNames:
		videoWriters[name].release()
cv2.destroyAllWindows()
