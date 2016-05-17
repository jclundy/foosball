import argparse
import datetime
import time
import cv2
import numpy as np
camera = cv2.VideoCapture('C:/Users/joe/Documents/Python/VID_20160221_170847251.avi')

whiteLower = (0, 0, 0)
whiteUpper = (0, 0, 255)

yellowLower = (255, 255, 240)
yellowUpper = (204,204,0)

greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
#whiteLower = np.array([0,0,0], dtype=np.uint8)
#whiteUpper = np.array([0,0,255], dtype=np.uint8)
redLower = (17, 15, 100)
redUpper = (50, 56, 200)
firstFrame = None
while True:
	(grabbed, frame) = camera.read()
	if not grabbed:
		break

	height, width = frame.shape[:2]
	frame = cv2.resize(frame,(240, 480), interpolation = cv2.INTER_CUBIC)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	cv2.imshow("hsv", hsv)
	if firstFrame is None:
		firstFrame = hsv
		continue
		# compute the absolute difference between the current frame and
	# first frame
	#frameDelta = cv2.absdiff(firstFrame, gray)

	mask = cv2.inRange(hsv, redLower, redUpper)
	cv2.imshow("Mask", mask)

	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	eroded = mask
	cv2.imshow("Eroded", eroded)
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
		if radius > 5:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
 

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
 
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()