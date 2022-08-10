import argparse
#import imutils
import cv2
import yaml
import numpy as np

import image_geometry as img_geo

from sensor_msgs.msg import CameraInfo


from skimage.metrics import structural_similarity as compare_ssim

class CameraCalibration:
	def __init__(self, matrix, dist, rect, proj):
		self.cameraMatrix = matrix
		self.distortionCoefficients = dist
		self.rectificationMatrix = rect
		self.projectionMatrix = proj


def parseCalibrationFile(fileName):
	calibrationFile = open(fileName, 'r')
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

	rectification_matrix = calibrationData['rectification_matrix']
	rmRows = rectification_matrix['rows']
	rmCols = rectification_matrix['cols']
	rectificationMatrixCoefficients = np.array(projection_matrix['data'])
	rectificationMatrixCoefficients = np.reshape(projectionMatrixCoefficients, (rmRows, rmCols))

	projection_matrix = calibrationData['projection_matrix']
	pmRows = projection_matrix['rows']
	pmCols = projection_matrix['cols']
	projectionMatrixCoefficients = np.array(projection_matrix['data'])
	projectionMatrixCoefficients = np.reshape(projectionMatrixCoefficients, (pmRows, pmCols))

	return CameraCalibration(cameraMatrix, distortionCoefficients, rectificationMatrixCoefficients, projectionMatrixCoefficients)

def yamlToCameraInfo(fileName):
	calibrationFile = open(fileName, 'r')
	calib_data = yaml.safe_load(calibrationFile)
	camera_info_msg = CameraInfo()
	camera_info_msg.width = calib_data["image_width"]
	camera_info_msg.height = calib_data["image_height"]
	camera_info_msg.K = calib_data["camera_matrix"]["data"]
	camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
	camera_info_msg.R = calib_data["rectification_matrix"]["data"]
	camera_info_msg.P = calib_data["projection_matrix"]["data"]
	camera_info_msg.distortion_model = calib_data["distortion_model"]
	return camera_info_msg

ap = argparse.ArgumentParser()


ap.add_argument("-i", "--image", required=True,
	help="input image")
args = vars(ap.parse_args())

inputImage = cv2.imread(args["image"])

calib1 = yamlToCameraInfo('calibrations/calibration1.yaml')
calib2 = yamlToCameraInfo('calibrations/calibration2.yaml')

cameraModel1 = img_geo.PinholeCameraModel()
cameraModel1.fromCameraInfo(calib1)

cameraModel2 = img_geo.PinholeCameraModel()
cameraModel2.fromCameraInfo(calib2)

height,width = inputImage.shape[:2]
w = width
h = height


#newCamMatrix1, roi=cv2.getOptimalNewCameraMatrix(calib1.cameraMatrix,calib1.distortionCoefficients,(w,h),1,(w,h))
#undistorted1 = cv2.undistort(inputImage, cameraModel1.K, cameraModel1.D, None, cameraModel1.K) 

#newCamMatrix2, roi=cv2.getOptimalNewCameraMatrix(calib2.cameraMatrix,calib2.distortionCoefficients,(w,h),1,(w,h))
#undistorted2 = cv2.undistort(inputImage, cameraModel1.K, cameraModel1.D, None, cameraModel1.K) 


#newCamMatrix1, roi = cv.getOptimalNewCameraMatrix(calib1.cameraMatrix, dist, (w,h), 1, (w,h))
#undistorted1 = cv2.undistort(inputImage, calib1.cameraMatrix, calib1.distortionCoefficients)
#undistorted2 = cv2.undistort(inputImage, calib2.cameraMatrix, calib2.distortionCoefficients) 

#mapx,mapy=cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
"""
mapx1 = cv.CreateImage((w, h), cv.IPL_DEPTH_32F, 1)
mapy1 = cv.CreateImage((w, h), cv.IPL_DEPTH_32F, 1)
cv.InitUndistortMap(cameraModel1.K, cameraModel1.D, mapx1, mapy1)
undistorted1 = cv.remap(raw, rectified, mapx1, mapy1)
"""

mapx,mapy=cv2.initUndistortRectifyMap(cameraModel1.K,cameraModel1.D,camreaModel1.R, cameraModel1.P,(w,h), cv2.CV16SC2)

newcameramtx1, roi = cv2.getOptimalNewCameraMatrix(cameraModel1.K, cameraModel1.D, (w,h), 0, (w,h))
mapx, mapy = cv2.initUndistortRectifyMap(cameraModel1.K, cameraModel1.D, None, newcameramtx1, (w,h), 5)
undistorted1 = cv2.remap(inputImage, mapx, mapy, cv2.INTER_LINEAR)

newcameramtx2, roi = cv2.getOptimalNewCameraMatrix(cameraModel2.K, cameraModel2.D, (w,h), 0, (w,h))
mapx, mapy = cv2.initUndistortRectifyMap(cameraModel2.K, cameraModel2.D, None, newcameramtx2, (w,h), 5)
undistorted2 = cv2.remap(inputImage, mapx, mapy, cv2.INTER_LINEAR)

#cameraModel1.rectifyImage(inputImage, undistorted1)
#cameraModel2.rectifyImage(inputImage, undistorted2)

binning_x = 0
binning_y = 0


imageA = undistorted1
imageB = undistorted2

grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)

(score, diff) = compare_ssim(grayA, grayB, full=True)
diff = (diff * 255).astype("uint8")
print("SSIM: {}".format(score))

thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
#cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#cnts = imutils.grab_contours(cnts)

# loop over the contours
#for c in cnts:
	# compute the bounding box of the contour and then draw the
	# bounding box on both input images to represent where the two
	# images differ
#	(x, y, w, h) = cv2.boundingRect(c)
#	cv2.rectangle(imageA, (x, y), (x + w, y + h), (0, 0, 255), 2)
#	cv2.rectangle(imageB, (x, y), (x + w, y + h), (0, 0, 255), 2)
# show the output images

cv2.imshow("origianl", inputImage)
cv2.imshow("calib 1", imageA)
cv2.imshow("calib 2", imageB)
cv2.imshow("Diff", diff)
cv2.imshow("Thresh", thresh)
cv2.waitKey(0)


"""
mapx,mapy=cv2.initUndistortRectifyMap(cameraModel1.K,cameraModel1.D,cameraModel1.R, cameraModel1.P,(w,h), cv2.CV_16SC2)
rectifiedNew = cv2.remap(inputImage, mapx, mapy, cv2.INTER_LINEAR)
cv2.imwrite("../image_processing/test1.png", rectifiedNew)
"""

