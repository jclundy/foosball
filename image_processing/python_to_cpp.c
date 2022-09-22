cv2.undistort
cv2.aruco.detectMarkers
cv2.line
cv2.getPerspectiveTransform
cv2.warpPerspective
Cv2.polylines
Cv2.resize
cv2.GaussianBlur
cv2.cvtColor
cv2.inRange
Cv2.imshow
Cv2.erode
Cv2.dilate
cv2.minEnclosingCircle(c)
cv2.moments

where _InputArray is a class that can be constructed from Mat, Mat_<T>, Matx<T, m, n>, std::vector<T>, std::vector<std::vector<T> >, std::vector<Mat>, std::vector<Mat_<T> >, UMat, std::vector<UMat> or double. 

Mat cv::getOptimalNewCameraMatrix 	( 	InputArray  	cameraMatrix,
		InputArray  	distCoeffs,
		Size  	imageSize,
		double  	alpha,
		Size  	newImgSize = Size(),
		Rect *  	validPixROI = 0,
		bool  	centerPrincipalPoint = false 
	) 
	
cv.getOptimalNewCameraMatrix(	cameraMatrix, distCoeffs, imageSize, alpha[, newImgSize[, centerPrincipalPoint]]	) -> 	retval, validPixROI

void cv::undistort 	( 	InputArray  	src,
		OutputArray  	dst,
		InputArray  	cameraMatrix,
		InputArray  	distCoeffs,
		InputArray  	newCameraMatrix = noArray() 
	) 		
Python:
	cv.undistort(	src, cameraMatrix, distCoeffs[, dst[, newCameraMatrix]]	) -> 	dst


void cv::aruco::detectMarkers 	( 	InputArray  	image,
		const Ptr< Dictionary > &  	dictionary,
		OutputArrayOfArrays  	corners,
		OutputArray  	ids,
		const Ptr< DetectorParameters > &  	parameters = DetectorParameters::create(),
		OutputArrayOfArrays  	rejectedImgPoints = noArray() 
	) 		
Python:
	cv.aruco.detectMarkers(	image, dictionary[, corners[, ids[, parameters[, rejectedImgPoints]]]]	) -> 	corners, ids, rejectedImgPoints
