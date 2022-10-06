// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

// std includes
#include <vector>
/*
Acknowledgements:
Used implementation presented here as a template
// Author: Addison Sears-Collins
// Website: https://automaticaddison.com
// Description: A basic image subscriber for ROS in C++
// Date: June 27, 2020

Then used this as a guide
http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
*/
using namespace cv;

void drawArucoCorners(Mat &, std::vector<std::vector<cv::Point2f>>, std::vector<int>);

class ColorTracker
{
  public: 
    Mat cameraMatrix;
    double cameraMatrixValues[9] = { 798.86256178, 0, 401.52277111,
                              0, 812.90563717, 319.32828483,
                              0, 0, 1.0};
    Mat distortionCoefficients;
    double defaultDistortionCoefficients[5] = {-3.51591693e-01, 1.92604733e-01, 3.20674878e-04, 1.56190371e-04, -1.16111572e-01};

    Ptr<aruco::Dictionary> arucoDictionary;
    Ptr<aruco::DetectorParameters> arucoDetectorParameters;

    cv::Point2f regionOfInterestCorners[4];
    bool regionOfInterestCornersInitialized[4] = {false, false, false, false};
    bool warpTransformInitialized = false;

    cv::Point2f croppedFrameCorners[4];
    Mat warpTransform;
    Size outputImageSize;
    float outputWidth;
    float outputHeight;
    std::vector<float> colorMaskUpperBound;
    std::vector<float> colorMaskLowerBound;

    ColorTracker() {


      cameraMatrix = Mat(3, 3, CV_64F, cameraMatrixValues);

      ROS_INFO("Initialized camera matrix");

      distortionCoefficients = Mat(1, 5, CV_64F, defaultDistortionCoefficients);

      ROS_INFO("Initialized distortion coefficients");

      arucoDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
      arucoDetectorParameters = aruco::DetectorParameters::create();

      ROS_INFO("Initialized aruco");

      outputWidth = 800;
      outputHeight = 600;
      outputImageSize = Size(outputWidth, outputHeight);

      ROS_INFO("Initialized output image size");

      colorMaskUpperBound = {180, 118, 255};
      colorMaskLowerBound = {50, 56, 200};

      ROS_INFO("Initialized color mask");

      cv::Point2f newBottomRight = cv::Point2f(outputWidth, outputHeight);
      cv::Point2f newTopRight = cv::Point2f(outputWidth, 0);
      cv::Point2f newTopLeft = cv::Point2f(0, 0);
      cv::Point2f newBottomLeft = cv::Point2f(0, outputHeight);
      croppedFrameCorners[0] = newBottomRight;
      croppedFrameCorners[1] = newTopRight;
      croppedFrameCorners[2] = newTopLeft;
      croppedFrameCorners[3] = newBottomLeft;
    }

    void handleNewFrame(Mat &frame) {

      // Step 1 - perform distortion correction
      Mat newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficients, frame.size(), 1, frame.size(), 0);
      Mat undistorted;
      undistort(frame, undistorted, cameraMatrix, distortionCoefficients, newCameraMatrix);

      // Step 2) - detect aruco markers
      std::vector<std::vector<cv::Point2f>> markerCorners;
      std::vector<int> markerIds;
      aruco::detectMarkers(undistorted, arucoDictionary, markerCorners, markerIds, arucoDetectorParameters);

      cv::Point2f cornerOfInterest;

      for (int i = 0; i < markerCorners.size(); i++) {

        // marker ID 0 : use top right corner [1]
        // marker ID 1 : use bottom right corner [2]
        // marker ID 2 : use bottom left corner [3]
        // marker ID 3 : use top left corner [0]
        int markerId = markerIds[i];
        switch(markerId) {
          case 0: {
            cornerOfInterest = markerCorners[i][1];
            break;
          }
          case 1: {
            cornerOfInterest = markerCorners[i][2];
            break;
          }
          case 2: {
            cornerOfInterest = markerCorners[i][3];
            break;
          }
          case 3: {
            cornerOfInterest = markerCorners[i][0];
            break;
          }
          default: {
            break;
          }
        }

        if(markerId >= 0 && markerId < 4) {
          if(!regionOfInterestCornersInitialized[markerId]) {
            ROS_INFO("initializing ROI corner %i, (%f, %f)", markerId, cornerOfInterest.x, cornerOfInterest.y);
            regionOfInterestCorners[markerId] = cornerOfInterest;
            regionOfInterestCornersInitialized[markerId] = true;
          }
        }
      }

      if(regionOfInterestInitialized() && !warpTransformInitialized) {
        ROS_INFO("initializing warp transform");
        warpTransform = getPerspectiveTransform(regionOfInterestCorners, croppedFrameCorners);
        warpTransformInitialized = true;
      }

      // Step 3) - perform perspective transform
      Mat warped;
      warpPerspective(undistorted, warped, warpTransform, outputImageSize);

      // Step 4) Gaussian blur
      Mat blurred;
      cv::Size2d kernelSize(11,11);
      GaussianBlur(warped, blurred, kernelSize, 0);

      // Step 5) Convert to HSV
      Mat hsv;
      cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

      // Step 6) Apply color mask
      Mat masked;
      inRange(hsv, colorMaskLowerBound, colorMaskUpperBound, masked);

      // Step 7) Perform erosion
      Mat eroded;
      erode(masked, eroded, Mat(),Point(-1,-1), 2);

      // Step 8) Perform dilation
      Mat dilated;
      dilate(eroded, dilated, Mat(),Point(-1,-1), 4);

      // Step 9) Find contours
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      findContours(dilated, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // Step 10) Find largest countour
      Point ballCenter;
      float radius = 0;

      for (size_t i = 0; i < contours.size(); i++) {
        Point2f contourCenter;
        float contourRadius;
        minEnclosingCircle(contours[i], contourCenter, contourRadius);
        if(contourRadius > radius) {
          radius = contourRadius;
          ballCenter = contourCenter;
        }
      }

      Mat gray;
      cvtColor(warped, gray, COLOR_BGR2GRAY);
      medianBlur(gray, gray, 5);

      // Step 11) Canny edge detection
      Mat cannyOutput;
      Canny(gray, cannyOutput, 100, 200);

      // Step 12) Hough circles
      std::vector<Vec3f> circles;
      HoughCircles(gray, circles, HOUGH_GRADIENT, 1, gray.rows/16, 100, 100, 10, 400);

      // Step 13) Hough lines
      std::vector<Vec2f> lines;
      HoughLines(cannyOutput, lines, 1, CV_PI/180, 150, 0,0);

      Mat experimentMarkup;
      warped.copyTo(experimentMarkup);

      for(size_t i = 0; i < circles.size(); i++) {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        // circle center
        circle( experimentMarkup, center, 1, Scalar(0,100,100), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle( experimentMarkup, center, radius, Scalar(255,0,255), 3, LINE_AA);
      }

      for( size_t i = 0; i < lines.size(); i++ ) {
          float rho = lines[i][0], theta = lines[i][1];
          Point pt1, pt2;
          double a = cos(theta), b = sin(theta);
          double x0 = a*rho, y0 = b*rho;
          pt1.x = cvRound(x0 + 1000*(-b));
          pt1.y = cvRound(y0 + 1000*(a));
          pt2.x = cvRound(x0 - 1000*(-b));
          pt2.y = cvRound(y0 - 1000*(a));
          line( experimentMarkup, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
      }


      // Mark up
      Mat markupFrame;
      undistorted.copyTo(markupFrame);
      drawArucoCorners(markupFrame,markerCorners, markerIds);
      drawRegionOfInterest(markupFrame);

      Mat detectionMarkupFrame;
      warped.copyTo(detectionMarkupFrame);
      for (size_t i = 0; i < contours.size(); i++) {
        drawContours(detectionMarkupFrame, contours, (int)i, Scalar(255,0,0), 2, LINE_8, hierarchy);
      }
      circle(detectionMarkupFrame, ballCenter, radius, Scalar(0,255,0), 2);


      Point textPosition(ballCenter.x + radius, ballCenter.y + radius);

      std::string ballPositionText = "Ball position : (";
      ballPositionText += std::to_string(ballCenter.x);
      ballPositionText += ", ";
      ballPositionText += std::to_string(ballCenter.y);
      ballPositionText += ")";

      putText(detectionMarkupFrame, ballPositionText, textPosition,
              cv::FONT_HERSHEY_SIMPLEX, 0.25,
              Scalar(0, 0, 255), 1, LINE_8);

      imshow("blurred", warped);
      imshow("masked", masked);
      imshow("dilated", dilated);
      imshow("pre-processing markup", markupFrame);
      imshow("detection markup", detectionMarkupFrame);
      imshow("canny", cannyOutput);

      imshow("circle detection", experimentMarkup);

      // publish data
      ROS_INFO("radius %f", radius);
      ROS_INFO("ball position px (%i, %i)", ballCenter.x, ballCenter.y);

    }

    bool regionOfInterestInitialized() {
      bool result = true;
      for (int i = 0; i < 4; i++) {
        result &= regionOfInterestCornersInitialized[i];
      }
      return result;
    }

    void drawRegionOfInterest(Mat frame) {
      if(regionOfInterestInitialized()) {
        // marker ID 0 : use top right corner [1]
        // marker ID 1 : use bottom right corner [2]
        // marker ID 2 : use bottom left corner [3]
        // marker ID 3 : use top left corner [0]
        line(frame, regionOfInterestCorners[3], regionOfInterestCorners[0], Scalar(255, 0, 0), 2);
        line(frame, regionOfInterestCorners[0], regionOfInterestCorners[1], Scalar(255, 0, 0), 2);
        line(frame, regionOfInterestCorners[1], regionOfInterestCorners[2], Scalar(255, 0, 0), 2);
        line(frame, regionOfInterestCorners[2], regionOfInterestCorners[3], Scalar(255, 0, 0), 2);
      }
    }
};

ColorTracker* ballTracker;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Pointer used for the conversion from a ROS message to 
  // an OpenCV-compatible image
  cv_bridge::CvImagePtr cv_ptr;
   
  try
  { 
   
    // Convert the ROS message  
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
     
    // Store the values of the OpenCV-compatible image
    // into the current_frame variable
    Mat current_frame = cv_ptr->image;

    // Display the current frame
    imshow("raw", current_frame); 

    ballTracker->handleNewFrame(current_frame);

    // Display frame for 30 milliseconds
    waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{

  ROS_INFO("Starting");

  ballTracker = new ColorTracker;

  ROS_INFO("Initialized ball tracker");

  // The name of the node
  ros::init(argc, argv, "color_tracker");
  
  // Default handler for nodes in ROS
  ros::NodeHandle nh;
   
  // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);
   
  // Subscribe to the /camera topic
  image_transport::Subscriber sub = it.subscribe("videofile/image_raw", 1, imageCallback);
  
  ROS_INFO("Subscribed");
  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();
   
  // Close down OpenCV
  destroyWindow("view");
}

void drawArucoCorners(Mat &frame,
  std::vector<std::vector<cv::Point2f>> markerCorners,
  std::vector<int> markerIds) {
    for (int i = 0; i < markerCorners.size(); i++) {
      cv::Point2f topLeft = markerCorners[i][0];
      cv::Point2f topRight = markerCorners[i][1];
      cv::Point2f bottomRight = markerCorners[i][2];
      cv::Point2f bottomLeft = markerCorners[i][3];

      cv::rectangle(frame, topLeft, bottomRight, Scalar(0, 255, 0), 2);
    }
}