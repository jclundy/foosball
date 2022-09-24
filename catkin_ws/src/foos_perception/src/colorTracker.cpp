// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

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

class ColorTracker
{
  public: 
    Mat cameraMatrix;
    Mat distortionCoefficients;
    Ptr<aruco::Dictionary> arucoDictionary;
    Ptr<aruco::DetectorParameters> arucoDetectorParameters;

    bool regionOfInterestInitialized;
    std::vector<cv::Point2f> regionOfInterestCorners;
    std::vector<cv::Point2f> croppedFrameCorners;
    Mat warpTransform;
    Size outputImageSize;
    float outputWidth;
    float outputHeight;
    std::vector<float> colorMaskUpperBound;
    std::vector<float> colorMaskLowerBound;

    ColorTracker() {

      double cameraMatrixValues[9] = { 798.86256178, 0, 401.52277111,
                                    0, 812.90563717, 319.32828483,
                                    0, 0, 1.0};
      cameraMatrix = Mat(3, 3, CV_64F, cameraMatrixValues);

      ROS_INFO("Initialized camera matrix");

      double defaultDistortionCoefficients[5] = {-3.51591693e-01, 1.92604733e-01, 3.20674878e-04, 1.56190371e-04, -1.16111572e-01};
      distortionCoefficients = Mat(1, 5, CV_64F, defaultDistortionCoefficients);

      ROS_INFO("Initialized distortion coefficients");

      arucoDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
      arucoDetectorParameters = aruco::DetectorParameters::create();
      regionOfInterestInitialized = false;

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

      ROS_INFO("Initialized new frame points");

      croppedFrameCorners.push_back(newTopRight);
      croppedFrameCorners.push_back(newBottomRight);
      croppedFrameCorners.push_back(newBottomLeft);
      croppedFrameCorners.push_back(newTopLeft);

      ROS_INFO("Initialized cropped frame corners");

      regionOfInterestCorners.insert(regionOfInterestCorners.begin(), croppedFrameCorners.begin(), croppedFrameCorners.end());

      ROS_INFO("Initialized ROI frame corners");
    }

    void handleNewFrame(Mat &frame) {

      // Step 1 - perform distortion correction
      Mat newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficients, frame.size(), 1, frame.size(), 0);
      Mat undistorted;
      frame.copyTo(undistorted);

      ROS_INFO("Undistorted");


      // undistort(frame, undistorted, cameraMatrix, distortionCoefficients, newCameraMatrix);

      // Step 2) - detect aruco markers
      // std::vector<std::vector<cv::Point2f>> markerCorners;
      // std::vector<int> markerIds;
      // aruco::detectMarkers(undistorted, arucoDictionary, markerCorners, markerIds, arucoDetectorParameters);

      // if(markerCorners.size() >= 4 && regionOfInterestInitialized == false) {

      //   // marker ID 0 : use top right corner [1]
      //   // marker ID 1 : use bottom right corner [2]
      //   // marker ID 2 : use bottom left corner [3]
      //   // marker ID 3 : use top left corner [0]
      //   cv::Point2f topRight     = markerCorners[0][1];
      //   cv::Point2f bottomRight  = markerCorners[1][2];
      //   cv::Point2f bottomLeft   = markerCorners[2][3];
      //   cv::Point2f topLeft      = markerCorners[3][0];

      //   std::vector<cv::Point2f>::iterator it;
      //   it = regionOfInterestCorners.begin();

      //   regionOfInterestCorners.insert(it + 0, topRight);
      //   regionOfInterestCorners.insert(it + 1, bottomRight);
      //   regionOfInterestCorners.insert(it + 2, bottomLeft);
      //   regionOfInterestCorners.insert(it + 3, topLeft);

      //   warpTransform = getPerspectiveTransform(regionOfInterestCorners, croppedFrameCorners);
      //   regionOfInterestInitialized = true;

      // }

      // Step 3) - perform perspective transform
      Mat warped;
      // warpPerspective(undistorted, warped, warpTransform, outputImageSize);
      undistorted.copyTo(warped);
      ROS_INFO("Warped");

      // Step 4) Gaussian blur
      Mat blurred;
      cv::Size2d kernelSize(11,11);
      GaussianBlur(warped, blurred, kernelSize, 0);
      ROS_INFO("Blurred");

      // Step 5) Convert to HSV
      Mat hsv;
      cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
      ROS_INFO("HSV");

      // Step 6) Apply color mask
      Mat masked;
      inRange(hsv, colorMaskLowerBound, colorMaskUpperBound, masked);
      ROS_INFO("Masked");

      // Step 7) Perform erosion
      Mat eroded;
      erode(masked, eroded, Mat(),Point(-1,-1), 2);
      ROS_INFO("Eroded");

      // Step 8) Perform dilation
      Mat dilated;
      dilate(eroded, dilated, Mat(),Point(-1,-1), 4);
      ROS_INFO("Dilated");

      // Step 9) Find contours
      // std::vector<std::vector<cv::Point2f>> contours;
      // findContours(eroded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      // ROS_INFO("Contours");


      imshow("blurred", warped);
      imshow("masked", masked);
      imshow("dilated", dilated);
    }
};

ColorTracker* ballTracker;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Frame received");

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