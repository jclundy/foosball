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
void drawRejectedArucoCorners(Mat &, std::vector<std::vector<cv::Point2f>>);

class ColorTracker
{
  public: 
    Mat cameraMatrix;
    double cameraMatrixValues[9] = { 1022.118843226931, 0, 656.8463531767748, 0, 1023.925425916706, 427.0013534242285, 0, 0, 1};
    Mat distortionCoefficients;
    double defaultDistortionCoefficients[5] = {-0.3926204279821043, 0.143966972216399, -0.005840266157887177, -0.002891787240109053, 0};

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

    cv::Point2f arucoFieldCornerMarkersRealWorldPosition[4];
    cv::Point2f arucoFieldCornerMarkersPixelPosition[4];

    bool fieldMarkerInitialized[4] = {false, false, false, false};

    Mat pixelToWorldTransform;
    bool pixelToWorldTransformInitialized = false;
    Size realWorldSize;

    int arucoIdToIndexMap[8] = {0,1,2,3, 0, 1, 2, 3 };


    ColorTracker() {


      cameraMatrix = Mat(3, 3, CV_64F, cameraMatrixValues);

      ROS_INFO("Initialized camera matrix");

      distortionCoefficients = Mat(1, 5, CV_64F, defaultDistortionCoefficients);

      ROS_INFO("Initialized distortion coefficients");

      arucoDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
      arucoDetectorParameters = aruco::DetectorParameters::create();

      ROS_INFO("Initialized aruco");

      outputWidth = 398 * 2;
      outputHeight = 288 * 2;
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

      cv::Point2f offset = cv::Point2f(3,3);
      int idx0 = arucoIdToIndexMap[4];
      int idx1 = arucoIdToIndexMap[5];
      int idx2 = arucoIdToIndexMap[6];
      int idx3 = arucoIdToIndexMap[7];

      arucoFieldCornerMarkersRealWorldPosition[idx0] = cv::Point2f(3,3) - offset; // Marker 4 : top left  
      arucoFieldCornerMarkersRealWorldPosition[idx1] = cv::Point2f(3, 285) - offset; // Marker 5: bottom left
      arucoFieldCornerMarkersRealWorldPosition[idx2] = cv::Point2f(395, 285) - offset; // Marker 6: bottom right
      arucoFieldCornerMarkersRealWorldPosition[idx3] = cv::Point2f(395, 3) - offset; // Marker 7: top right

      // realWorldSize = cv::Size(398, 288);
      realWorldSize = cv::Size(392, 282);

    }

    void handleNewFrame(Mat &frame) {

      // Step 1 - perform distortion correction
      // Mat newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficients, frame.size(), 1, frame.size(), 0);
      Mat undistorted;
      // undistort(frame, undistorted, cameraMatrix, distortionCoefficients, newCameraMatrix);
      frame.copyTo(undistorted);

      // Mat undistortedResized;
      // resize(frame, undistorted, Size(), 0.5, 0.5);


      // scale down image to roughly 600x800
      


      // Step 2) - detect aruco markers
      std::vector<std::vector<cv::Point2f>> markerCorners;
      std::vector<std::vector<cv::Point2f>> rejectedCorners;
      std::vector<int> markerIds;
      aruco::detectMarkers(undistorted, arucoDictionary, markerCorners, markerIds, arucoDetectorParameters, rejectedCorners);

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
          case 4: {
            // top left
            cornerOfInterest = markerCorners[i][0];
            break;
          }
          case 5: {
            // bottom left
            cornerOfInterest = markerCorners[i][0];
            break;
          }
          case 6: {
            // bottom right
            cornerOfInterest = markerCorners[i][0];
            break;
          }
          case 7: {
            // top right
            cornerOfInterest = markerCorners[i][0];
            break;
          }                              
          default: {
            break;
          }
        }

        if(markerId >= 0 && markerId <= 3) {
          if(!regionOfInterestCornersInitialized[markerId]) {
            ROS_INFO("initializing ROI corner %i, (%f, %f)", markerId, cornerOfInterest.x, cornerOfInterest.y);
            regionOfInterestCorners[markerId] = cornerOfInterest;
            regionOfInterestCornersInitialized[markerId] = true;
          }
        }

        if(markerId >= 4 && markerId <= 7) {
          int idx = arucoIdToIndexMap[markerId];
          if(!fieldMarkerInitialized[idx]) {
            ROS_INFO("field corner %i, (%f, %f)", markerId, cornerOfInterest.x, cornerOfInterest.y);
            arucoFieldCornerMarkersPixelPosition[idx] = cornerOfInterest;
            fieldMarkerInitialized[idx] = true;
          }
        }

      }

      if(regionOfInterestInitialized() && !warpTransformInitialized) {
        ROS_INFO("initializing warp transform");
        warpTransform = getPerspectiveTransform(regionOfInterestCorners, croppedFrameCorners);
        warpTransformInitialized = true;

        double m00 = warpTransform.at<double>(0,0);
        double m01 = warpTransform.at<double>(0,1);
        double m02 = warpTransform.at<double>(0,2);
        double m10 = warpTransform.at<double>(1,0);
        double m11 = warpTransform.at<double>(1,1);
        double m12 = warpTransform.at<double>(1,2);
        double m20 = warpTransform.at<double>(2,0);
        double m21 = warpTransform.at<double>(2,1);
        double m22 = warpTransform.at<double>(2,2);

        ROS_INFO("initializing warp transform values");

        ROS_INFO("%f, %f, %f", m00, m01, m01);
        ROS_INFO("%f, %f, %f", m10, m11, m11);
        ROS_INFO("%f, %f, %f", m20, m21, m21);

      }

      if(fieldMarkersInitialized() && !pixelToWorldTransformInitialized) {
        ROS_INFO("initializing field warp transform");

        pixelToWorldTransform = getPerspectiveTransform(arucoFieldCornerMarkersPixelPosition, arucoFieldCornerMarkersRealWorldPosition);
        pixelToWorldTransformInitialized = true;

        double m00 = pixelToWorldTransform.at<double>(0,0);
        double m01 = pixelToWorldTransform.at<double>(0,1);
        double m02 = pixelToWorldTransform.at<double>(0,2);
        double m10 = pixelToWorldTransform.at<double>(1,0);
        double m11 = pixelToWorldTransform.at<double>(1,1);
        double m12 = pixelToWorldTransform.at<double>(1,2);
        double m20 = pixelToWorldTransform.at<double>(2,0);
        double m21 = pixelToWorldTransform.at<double>(2,1);
        double m22 = pixelToWorldTransform.at<double>(2,2);

        ROS_INFO("field warp transform values");
        ROS_INFO("%f, %f, %f", m00, m01, m01);
        ROS_INFO("%f, %f, %f", m10, m11, m11);
        ROS_INFO("%f, %f, %f", m20, m21, m21);  

      }

/**
 * @brief 
 * 
 *
      // Step 3) - apply mask on ROI perform perspective transform
      Mat warped;
      warpPerspective(undistorted, warped, warpTransform, outputImageSize);

 */
  

      // Step 4) Gaussian blur
      Mat blurred;
      cv::Size2d kernelSize(11,11);
      GaussianBlur(undistorted, blurred, kernelSize, 0);

      // Step 5) Convert to HSV
      Mat hsv;
      cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

      // Step 6) Apply color mask
      Mat masked;
      Mat roiMask;
      inRange(hsv, colorMaskLowerBound, colorMaskUpperBound, masked);

      Mat maskedOutput;
      if(regionOfInterestInitialized()) {

        Mat masked2;
        masked.convertTo(masked2, CV_8U);

        // Step 6a) ROI mask
        roiMask = Mat::zeros(masked2.size(), masked2.type());
        int contourNumPoints[1] = {4};

        Point roiContours[4];
        for (int i = 0; i < 4; i++) {
          roiContours[i].x = round(regionOfInterestCorners[i].x);
          roiContours[i].y = round(regionOfInterestCorners[i].y);
        }


        static bool printROI = true;
        if(printROI) {
          ROS_INFO("ROI points:");
          printROI = false;
          for(int i = 0; i < 4; i++) {
            ROS_INFO("Corner %i: (%li, %li)", i, roiContours[i].x, roiContours[i].y);
          }
        }

        const Point* countoursPtr[1] = {roiContours}; 
        fillPoly(roiMask, countoursPtr, contourNumPoints, 1, Scalar(255), LINE_8);

        Mat blurredCopy;
        blurred.copyTo(blurredCopy);
        fillPoly(blurredCopy, countoursPtr, contourNumPoints, 1, Scalar(255,255,255), LINE_8);
        imshow("blurred copy", blurredCopy);

        maskedOutput = Mat::zeros(masked2.size(), masked2.type());
        static bool printShapes = true;

        if(printShapes) {
          printShapes = false;
          ROS_INFO("masked2 shape (%i,%i)", masked2.cols, masked2.rows);
          ROS_INFO("roi mask shape (%i,%i)", roiMask.cols, roiMask.rows);
          ROS_INFO("output shape (%i,%i)", maskedOutput.cols, maskedOutput.rows);
        }
        bitwise_and(masked2, roiMask, maskedOutput);
      } else {
        masked.copyTo(maskedOutput);
      }



      // Step 7) Perform erosion
      Mat eroded;
      erode(maskedOutput, eroded, Mat(),Point(-1,-1), 2);

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

      // Mark up
      Mat markupFrame;
      undistorted.copyTo(markupFrame);
      drawArucoCorners(markupFrame,markerCorners, markerIds);
      // drawRejectedArucoCorners(markupFrame, rejectedCorners);
      drawRegionOfInterest(markupFrame);
      drawFieldMarkers(markupFrame);
      
      Mat markupResized;
      resize(markupFrame, markupResized, Size(), 0.5, 0.5);

      Mat detectionMarkupFrame;
      undistorted.copyTo(detectionMarkupFrame);
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


      // imshow("undistorted", undistortedResized);
      // imshow("blurred", blurred);
      imshow("masked", masked);
      imshow("roi mask", roiMask);
      imshow("masked output", maskedOutput);
      // imshow("dilated", dilated);
      imshow("pre-processing markup", markupResized);
      imshow("detection markup", detectionMarkupFrame);

      if(pixelToWorldTransformInitialized) {
        Mat fieldWarped;
        warpPerspective(undistorted, fieldWarped, pixelToWorldTransform, realWorldSize);

        cv::Point2f referenceA = cv::Point2f(66,71);
        cv::Point2f referenceB = cv::Point2f(40,106);
        circle(fieldWarped, referenceA, 5, Scalar(0,0,255), 2);
        circle(fieldWarped, referenceB, 5, Scalar(0,0,255), 2);

        double ballPositionValues[3] = {ballCenter.x, ballCenter.y, 1};
        Mat ballPositionUnWarped = Mat(3, 1, CV_64F, ballPositionValues);

        Mat ballPositionWarped = pixelToWorldTransform.inv() * ballPositionUnWarped;
        float scaleFactor = ballPositionWarped.at<double>(2);
        ballPositionWarped /= scaleFactor;

        Point2f ballPosition2d = cv::Point2f(ballPositionWarped.at<double>(0), ballPositionWarped.at<double>(1)) - cv::Point2f(3,3);
        circle(fieldWarped, ballPosition2d, 10, Scalar(0,0,255), 2);
        imshow("field warped", fieldWarped);

      }
      // publish data
      // ROS_INFO("radius %f", radius);
      // ROS_INFO("ball position px (%i, %i)", ballCenter.x, ballCenter.y);

    }

    bool regionOfInterestInitialized() {
      bool result = true;
      for (int i = 0; i <= 3; i++) {
        result &= regionOfInterestCornersInitialized[i];
      }
      return result;
    }

    bool fieldMarkersInitialized() {
      bool result = true;
      for (int i = 0; i <= 3; i++) {
        result &= fieldMarkerInitialized[i];
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

    void drawFieldMarkers(Mat frame) {
      for (int i = 0; i <= 3; i++) {
        if(fieldMarkerInitialized[i]) {
          cv::Point2f corner = arucoFieldCornerMarkersPixelPosition[i];

          circle(frame, corner, 10, Scalar(0,255,0),2, FILLED);
          Point textPosition(corner.x + 20, corner.y + 20);

          std::string text = "Corner pixels : (";
          text += std::to_string(corner.x);
          text += ", ";
          text += std::to_string(corner.y);
          text += ")";

          putText(frame, text, textPosition, cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1, LINE_8);
        }
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
    // imshow("raw", current_frame); 

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

void drawRejectedArucoCorners(Mat &frame, std::vector<std::vector<cv::Point2f>> markerCorners) {
    for (int i = 0; i < markerCorners.size(); i++) {
      cv::Point2f topLeft = markerCorners[i][0];
      cv::Point2f topRight = markerCorners[i][1];
      cv::Point2f bottomRight = markerCorners[i][2];
      cv::Point2f bottomLeft = markerCorners[i][3];

      cv::rectangle(frame, topLeft, bottomRight, Scalar(0, 255, 0), 2);

      Point textPosition((topLeft.x + bottomRight.x) / 2, (topLeft.y + bottomRight.y) / 2);

      std::string ballPositionText = "idx ";
      ballPositionText += std::to_string(i);

      putText(frame, ballPositionText, textPosition,
              cv::FONT_HERSHEY_SIMPLEX, 0.15,
              Scalar(0, 0, 255), 1, LINE_8);

    }
}