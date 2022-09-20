#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
// OpenCV includes

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
    
    ColorTracker() {

    }

    void handleNewFrame(Mat &frame) {
      Mat newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficients, frame.size(), 1, frame.size(), 0);
      //	frame = cv2.undistort(frame, cameraMatrix, distortionCoefficients, None, newcameramtx) 
      Mat undistorted;
      undistort(frame, undistorted, cameraMatrix, distortionCoefficients, newCameraMatrix);
    }
};

ColorTracker ballTracker;

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

    ballTracker.handleNewFrame(current_frame);

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
  double cameraMatrixValues[9] = { 798.86256178, 0, 401.52277111,
                                0, 812.90563717, 319.32828483,
                                0, 0, 1.0};
  double defaultDistortionCoefficients[5] = {-3.51591693e-01, 1.92604733e-01, 3.20674878e-04, 1.56190371e-04, -1.16111572e-01};
  ballTracker.cameraMatrix = Mat(3, 3, CV_64F, cameraMatrixValues);
  ballTracker.distortionCoefficients = Mat(1, 5, CV_64F, defaultDistortionCoefficients);

  // The name of the node
  ros::init(argc, argv, "color_tracker");
  
  // Default handler for nodes in ROS
  ros::NodeHandle nh;
   
  // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);
   
  // Subscribe to the /camera topic
  image_transport::Subscriber sub = it.subscribe("cv_camera/image_raw", 1, imageCallback);
   
  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();
   
  // Close down OpenCV
  destroyWindow("view");
}