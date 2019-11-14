#ifndef P_CALIB_H
#define P_CALIB_H

#endif

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int64.h>

using namespace cv;
using namespace std;

/*!
 * - This class performs the initial step for calibrating the projector - camera interaction
 */

class calibrator {
public:
  /*!
   * - callback funtion that receives a frame from the camera and transforms it to a cv::Mat image to be used by the class
   * - if the calibration is done, publishes the resulting image in a ROS topic
   */
  void read_image(const sensor_msgs::CompressedImageConstPtr& msg);

  /*! - callback function that receives the threshold value choosed by the operator */
  void thresh_callback(const std_msgs::Int64ConstPtr &value);
  /*! - receives the cv::Mat camera frame and transforms it into a binary image using the threshold value */
  Mat thresh(Mat image);
  /*! - receives the binary image and finds the map area using a region growing algorithm with one seeding point, the center of the image */
  bool find_map(Mat image);

  image_transport::Publisher pub_;
  ros::Subscriber thresh_sub, sub;

private:
  int thresh_value;
  Mat final;
};
