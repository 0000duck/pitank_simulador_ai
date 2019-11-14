#include "projector_calib/p_calib.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  calibrator calib;
  ros::init(argc, argv, "p_calib");
  ros::NodeHandle nh;
  image_transport::ImageTransport it_(nh);

  calib.thresh_sub = nh.subscribe("/calibration_thresh", 1, &calibrator::thresh_callback, &calib);
  calib.pub_ = it_.advertise("/map", 1);

  ros::spin();

  return 0;
}
