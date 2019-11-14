#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float64MultiArray.h>

#include <AprilTags/TagDetector.h>
#include <tf/transform_broadcaster.h>

//aaguiar96 implementation
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// -----

#define PI 3.14159265358979323846

namespace apriltags_ros{

class AprilTagDescription{
 public:
  AprilTagDescription(int id, double size, std::string &frame_name):id_(id), size_(size), frame_name_(frame_name){}
  double size(){return size_;}
  int id(){return id_;}
  std::string& frame_name(){return frame_name_;}
 private:
  int id_;
  double size_;
  std::string frame_name_;
};


class AprilTagDetector{
 public:
  AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~AprilTagDetector();
 private:
  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
  std::map<int, AprilTagDescription> parse_tag_descriptions(XmlRpc::XmlRpcValue& april_tag_descriptions);

 private:
  std::map<int, AprilTagDescription> descriptions_;
  std::string sensor_frame_id_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  boost::shared_ptr<AprilTags::TagDetector> tag_detector_;
  bool projected_optics_;
  bool use_cam_info_;
  double cam_fx_;
  double cam_fy_;
  double cam_px_;
  double cam_py_;
  double cam_pos_roll_;
  double cam_pos_pitch_;
  double cam_pos_yaw_;
  double cam_pos_x_;
  double cam_pos_y_;
  double cam_pos_z_;
  bool   simulation_;
  bool   send_transform_;

  //aaguiar96 implementation

  void calibReceiver(const std_msgs::Float32MultiArrayConstPtr& map_corners);

  double angle;
  std_msgs::Float64MultiArray tag_corners;
  ros::Publisher tag_publisher;
  ros::Subscriber calib_sub;
  std_msgs::Float32MultiArrayConstPtr map_corners;
  std_msgs::BoolConstPtr init;
  int max_x, max_y, min_x, min_y;
  cv::Rect roi;
  std::vector<cv::Point2f> corners, squareCorners;
  cv::Mat H, cropped, gray;
  geometry_msgs::PoseStamped tag_pose, pixel_pose;
  // -----
};



}


#endif
