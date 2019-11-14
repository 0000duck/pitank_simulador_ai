#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>
#include <ros/transport_hints.h>

namespace apriltags_ros{

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh): it_(nh), send_transform_(false){
  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
    ROS_WARN("No april tags specified");
  }
  else{
    try{
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
    }
  }

  if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)){
    sensor_frame_id_ = "camera::camera_link";
  }

  std::string tag_family;
  pnh.param<std::string>("tag_family", tag_family, "36h11");

  pnh.param<bool>("projected_optics", projected_optics_, false);

  pnh.param<bool>("use_cam_info", use_cam_info_, false);

  pnh.param<double>("cam_fx", cam_fx_, 700.0);
  pnh.param<double>("cam_fy", cam_fy_, 700.0);
  pnh.param<double>("cam_px", cam_px_, 320.0);
  pnh.param<double>("cam_py", cam_py_, 240.0);

  pnh.param<bool>("simulation",simulation_, false);

  //! orientation defaults with respect to iris's camera in gazebo
  pnh.param<double>("cam_position_roll",cam_pos_roll_,  0.0);
  pnh.param<double>("cam_position_pitch",cam_pos_pitch_, 0.0);
  pnh.param<double>("cam_position_yaw",cam_pos_yaw_,   0.0);
  pnh.param<double>("cam_position_x",cam_pos_x_,  0.0);
  pnh.param<double>("cam_position_y",cam_pos_y_, 0.0);
  pnh.param<double>("cam_position_z",cam_pos_z_,   0.0);


  ROS_INFO("Cam params: fx: %f fy: %f px: %f py: %f", cam_fx_, cam_fy_, cam_px_, cam_py_);
  ROS_INFO("Orientation params: roll: %f pitch: %f yaw: %f ", cam_pos_roll_,
                                                              cam_pos_pitch_,
                                                              cam_pos_yaw_);


  if(use_cam_info_)
  {
    ROS_INFO("Getting px,py,fx,fy parameters from the camera");
  }


  if(simulation_)
  {
    ROS_INFO("in simulation mode");
  }


  const AprilTags::TagCodes* tag_codes;
  if(tag_family == "16h5"){
    tag_codes = &AprilTags::tagCodes16h5;
  }
  else if(tag_family == "25h7"){
    tag_codes = &AprilTags::tagCodes25h7;
  }
  else if(tag_family == "25h9"){
    tag_codes = &AprilTags::tagCodes25h9;
  }
  else if(tag_family == "36h9"){
    tag_codes = &AprilTags::tagCodes36h9;
  }
  else if(tag_family == "36h11"){
    tag_codes = &AprilTags::tagCodes36h11;
  }
  else{
    ROS_WARN("Invalid tag family specified; defaulting to 36h11");
    tag_codes = &AprilTags::tagCodes36h11;
  }


  //aaguiar96 implementation

  map_corners = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/map_corners");

  max_x = 0; max_y = 0; min_x = 1500; min_y = 1500;
  for(size_t i = 0; i < map_corners->data.size(); i++) {
    if(map_corners->data[i] > max_x)
      max_x = map_corners->data[i];
    if(map_corners->data[i] < min_x)
      min_x = map_corners->data[i];
    i++;
  }

  for(size_t j = 1; j < map_corners->data.size(); j++) {
    if(map_corners->data[j] > max_y)
      max_y = map_corners->data[j];
    if(map_corners->data[j] < min_y)
      min_y = map_corners->data[j];
    j++;
  }

  squareCorners.clear();
  corners.clear();
  squareCorners.push_back(cv::Point2f(0, 0));
  squareCorners.push_back(cv::Point2f(max_x - min_x, 0));
  squareCorners.push_back(cv::Point2f(0, max_y - min_y));
  squareCorners.push_back(cv::Point2f(max_x - min_x, max_y - min_y));
  corners.push_back(cv::Point2f(map_corners->data[0] - min_x, map_corners->data[1] - min_y));
  corners.push_back(cv::Point2f(map_corners->data[4] - min_x, map_corners->data[5] - min_y));
  corners.push_back(cv::Point2f(map_corners->data[2] - min_x, map_corners->data[3] - min_y));
  corners.push_back(cv::Point2f(map_corners->data[6] - min_x, map_corners->data[7] - min_y));

  H = getPerspectiveTransform(corners, squareCorners);


  calib_sub = nh.subscribe("/map_corners", 1, &AprilTagDetector::calibReceiver, this);
  tag_publisher = nh.advertise<AprilTagDetectionArray>("tag_pixel_detections", 1);
  // -----

  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
  image_sub_ = it_.subscribeCamera("image_rect", 1, &AprilTagDetector::imageCb, this);
}

AprilTagDetector::~AprilTagDetector() {}

void AprilTagDetector::calibReceiver(const std_msgs::Float32MultiArrayConstPtr& msg) {

  max_x = 0; max_y = 0; min_x = 1500; min_y = 1500;
  for(size_t i = 0; i < msg->data.size(); i++) {
    if(msg->data[i] > max_x)
      max_x = msg->data[i];
    if(msg->data[i] < min_x)
      min_x = msg->data[i];
    i++;
  }

  for(size_t j = 1; j < msg->data.size(); j++) {
    if(msg->data[j] > max_y)
      max_y = msg->data[j];
    if(msg->data[j] < min_y)
      min_y = msg->data[j];
    j++;
  }

  squareCorners.clear();
  corners.clear();
  squareCorners.push_back(cv::Point2f(0, 0));
  squareCorners.push_back(cv::Point2f(max_x - min_x, 0));
  squareCorners.push_back(cv::Point2f(0, max_y - min_y));
  squareCorners.push_back(cv::Point2f(max_x - min_x, max_y - min_y));
  corners.push_back(cv::Point2f(msg->data[0] - min_x, msg->data[1] - min_y));
  corners.push_back(cv::Point2f(msg->data[4] - min_x, msg->data[5] - min_y));
  corners.push_back(cv::Point2f(msg->data[2] - min_x, msg->data[3] - min_y));
  corners.push_back(cv::Point2f(msg->data[6] - min_x, msg->data[7] - min_y));

  H = getPerspectiveTransform(corners, squareCorners);

}

void AprilTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //aaguiar96 implementation
  if(min_x - 10 > 0 && min_y - 10 > 0 && max_x + 35 < cv_ptr->image.cols && max_y + 35 < cv_ptr->image.rows) {
    roi.x = min_x - 10;
    roi.y = min_y - 10;
    roi.width = max_x - min_x + 25;
    roi.height = max_y - min_y + 25;
  }
  else {
    roi.x = min_x;
    roi.y = min_y;
    roi.width = max_x - min_x;
    roi.height = max_y - min_y;
  }

  cropped = cv_ptr->image(roi);

  //cv::warpPerspective(cropped, cropped, H, cropped.size());
  cv::cvtColor(cropped, gray, CV_BGR2GRAY);
  // -----

  std::vector<AprilTags::TagDetection>	detections = tag_detector_->extractTags(gray);

  double fx;
  double fy;
  double px;
  double py;

  if(!use_cam_info_)
  {
      fx = cam_fx_;
      fy = cam_fy_;
      px = cam_px_;
      py = cam_py_;

  } else {
    if (projected_optics_) {
      // use projected focal length and principal point
      // these are the correct values
      fx = cam_info->P[0];
      fy = cam_info->P[5];
      px = cam_info->K[2];
      py = cam_info->K[6];
    } else {
      // use camera intrinsic focal length and principal point
      // for backwards compatability
      fx = cam_info->K[0];
      fy = cam_info->K[4];
      px = cam_info->K[2];
      py = cam_info->K[5];
    }
  }
  if(!sensor_frame_id_.empty())
    cv_ptr->header.frame_id = sensor_frame_id_;

  AprilTagDetectionArray pixel_tag_detection;

  BOOST_FOREACH(AprilTags::TagDetection detection, detections){
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end()){
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }
    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);

    Eigen::Matrix3d rott;
    rott = Eigen::AngleAxisd(   Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
                               *Eigen::AngleAxisd(0.0 , Eigen::Vector3d::UnitY())
                               *Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));

    Eigen::Vector4d T(0,0,0,1);
    Eigen::Matrix4d transform_fcu;
    transform_fcu.block<3,3>(0,0) = rott;
    transform_fcu.rightCols<1>() = T;
    transform = transform_fcu*transform;
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

    tag_pose.pose.orientation.x = rot_quaternion.x();
    tag_pose.pose.orientation.y = rot_quaternion.y();
    tag_pose.pose.orientation.z = rot_quaternion.z();
    tag_pose.pose.orientation.w = rot_quaternion.w();

    //aaguiar96 implementation
    AprilTagDetection pixel_detections;
    std::vector<cv::Point2f> input, output;
    input.push_back(cv::Point2f(detection.cxy.first - 10, detection.cxy.second - 10));
    cv::perspectiveTransform(input, output, H);
    pixel_pose.pose.position.x = output[0].x;
    pixel_pose.pose.position.y = output[0].y;
    pixel_pose.pose.position.z = transform(2, 3);;
    pixel_pose.pose.orientation.x = rot_quaternion.x();
    pixel_pose.pose.orientation.y = rot_quaternion.y();
    pixel_pose.pose.orientation.z = rot_quaternion.z();
    pixel_pose.pose.orientation.w = rot_quaternion.w();

    pixel_detections.pose = pixel_pose;
    pixel_detections.id = detection.id;
    pixel_detections.size = tag_size;
    pixel_tag_detection.detections.push_back(pixel_detections);

  }
  tag_publisher.publish(pixel_tag_detection);
  // -----
}


std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id << "__link";
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}


}
