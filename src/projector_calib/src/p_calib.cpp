#include "projector_calib/p_calib.h"

void calibrator::read_image(const sensor_msgs::CompressedImageConstPtr& msg) {
  cv::Mat image;
  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); // ROS image ---> OPENCV grayscale image
    image = cv_ptr->image;
  }
  catch(cv_bridge::Exception e) {
    ROS_ERROR("Cv Bridge exeption: %s.", e.what());
  }

  if(!find_map(thresh(image))) {
    cv_ptr->image = final;
    pub_.publish(cv_ptr->toImageMsg());
  }
  pub_.publish(cv_ptr->toImageMsg());
}

void calibrator::thresh_callback(const std_msgs::Int64ConstPtr& value) {
  thresh_value = value->data;

  sensor_msgs::CompressedImageConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CompressedImage>("/image_raw/compressed");
  read_image(msg);
}

Mat calibrator::thresh(Mat image) {
  Mat thresh_img;
  int max_val = 255, option = 1; // option = 1 -> THRESH_BINARY_INV
  threshold(image, thresh_img, thresh_value, max_val, option);

  return thresh_img;
}

bool calibrator::find_map(Mat image) {
  Mat floodfill_img = image.clone();
  Size s = image.size();
  int filled_vals = 255;

  floodFill(floodfill_img, Point(round(s.width/2), round(s.height/2)), filled_vals);
  final = floodfill_img - image;

  if(!countNonZero(final))
    return 1;
  return 0;
}

