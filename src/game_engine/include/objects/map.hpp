#ifndef MAP_H
#define MAP_H

#ifndef Q_MOC_RUN
#include <image_transport/image_transport.h>
#endif
#include <thread>
#include <mutex>
#include <sstream>
#include <iostream>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <QPainter>
#include <QGraphicsItem>
#include <QtGui>

#include "../utils/system_info.hpp"

/*! This class:
 *    - represents the map area of the game
 *    - contains the importante info about the map (corners, centroid, bounding rect)
 *    - contains the corner detection image and the actual frame received by the calibration node before searching the corners
 */

extern bool gameRunning, gamePaused;

class map {

public:
  /*! - class constructor */
  map();

  /*!
   * - receives a binary image with the map area highlighted and finds the corners and the centroid of the map
   * - draws an image with the contour, the corners and the centroid of the map to show the calibration
   * - finds the shortest rectangle where the map fits, to be used to perform a scalar transformation later
   */
  bool performCalibration(sensor_msgs::ImageConstPtr input_img);
  bool new_corners(std::vector<cv::Point2f> new_corners);

  cv::Mat calib_image, Tr;
  std_msgs::Float32MultiArray cornersToPublish;
  std::vector<cv::Point2f> corners, sceneCorners;
  std::vector<float> linearTransf, offsetTransf;
  cv::Mat H;
  int max_x, max_y, min_x, min_y;

private:
  cv::Mat image;
  std::vector<cv::Point2f> contour;
  cv::Point2f centroid;
  QTimer *timer;
};

#endif // MAP_H
