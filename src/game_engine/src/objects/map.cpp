#include "../../include/objects/map.hpp"

bool gameRunning, gamePaused;

map::map() {}

bool map::performCalibration(sensor_msgs::ImageConstPtr input_img) {
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contourTemp;
  double xx = 0, yy = 0, max_1 = 0, max_2 = 0, max_3 = 0, max_4 = 0;
  cv::Point2f corner_1, corner_2, corner_3, corner_4;

  contour.clear();
  corners.clear();
  linearTransf.clear();

  //find contour of the map

  image = cv_bridge::toCvCopy(input_img, sensor_msgs::image_encodings::MONO8)->image;
  cv::findContours(image, contourTemp, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1, cv::Point(0,0));

  std::vector<cv::Moments> mu(contourTemp.size());
  for(int i = 0; i < contourTemp.size(); i++)
    mu[i] = cv::moments(contourTemp[i], false);

  if(contourTemp.size() <= 0)
    return false;
  else {
    for(size_t i = 0; i < contourTemp.size(); i++) {
      for(size_t j = 0; j < contourTemp[i].size(); j++) {
        if(contourTemp[i][j].x > 10 && contourTemp[i][j].y > 10)
          contour.push_back(contourTemp[i][j]);
      }
    }

    sensor_msgs::CompressedImageConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CompressedImage>("/image_raw/compressed");
    calib_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

    //find centroid & corners of the map

    cv::Point2f COG(mu[0].m10/mu[0].m00 , mu[0].m01/mu[0].m00);
    centroid = COG;

    for(size_t i = 0; i < contour.size(); i++) {
      if(contour[i].x < COG.x && contour[i].y < COG.y) {
        if(max_1 < norm(COG - contour[i])) {
          corner_1 = contour[i];
          max_1 = norm(COG - contour[i]);
        }
      }
      else if(contour[i].x < COG.x && contour[i].y > COG.y) {
        if(max_2 < norm(COG - contour[i])) {
          corner_2 = contour[i];
          max_2 = norm(COG - contour[i]);
        }
      }
      else if(contour[i].x > COG.x && contour[i].y < COG.y) {
        if(max_3 < norm(COG - contour[i])) {
          corner_3 = contour[i];
          max_3 = norm(COG - contour[i]);
        }
      }
      else if(contour[i].x > COG.x && contour[i].y > COG.y) {
        if(max_4 < norm(COG - contour[i])) {
          corner_4 = contour[i];
          max_4 = norm(COG - contour[i]);
        }
      }
    }

    corners.push_back(corner_1);
    corners.push_back(corner_2);
    corners.push_back(corner_3);
    corners.push_back(corner_4);

    cv::circle(calib_image, COG, 5, cv::Scalar(255,255,255), 2, 8, 0);
    cv::circle(calib_image, corner_1, 5, cv::Scalar(255,255,255), 2, 8, 0);
    cv::circle(calib_image, corner_2, 5, cv::Scalar(255,255,255), 2, 8, 0);
    cv::circle(calib_image, corner_3, 5, cv::Scalar(255,255,255), 2, 8, 0);
    cv::circle(calib_image, corner_4, 5, cv::Scalar(255,255,255), 2, 8, 0);
  }

  /* finds the shortest rect where the map fits */

  max_x = 0; max_y = 0; min_x = 1500; min_y = 1500;
  for(size_t i = 0; i < corners.size(); i++) {
    if(corners[i].x > max_x)
      max_x = corners[i].x;
    if(corners[i].x < min_x)
      min_x = corners[i].x;
  }

  for(size_t j = 0; j < corners.size(); j++) {
    if(corners[j].y > max_y)
      max_y = corners[j].y;
    if(corners[j].y < min_y)
      min_y = corners[j].y;
  }


  float linearTx = (float)(projectedImageSizeX - wallWidth -  borderOffset_x)/(max_x - min_x), linearTy = (float)(projectedImageSizeY - wallWidth)/(max_y - min_y);
  float offsetTx = 2 * offset + borderOffset_x,  offsetTy = 2 * offset;
  linearTransf.push_back(linearTx);
  linearTransf.push_back(linearTy);
  offsetTransf.push_back(offsetTx);
  offsetTransf.push_back(offsetTy);

  return true;
}

bool map::new_corners(std::vector<cv::Point2f> new_corners) {
  corners.clear();
  linearTransf.clear();
  offsetTransf.clear();

  corners = new_corners;

  sensor_msgs::CompressedImageConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CompressedImage>("/image_raw/compressed");
  calib_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

  /* finds the shortest rect where the map fits */

  max_x = 0; max_y = 0; min_x = 1500; min_y = 1500;
  for(size_t i = 0; i < corners.size(); i++) {
    if(corners[i].x > max_x)
      max_x = corners[i].x;
    if(corners[i].x < min_x)
      min_x = corners[i].x;
  }

  for(size_t j = 0; j < corners.size(); j++) {
    if(corners[j].y > max_y)
      max_y = corners[j].y;
    if(corners[j].y < min_y)
      min_y = corners[j].y;
  }

  cv::circle(calib_image, corners[0], 5, cv::Scalar(255,255,255), 2, 8, 0);
  cv::circle(calib_image, corners[1], 5, cv::Scalar(255,255,255), 2, 8, 0);
  cv::circle(calib_image, corners[2], 5, cv::Scalar(255,255,255), 2, 8, 0);
  cv::circle(calib_image, corners[3], 5, cv::Scalar(255,255,255), 2, 8, 0);

  float linearTx = (float)(projectedImageSizeX - wallWidth -  borderOffset_x)/(max_x - min_x), linearTy = (float)(projectedImageSizeY - wallWidth)/(max_y - min_y);
  float offsetTx = 2 * offset + borderOffset_x,  offsetTy = 2 * offset;
  linearTransf.push_back(linearTx);
  linearTransf.push_back(linearTy);
  offsetTransf.push_back(offsetTx);
  offsetTransf.push_back(offsetTy);
}
