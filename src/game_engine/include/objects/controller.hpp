#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#ifndef Q_MOC_RUN
#include <image_transport/image_transport.h>
#endif
#include <thread>
#include <iostream>
#include <map>
#include <algorithm>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <QPainter>
#include <QGraphicsItem>
#include <QtGui>

#include "../graphics/dialog.hpp"

/*!
 * \brief Class that controls all the game engine. It handles:
 *  - which game is going to be played
 *  - the game state (paused, stopped, play, reset, etc)
 *  - the game scores/classifications
 */

class controller {

public:
  /*! - class constructor */
  controller(MainWindow *w, Dialog *dialog);
  /*! - class destructor */
  ~controller();
  /*! - function that throw a thread to the method game_state */
  void throw_thread();
  /*! - state machine that controlls all the game engine (calibration, game states) */
  bool game_state();

  void delete_item();

  MainWindow *gui;
  Dialog *d;

private:
  /*! - sub state machine of game_state that controlls the game in case of the user choose pitank */
  bool pitank_state();
  /*! - function that handles the pitank game scores */
  bool pitank_scores();

  /*! - sub state machine of game_state that controlls the game in case of the user choose robot factory */
  bool robotFactory_state();
  /*! - function that handles the robotf game scores */
  bool robotf_scores();

  /*! - sub state machine of game_state that controlls the game in case of the user choose race car */
  bool raceCar_state();
  /*! - function that handles the race car game scores */
  bool raceCar_scores();

  std::thread *th;
};

#endif // CONTROLLER_HPP
