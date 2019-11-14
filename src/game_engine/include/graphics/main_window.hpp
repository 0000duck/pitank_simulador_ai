#ifndef game_engine_MAIN_WINDOW_H
#define game_engine_MAIN_WINDOW_H

#include <iostream>
#include <sstream>
#include <QMessageBox>
#include <QMainWindow>
#include <QtGui>
#include <QApplication>
#include <QtCore>
#include <QDialog>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ui_main_window.h"

extern bool setupStart, gameStart, teamGame, paused, aiGame, simGame;
extern int  game, state, pitank_gamestate, robotf_gamestate, raceCar_gamestate, robotn_sim;
extern QTime game_clock;

/*! This class:
 *    - provides user interface
 *    - tells ROS when to start
 */

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  /*!
   * - class constructor
   * - connects SIGNALS and SLOTS
   */
  MainWindow(QDialog *dialog, QWidget *parent = 0);
  /*! - class destructor */
  ~MainWindow();
  void showNoMasterMessage();

  Ui::MainWindowDesign ui;
  bool calibrated, paused;
  QString game_option[3];
  int idA, idB, idC, idD, idE, idF;
  std::vector<cv::Point2f> corners;

public Q_SLOTS:
  /*! - tells Dialog class that the user wants to try a calibration, sending calibrate SIGNAL */
  void on_calibrate_clicked();
  /*! - active a flag that tells that the game has started when the button Start is clicked*/
  void on_start_clicked();
  /*! - tells Dialog class that the user has changed the thresold value */
  void on_threshold_valueChanged(int);
  /*! - slot that signals that the user wants to pause the game */
  void on_pause_clicked();
  /*! - slot that signals that the user wants to stop the game */
  void on_stop_clicked();
  /*! - slot that signals that the user wants to prepare the game */
  void on_setup_game_clicked();
  /*! - slot that signals that the user update the game time */
  void canStart(bool, cv::Mat, std::vector<cv::Point2f>);
  /*! - update the game timer */
  void update_time();
  /*! - slot for the signal reset_buttons */
  void reset_buttons_slot();
  /*! - slot for the signal detected_robots */
  void detected_robots_slot(std::map<int, int>);
  /*! - slot to toggle simulation button */
  void on_game_choose_currentIndexChanged(int);

  void on_sim_bool_stateChanged(int);

  void on_c1_x_valueChanged(int c1_x);
  void on_c1_y_valueChanged(int c1_y);
  void on_c2_x_valueChanged(int c2_x);
  void on_c2_y_valueChanged(int c2_y);
  void on_c3_x_valueChanged(int c3_x);
  void on_c3_y_valueChanged(int c3_y);
  void on_c4_x_valueChanged(int c4_x);
  void on_c4_y_valueChanged(int c4_y);

Q_SIGNALS:
  /*! - sends the thresold choosed to the Dialog class */
  void tryCalibration(int64_t);
  void manually_calibrate(std::vector<cv::Point2f>);
  void calibrate();
  /*! - a SIGNAL that tells ROS to start */
  void startROS();
  /*! - tells Dialog class to put the game map on fullscreen mode */
  void setFullscreen();


private:
};

#endif
