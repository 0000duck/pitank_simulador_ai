#ifndef DIALOG_HPP
#define DIALOG_HPP

#ifndef Q_MOC_RUN
#include "../utils/rosConnection.hpp"
#endif
#include </usr/include/SerialStream.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <QDesktopWidget>
#include "../objects/grid.hpp"
#include "../objects/tag.hpp"
#include "../objects/map.hpp"
#include "../objects/wall.hpp"
#include "../objects/circle.hpp"
#include "../objects/machine.hpp"
#include "../objects/pallet.hpp"
#include "../objects/person.hpp"
#include "../objects/racemap.hpp"
#include "../objects/game_clock.hpp"
#include "../graphics/main_window.hpp"
#include "game_engine/RobotDescriptionArray.h"
#include "game_engine/UIState.h"
#include "game_engine/ShootAndTurbo.h"
#include "game_engine/WallInfoArray.h"
#include <ctime>
#include <pthread.h>
#include <iostream>
#include <fstream>

using namespace LibSerial;

extern SerialStream my_serial_stream, my_serial_port;

namespace Ui {
class Dialog;
}

struct uistate{
    bool setupStart;
    bool gameStart;
    bool teamGame;
    bool paused;
    bool aiGame;
    /*if(clk != NULL)
        ui_state.seconds = clk->getTime().minute()*60 + clk->getTime().second();
    else*/
    //ui_state.seconds = -1;
};

/*! This class vides:
 *    - communication with the main window (user interface)
 *    - game window (map)
 *    - joysticks & xBee communication
 *    - communication with the apriltags package
 *    - communication with the calibrator of the game
 */

class Dialog : public QDialog {
    Q_OBJECT

public:
  /*! - class contructor that subscribes and advertises ROS nodes
   *  - initializes map class
   *  - creates the game window scene
   */
  explicit Dialog(int argc, char **argv, QWidget *parent = 0);
  /*! - class destructor */
  ~Dialog();

  /*! - function called at the beggining of each game that resets the previous game info if there was one */
  bool reset_game();
  /*! - auxiliar funtion that the controller uses to emit a signal for the main window */
  void orderToGui(bool calib);
  void paintGoals();

  char addr[max_number_robots];
  double robot_z[max_number_robots];
  int tag_id[max_number_robots];
  std::map<int, char> tagAndXbee;
  bool init_flag, finish_flag, pause_flag;


  bool asdf_flag = true;


  QGraphicsScene scene;
  std::map<int, int> tagAndRobot;
  uistate uis;

  map *M;
  tag *t;
  int init_argc;
  char** init_argv;
  Ui::Dialog *ui;
  QTime time;
  bool flag;

public Q_SLOTS:
  /*! - collects the thresold choosed by the user and send it to the calibrator */
  void calibrateSIGNAL(int64_t);
  /*! - publishes info of the calibration to apriltags package (corners of the map) */
  void endCalibrationSIGNAL();
  /*! - implements ros::spinOnce() if the game has started */
  void startSIGNAL();
  void manually_calibration(std::vector<cv::Point2f>);
  void updateWallMessage(int id);

Q_SIGNALS:
  /*! - tells the main window that a calibration was finished so that it can be showed */
  void calibDone(bool, cv::Mat, std::vector<cv::Point2f>);
  /*! signals the gui that a game has ended so its necessary to reset the game buttons */
  void reset_buttons();
  /*! signal that tells the gui which robots where detected at the game setup */
  void detected_robots(std::map<int, int>);

private:
  /*! - callback function that receives apriltags coordinates and processes it */
  void listener(const apriltags_ros::AprilTagDetectionArray &msg);
  void sim_listener(const gazebo_msgs::ModelStates &msg);
  /*! - creates a trama to send to the wifi module to activate robots motors */
  int createTrama(char addr0, char addr1, char data1, char data2, char options);
  /*! - processes joystick A commands and send the trama to the xBee */
  void velJoystickA(geometry_msgs::Twist speed);
  /*! - processes joystick B commands and send the trama to the xBee */
  void velJoystickB(geometry_msgs::Twist speed);
  /*! - processes joystick C commands and send the trama to the xBee */
  void velJoystickC(geometry_msgs::Twist speed);
  /*! - processes joystick D commands and send the trama to the xBee */
  void velJoystickD(geometry_msgs::Twist speed);

  /*! - function that initializes the robot factory game -> all the objects and arrays needed are declared here */
  void initialize_robotf(const apriltags_ros::AprilTagDetectionArray &msg);
  /*! - function called at the beggining of the Robot factory game, that draws the game map on the scene */
  void drawRobotFactoryMap();

  /*! - function called before the calibration that draws the border of the map */
  void initMap();

  /*! - function that initializes the pitank game  -> all the objects and arrays needed are declared here */
  void initialize_pitank(const apriltags_ros::AprilTagDetectionArray &msg);
  void initialize_pitank_sim(const gazebo_msgs::ModelStates &msg);
  /*!
   * - function that ends the pitank & robot @ factory game
   * - draws a podium and sets teams ranking on it
   */
  void finish_game();
  /*! - function called at the beggining of the Pi tank game, that draws the game map on the scene */
  void drawPitankMap();

  /*! - function called at the beggining of the Race car game */
  void initialize_raceCar(const apriltags_ros::AprilTagDetectionArray &msg);
  /*! - */
  void finish_raceCar();

  void convertRobotToMessage();

  void publish_ui_state();

  void uistate_init();

  void shootAndTurboA(const game_engine::ShootAndTurbo &msg);
  void shootAndTurboB(const game_engine::ShootAndTurbo &msg);
  void shootAndTurboC(const game_engine::ShootAndTurbo &msg);
  void shootAndTurboD(const game_engine::ShootAndTurbo &msg);

  void convertWallToMessage(wall *w);

  QGraphicsView* map_view;
  QPixmap *map_pix;
  QTimer *timer;
  QNode qnode;
  QElapsedTimer bullet_interval[max_number_robots];

  ros::NodeHandle *nh;
  ros::Subscriber pose_sub, pose_sim_sub, calib_sub, controls_sub[max_number_joysticks],shootandturbo_sub[max_number_joysticks];
  ros::Publisher thresh_pub, corners_pub, gameStarted_pub, robotDescription_pub, uiState_pub, wallInfo_pub, controls_pub[max_number_joysticks];

  clock_t timer_sim;
  gazebo_msgs::SpawnModel model;
  gazebo_msgs::DeleteModel model_to_delete;
  geometry_msgs::Pose init_pose;
  ros::ServiceClient spawn;
  ros::ServiceClient delete_model;
  float init_xy[4][2];
  geometry_msgs::Twist twist[4];

  grid *gr;
  wall *w;
  circle *c;
  bullet *b;
  machine *mc;
  pallet *p;
  person *h;
  racemap *rc;
  game_timer *clk;
  cv::Mat game_map;
  std::map<int,int>::iterator it;
  sensor_msgs::ImageConstPtr input_img;
  std_msgs::Bool gameHasStarted;
  char trama[15];
  bool team_game, game_end;
  game_engine::RobotDescription r_description;
  game_engine::RobotDescriptionArray r_array;
  game_engine::UIState ui_state;
  game_engine::ShootAndTurbo shootandturbo[max_number_joysticks];
  game_engine::WallInfoArray w_array;
  game_engine::WallInfo w_info;
};

#endif // DIALOG_HPP
