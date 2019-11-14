#ifndef TAG_H
#define TAG_H

#include "wall.hpp"
#include "bullet.hpp"
#include "circle.hpp"
#include "pallet.hpp"
#include "racemap.hpp"
#include "../../../game_engine/include/utils/system_info.hpp"

struct Node
{
    int y;
    int x;
    int parentX;
    int parentY;
    float gCost;
    float hCost;
    float fCost;
};

inline bool operator < (const Node& lhs, const Node& rhs)
{//We need to overload "<" to put our struct into a set
    return lhs.fCost < rhs.fCost;
}

/*! - This structure contains the basic information of each robot */

struct robot_info {
  int teamId;

  int tagId;
  cv::Point2d centroid;
  double height, angle;
  char addr0, addr1;
  int vel1, vel2, previous_vel;

  bool collisionFlag, threadIsRunning;
  int collisionStateVar;
  bool autonomous_drive;
  int driving_state;
  cv::Point2f driving_pos;

  bool catch_pallet;
  bool transporting_pallet;
  bool immobilized;

  bool slow_motion;
  int goal_crosses;
  double initial_angle;
  int lap_state;
  int linear_direction;

  int damage, kills;

  QColor robot_color;
};

/*! - This structure is used to associate robots to a team in the pitank game
 *  - If the user doesn't choose the team game mode, each team is composed by a single robot
 */

class team_info {
public:
  team_info(QColor color, cv::Point2f pos, int id);

  QColor team_color;

  std::vector<int> member_id;
  int damage, kills;
  int classification;
  int wareh_pallets;
  int lapsDone;
  cv::Point2f class_pos;
};

extern std::vector<robot_info> robot;
extern std::vector<team_info>  team;
extern int id, driving_counter, number_robots;
extern std::vector<float> linearTransformation, offsetTranformation;
extern cv::Point3d cam_pos;
extern bool raceHasStarted;

/*!
 * This class:
 *    - represents the virtual robot location
 *    - implements Qt functions to draw each tag location on the game map
 */

class tag : public QGraphicsObject {
  Q_OBJECT
public:
  /*!
   * - class contructor
   * - receives the variable msg that contains the robot location, angle, id and size
   * - initializes the robot info and adds it to an array of robot_info structures
   * - creates a timer for each robot that is used to access to the SLOT advance
   */
  tag(apriltags_ros::AprilTagDetection msg, char addr, double robot_z, int team_id);
  tag(geometry_msgs::Pose msg, double robot_z, int team_id, bool &gameS, bool &aiG, ros::Publisher &pub);
  tag(geometry_msgs::Pose msg, double robot_z, int team_id);
  /*! - convertes the argument angle to [0, 360] degrees scale */
  double normDegrees(double angle);

  double normDegrees180(double angle);

  void createThread();

  /*! - defines the bounding rect of each robot, using the robot radius */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - uses the bounding rect to draw a circle around each robot, on the game map */
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  /*! - send a robot to a specified position autonomously in the game map */
  bool send_to_pos(cv::Point2f pos, int id);
  /*! - avoids robot collision at the autonomous drive by planning a trajectory based on artificial potential fields */
  double potential_fields(int id, cv::Point2f target);
  /*! - computes the atan and convertes it to the coordinate system used */
  double arctan(double x, double y);
  /*!  - corrects the parallax displacement of each robot */
  cv::Point2f parallax_correction(double x, double y, double z);
  /*! - delete the Qt item from the scene, safely, without going out of scope */
  void delete_item();

  /*! - function that handles catching a pallet in robot factory game */
  void catch_pallet(int id);
  /*! - function that immobilizes the robot in case of collision with certain Q_OBJECTS */
  void immobilus(int id);
  void lapState(int id);

  bool immobilize;
  int  game;

public Q_SLOTS:
  /*!
   * - function accessed using a periodic timer
   * - updates each robot location on the game map
   * - detects collisions between the robots and the walls
   * - throws a thread for each collision that runs collisionState method
   */
  void advance(int);

private:
  QTimer *timer[max_number_robots];
  std::thread th[max_number_robots];
  std::mutex mx;
  ros::Publisher *velPub;
  QColor color;
  int stop_counter;

  /*!
   * - machine state that holds collisions
   * - updates the state of the collision controlling the joystick controls that can be sent to each robot
   */
  bool collisionState(int id);

  geometry_msgs::Twist vel[2];
  ros::Publisher vel_pub;
  ros::Subscriber uistate_sub, robot_sub;


};

#endif // TAG_H
