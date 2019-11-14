#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <array>
#include <vector>
#include <stack>
#include <geometry_msgs/Twist.h>
#include "game_engine/RobotDescription.h"
#include "game_engine/RobotDescriptionArray.h"
#include "game_engine/UIState.h"
#include "game_engine/ShootAndTurbo.h"
#include "game_engine/WallInfoArray.h"
#include "../../../game_engine/include/utils/system_info.hpp"
//#include "../../../game_engine/include/objects/machine.hpp"
//#include "../../../game_engine/include/objects/point.hpp"
#include <QPointF>
//#include <QColor>
//#include <QPainter>
#include <stdlib.h>
#include <time.h>

#include <math.h>
using namespace std;

#define ROTATE 0
#define TRANSL 1
#define ROT_ENEMY 2
#define END 3
#define MAX_ETF 5
#define HIST_ETF 5
#define ERRO_DIST 5
#define GAIN_FWD 20
#define GAP 40
#define GAP2 37

#define MOVE 0
#define ROTATE_TO_ENEMY 1

#define X_STEP SPACE
#define Y_STEP SPACE

#define HIST_COST_DEF 2
#define DEPTH_NEGAMAX 2

#define PERIOD 3
#define COST1CELL 1.25

#define SPEED_BITS 127

#define WINNING 1
#define LOSING 0
#define HIDDEN 1
#define VISIBLE 0

#define pdd pair<double, double>
#define w_id_inter pair<int,pdd>

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

struct PlayerPosition
{
    int x;
    int y;
    float angle;
};

inline bool operator < (const Node& lhs, const Node& rhs)
{//We need to overload "<" to put our struct into a set
    return lhs.fCost < rhs.fCost;
}

const int sign[2]={1,-1};

int flag_test = 0;

int max_cells;
geometry_msgs::Twist vel[2];
game_engine::ShootAndTurbo shootandturbo[4];
game_engine::WallInfo w_info;
game_engine::WallInfoArray w_array;
ros::Publisher vel_pub[4], shootandturbo_pub[4];
ros::Subscriber uistate_sub, robot_sub, wall_sub;
//pthread_t t[2];
int id = 0;
bool asas = false;
bool end_gotoxy = false;
int state_gotoxy = ROTATE;
int flag = false;
int play =-1;

int map_astar_size_x = (projectedImageSizeX-borderOffset_x)/SPACE + 1;
int map_astar_size_y = (projectedImageSizeY)/SPACE + 1;
bool map_astar[(projectedImageSizeX-borderOffset_x)/SPACE + 1][(projectedImageSizeY)/SPACE + 1];

struct robot_info {
  int teamId;
  int tagId;
  int x;
  int y;
  double height, angle;
  char addr0, addr1;
  int vel1, vel2, previous_vel;
  bool collisionFlag, threadIsRunning;
  int collisionStateVar;
  bool autonomous_drive;
  bool immobilized;
  int damage, kills, classification;
};

robot_info r;

struct ui_state {
  bool setupStart, gameStart, teamGame, paused, aiGame, simGame;
  int seconds;
};

std::vector<robot_info> robot;
ui_state uistate;
vector<Node>aux_node_path;
vector<Node>aux_node_path_min;
Node player;
Node enemy;

void move_forward(int id, double error_theta);
void move_back(int id);
void move_right(int id);
void move_left(int id);
void stop(int id);
void shoot(int id);
void turbo(int id);
void idle();

void create_map_astar(int space);
static bool isValid(int x, int y);
static bool isDestination(int x, int y, Node dest);
static double calculateH(int x, int y, Node dest);
static vector<Node> aStar(Node player, Node dest);
static vector<Node> makePath(array<array<Node, (projectedImageSizeY)/SPACE + 1>, ((projectedImageSizeX-borderOffset_x)/SPACE + 1)> map, Node dest);
void printNodes(vector<Node> vec);
int gridXtoPixel(int num);
int gridYtoPixel(int num);
int pixelXtoGrid(int num);
int pixelYtoGrid(int num);
QPointF *convertToGrid(int x, int y);
void followPath(vector<Node> vec);
void decisionMaking();
void moveToRandomPosition(int id);
void rotateToEnemy(int id, int x_e, int y_e);
void testDestroyWall(int id);
bool checkTrajectoryCollision(int p0_x, int p0_y, int p1_x, int p1_y, int p2_x, int p2_y, int p3_x, int p3_y);
bool onSegment(QPointF *p, QPointF *q, QPointF *r);
int orientation(QPointF *p, QPointF *q, QPointF *r);
bool doIntersect(QPointF *p1, QPointF *q1);
bool doIntersect2(QPointF *p1, QPointF *q1);
float distanceTwoPoints(Node a, Node b);
float costTimeOfPath(int id, vector<Node> path);
vector<Node> bestPointToAttack(int id_player, QPointF *p2);
vector<Node> bestPointToDefend(int id_player, int id_enemy);
void moveToBestAttackPosition(int id_player, int id_enemy);
void moveToBestDefensePosition(int id_player, int id_enemy);
int findBestWalls(int id);
bool checkBehindWallCluster(QPointF *p1, QPointF *q1, int wall_cluster);
bool attackOrDefense(int id_player, int id_enemy);
float evaluation(int player, Node player_pos, Node enemy_pos, Node pos_init);
float evaluation2(int player, Node player_pos, Node enemy_pos, Node pos_init);
float negaMax(int depth, int player, Node player_pos, Node enemy_pos, Node pos_init);
float negaMax2(int depth, int player, Node player_pos, Node enemy_pos, Node pos_init, float cost);
int winning(int id);
pdd lineLineIntersection(pdd A, pdd B, pdd C, pdd D);
float negaMax3(int depth, int player, Node player_pos, Node enemy_pos);
float evaluation3(int player, Node player_pos, Node enemy_pos);
int distToWallCenter(pdd p, int wall_id);
int distToWallVertice(pdd p, int wall_id, int player_id);
int distToWallVertice2(pdd p, int wall_id, PlayerPosition player);
vector<w_id_inter> wallsBetweenPlayers(Node player_pos, Node enemy_pos);
PlayerPosition closestVertice(PlayerPosition p, int wall_id);
float evaluation4(int player, PlayerPosition player_pos, PlayerPosition enemy_pos);
float negaMax4(int depth, int player, PlayerPosition player_pos, PlayerPosition enemy_pos);
void testecenas(Node player, Node dest, int flag_teste);

double diffAngle(double x, double y);

#endif
