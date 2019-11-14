#include "../../include/utils/rosConnection.hpp"

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv) {}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc, init_argv, "game_engine");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();
  start();
  return true;
}
