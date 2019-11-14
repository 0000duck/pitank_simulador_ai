#ifndef ROSCONNECTION_HPP
#define ROSCONNECTION_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#endif
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <QThread>
#include <QStringListModel>

/*!
 * - This class initializes and finishes ROS connection
 */

class QNode : public QThread {
  Q_OBJECT

public:
  /*! - class constructor */
  QNode(int argc, char** argv);
  /*! - class destructor, shutdowns ROS */
  virtual ~QNode();

  /*! - initializes the ROS connection using the class contructor parameters */
  bool init();

Q_SIGNALS:
  /*! - signals ROS termination */
  void rosShutdown();

protected:
  int init_argc;
  char** init_argv;
  const std::string node_name;
};

#endif
