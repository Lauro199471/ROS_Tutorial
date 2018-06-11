/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef LISTENER_NODE_HPP_
#define LISTENER_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "qnode.hpp"
#endif

#include <sensor_msgs/Joy.h>
#include <string>

/*****************************************************************************
** Class
*****************************************************************************/

class Listener : public QNode {

public:
  Listener(int argc, char** argv);
  virtual ~Listener() {}
  void run();
  void ros_comms_init();
private:
  void chatterCallback(const sensor_msgs::Joy &msg);
  ros::Subscriber chatter_subscriber;
};

double getServo0(void);
double getServo1(void);
bool get_A_Status(void);
bool get_B_Status(void);
bool get_X_Status(void);
bool get_Y_Status(void);

#endif /* LISTENER_NODE_HPP_ */
