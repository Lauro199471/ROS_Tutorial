#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP


#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "../include/qnode.hpp"
#endif

#include <string>

/*****************************************************************************
** Class
*****************************************************************************/

class Communication : public QNode {

public:
  Communication(int argc, char** argv);
  virtual ~Communication() {}
  void run();
  void ros_comms_init();
private:
  ros::Publisher chatter_publisher;
  ros::Subscriber chatter_listener;
  ros::Subscriber listern_for_user_input;

};

#endif // COMMUNICATION_HPP
