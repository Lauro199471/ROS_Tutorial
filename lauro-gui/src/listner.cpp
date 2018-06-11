#include <ros/ros.h>
#include <string>
#include <sstream>
#include "../include/listner.hpp"
#include <sensor_msgs/Joy.h>
#include <QDebug>

double servo0_pos = 0 , servo1_pos= 0;

bool a_button = FALSE ,
     b_button = FALSE ,
     x_button = FALSE ,
     y_button = FALSE;
/*****************************************************************************
** Implementation
*****************************************************************************/

Listener::Listener(int argc, char** argv ) :
  QNode(argc,argv,"qlistenerWORKS")
  {}

void Listener::ros_comms_init() {
  ros::NodeHandle n;
  chatter_subscriber = n.subscribe("joy", 1000, &Listener::chatterCallback, this);
}

void Listener::chatterCallback(const sensor_msgs::Joy &msg) {
   servo0_pos = (msg.axes[5] - 1.0) * -90; // fit -2~0 to 0~180 not 0-2 to 180 ~ 0
   servo1_pos = (msg.axes[2] - 1.0) * -90; // fit -2~0 to 0~180 not 0-2 to 180 ~ 0
   a_button = msg.buttons[0];
   b_button = msg.buttons[1];
   x_button = msg.buttons[2];
   y_button = msg.buttons[3];
}

void Listener::run() {
  ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

double getServo0(void){return servo0_pos;}
double getServo1(void){return servo1_pos;}
bool get_A_Status(void){return a_button;}
bool get_B_Status(void){return b_button;}
bool get_X_Status(void){return x_button;}
bool get_Y_Status(void){return y_button;}
