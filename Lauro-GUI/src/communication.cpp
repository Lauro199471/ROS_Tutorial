#include <ros/ros.h>
#include <string>
#include "../include/communication.hpp"

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/common_vars.hpp"


void usernameCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]\t", msg->data.c_str());
  received_string = msg->data.c_str();
  std::cout << received_string << std::endl;
}

void userCountCallback(const std_msgs::Int32& msg)
{
  count = 0;
  std::cout << msg.data << std::endl;
  user_count_val = msg.data;
}

Communication::Communication(int argc, char** argv ) :
  QNode(argc,argv,"lauro_node")
  {}

void Communication::ros_comms_init() {

    ros::NodeHandle n;

    chatter_publisher = n.advertise<std_msgs::Int32>("/counter", 1000);
    chatter_listener = n.subscribe("/stringData", 1000, usernameCallback);
    listern_for_user_input = n.subscribe("/userInput", 1000, userCountCallback);

}

void Communication::run() {
  ros::Rate loop_rate(1000);
  while ( ros::ok() ) {
    std_msgs::Int32 msg;
    msg.data = count;
    chatter_publisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
