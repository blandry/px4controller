#include <math.h>
#include "ros/ros.h"
#include "mavros_msgs/RCIn.h"

void rcincallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
  ROS_INFO("%.8f",(double)msg->channels[0]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rcin");

  ros::NodeHandle _n;

  ros::Subscriber _rc_sub = _n.subscribe("/mavros/rc/in", 1, rcincallback);

  ros::spin();

  return 0;
}
