#include <math.h>
#include <string.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "mavros_msgs/RCIn.h"

class PX4Sub {
public:
  tf::StampedTransform global_pose;
  tfScalar roll, pitch, yaw;

  PX4Sub(ros::NodeHandle ROS_n) {
    _px4_sub = ROS_n.subscribe("/mavros/local_position/pose", 1, &PX4Sub::px4callback, this);
  }

  void px4callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

    tf::Quaternion q_base_link(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m_base_link(q_base_link);

    m_base_link.getRPY(roll, pitch, yaw);
    tf::Quaternion q_base_stabilized;
    q_base_stabilized.setRPY(0.0, 0.0, yaw);

    transform.setRotation(q_base_link);
    _tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

    transform.setRotation(q_base_stabilized);
    _tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_stabilized"));
  }

private:
  ros::Subscriber _px4_sub;
  tf::TransformBroadcaster _tf_br;
};

class ObstacleSub {
public:
  tf::Vector3 obs_center;
  float obs_radius;
  bool received_obs;

  ObstacleSub(ros::NodeHandle ROS_n) {
    received_obs = false;
    _obs_sub = ROS_n.subscribe("/stereo_avoid/clusters/marker0", 1, &ObstacleSub::obscallback, this);
  }

  void obscallback(const visualization_msgs::Marker::ConstPtr& msg) {
    received_obs = true;
    obs_center[0] = msg->pose.position.x;
    obs_center[1] = msg->pose.position.y;
    obs_center[2] = msg->pose.position.z;
    obs_radius = msg->scale.x;
  }
private:
  ros::Subscriber _obs_sub;
};

class RCSub {
public:
  tf::Vector3 get_setpoint_accel() {
    tf::Vector3 sp_vel;
    sp_vel[0] = -1.0 * (rc_channels[1] - 1500.0)/500.0;
    sp_vel[1] = -1.0 * (rc_channels[0] - 1500.0)/500.0;
    sp_vel[2] = (rc_channels[2] - 1500.0)/500.0;
    return sp_vel;
  }

  RCSub(ros::NodeHandle ROS_n) {
    for (int i = 0; i<8; i++) {
      rc_channels[i] = 1500.0;
    }
    _rc_sub = ROS_n.subscribe("/mavros/rc/in", 1, &RCSub::rcincallback, this);
  }

  void rcincallback(const mavros_msgs::RCIn::ConstPtr& msg) {
    for (int i = 0; i<8; i++) {
      rc_channels[i] = msg->channels[i];
    }
  }

private:
  ros::Subscriber _rc_sub;
  float rc_channels[8];
};

int main(int argc, char **argv)
{
  float sp_accel_max = 4;
  float out_accel_max = 4;
  
  ros::init(argc, argv, "px4accelctrl");
  ros::NodeHandle _n;

  // Gets messages from mavros and publishes them as transforms
  PX4Sub px4_subscription(_n);
  // Gets messages from the artoo sticks (through mavros)
  RCSub rc_subscription(_n);

  // Sends commands back to px4
  ros::Publisher _px4_pub;
  _px4_pub = _n.advertise<geometry_msgs::Vector3Stamped>("/mavros/setpoint_accel/accel",1);
  
  // runs the control loop at 100Hz
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    tf::Vector3 setpoint_accel;
    setpoint_accel = rc_subscription.get_setpoint_accel();
    for (int i=0; i<3; i++) {
      setpoint_accel[i] *= sp_accel_max;
    }

    // send the desired acceleration
    geometry_msgs::Vector3Stamped cmd_accel;
    cmd_accel.vector.x = setpoint_accel.getX();
    cmd_accel.vector.y = setpoint_accel.getY();
    cmd_accel.vector.z = setpoint_accel.getZ();
    cmd_accel.header.frame_id = "fcu";
    _px4_pub.publish(cmd_accel);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
