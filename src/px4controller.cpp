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

  PX4Sub(ros::NodeHandle ROS_n) {
    _px4_sub = ROS_n.subscribe("/mavros/local_position/pose", 1, &PX4Sub::px4callback, this);
  }

  void px4callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

    tf::Quaternion q_base_link(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m_base_link(q_base_link);

    tfScalar roll, pitch, yaw;
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
  tf::Vector3 get_setpoint_vel() {
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
  float obs_buffer = .5;
  float obs_vel_scale = 2;
  
  float sp_vel_max = 2;
  float out_vel_max = 3;

  ros::init(argc, argv, "px4controller");
  ros::NodeHandle _n;

  // Gets messages from mavros and publishes them as transforms
  PX4Sub px4_subscription(_n);
  // Gets the obstacles from computer vision
  ObstacleSub obstacle_subscription(_n);
  // Gets messages from the artoo sticks (through mavros)
  RCSub rc_subscription(_n);

  // Sends commands back to px4
  ros::Publisher _px4_pub;
  _px4_pub = _n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);
  
  // runs the control loop at 100Hz
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    tf::Vector3 setpoint_vel;
    setpoint_vel = rc_subscription.get_setpoint_vel();
    for (int i=0; i<3; i++) {
      setpoint_vel[i] *= sp_vel_max;
    }
    
    tf::Vector3 delta_vel;
    delta_vel.setZero();
    if (obstacle_subscription.received_obs) {
      // compute delta vel from obstacle
      
      delta_vel = px4_subscription.global_pose.getOrigin() - obstacle_subscription.obs_center;
      // ignore the z component, obstacles are infinite
      delta_vel[2] = 0;

      // make the velocity be zero a buffer distance away from the obstacle
      float dist = delta_vel.length() - obstacle_subscription.obs_radius - obs_buffer;
      if (dist > 1) {
        delta_vel.setZero();
      } else {
        if (delta_vel.length()>0) {
          delta_vel /= delta_vel.length();
          delta_vel *= obs_vel_scale * setpoint_vel.length() / (1+dist);
        }
      }
    }

    // compute the demanded velocity
    tf::Vector3 out_vel = setpoint_vel + delta_vel;
    if (out_vel.length()>out_vel_max && out_vel.length()>0) {
      out_vel /= out_vel.length();
      out_vel *= out_vel_max;
    }
    
    // add delta vel to setpoint_vel
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.twist.linear.x = out_vel.getX();
    cmd_vel.twist.linear.y = out_vel.getY();
    cmd_vel.twist.linear.z = out_vel.getZ();
    cmd_vel.header.frame_id = "fcu";
    _px4_pub.publish(cmd_vel);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
