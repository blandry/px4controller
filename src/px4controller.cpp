#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "dwa_local_planner/dwa_planner_ros.h"

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
    transform.setRotation(q_base_stabilized);
    _tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
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

  ObstacleSub(ros::NodeHandle ROS_n) {
    _obs_sub = ROS_n.subscribe("/stereo_avoid/clusters/marker0", 1, &ObstacleSub::obscallback, this);
  }

  void obscallback(const visualization_msgs::Marker::ConstPtr& msg) {
    obs_center[0] = msg->pose.position.x;
    obs_center[1] = msg->pose.position.y;
    obs_center[2] = msg->pose.position.z;
    obs_radius = msg->scale.x;
  }
private:
  ros::Subscriber _obs_sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "px4controller");

  ros::NodeHandle _n;

  // // Gets messages from mavros and publishes them as transforms
  PX4Sub px4_subscription(_n);

  // Sends commands back to px4
  ros::Publisher _px4_pub;
  _px4_pub = _n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);

  // Occupancy grid filled up by point cloud
  // tf::TransformListener tf;
  // costmap_2d::Costmap2DROS costmap("costmap", tf);
  // std::vector<geometry_msgs::Point> footprint(4);
  // geometry_msgs::Point footp1;
  // footp1.x = -0.30; footp1.y = 0.30;
  // geometry_msgs::Point footp2;
  // footp2.x = -0.30; footp2.y = 0.30;
  // geometry_msgs::Point footp3;
  // footp3.x = 0.30; footp3.y = 0.30;
  // geometry_msgs::Point footp4;
  // footp4.x = 0.30; footp4.y = -0.30;
  // footprint[0] = footp1;
  // footprint[1] = footp2;
  // footprint[2] = footp3;
  // footprint[3] = footp4;
  // costmap.setUnpaddedRobotFootprint(footprint);

  // Runs an MPC style planner (dynamic window approach)
  // dwa_local_planner::DWAPlannerROS planner;
  // planner.initialize("dwa_planner", &tf, &costmap);

  // Setting the goal
  // geometry_msgs::PoseStamped goal_pose;
  // goal_pose.pose.position.x = -2;
  // goal_pose.pose.position.y = -2;
  // goal_pose.header.frame_id = "map";
  // std::vector<geometry_msgs::PoseStamped> goal_plan(1);
  // goal_plan[0] = goal_pose;

  // planner.setPlan(goal_plan);

  tf::Vector3 setpoint_vel(.25, 0, 0);
  float obs_buffer = .5;
  float out_vel_max = 1;

  float obs_vel_scale = 2;
  // float obs_radius = 1;
  // tf::Vector3 obs_center(100, 100, 0);
  ObstacleSub obstacle_subscription(_n);

  // runs the control loop at 100Hz
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    // geometry_msgs::Twist planner_cmd_vel;
    // planner.computeVelocityCommands(planner_cmd_vel);
    // ROS_INFO("Velocity Command: %.8f %.8f %.8f",planner_cmd_vel.linear.x,planner_cmd_vel.linear.y,planner_cmd_vel.linear.z);

    tf::Vector3 demanded_vel(setpoint_vel);

    // compute delta vel from obstacle
    tf::Vector3 delta_vel = px4_subscription.global_pose.getOrigin() - obstacle_subscription.obs_center;
    delta_vel[2] = 0;

    // make the velocity be zero a little bit outside of the obstacle
    float dist = delta_vel.length() - obstacle_subscription.obs_radius - obs_buffer;
    if (dist > 1) {
      delta_vel.setZero();
    } else {
      if (delta_vel.length()>0) {
        delta_vel /= delta_vel.length();
        delta_vel *= obs_vel_scale * demanded_vel.length() / (1+dist);
      }
    }

    // compute the demanded velocity
    tf::Vector3 out_vel = demanded_vel + delta_vel;
    if (out_vel.length()>out_vel_max && out_vel.length()>0) {
      out_vel /= out_vel.length();
      out_vel *= out_vel_max;
    }

    // add delta vel to setpoint_vel
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.twist.linear.x = 0.75; //out_vel.getX();
    cmd_vel.twist.linear.y = 0.0;//out_vel.getY();
    cmd_vel.twist.linear.z = 0.0;//out_vel.getZ();
    cmd_vel.header.frame_id = "fcu";

    _px4_pub.publish(cmd_vel);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
