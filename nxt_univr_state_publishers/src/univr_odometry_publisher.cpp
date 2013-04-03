#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <math.h>

using namespace std;
using namespace ros;

class base_odometry_publisher
{
public:
  base_odometry_publisher();

private:
  /** Receive Jointstate Message from NXT Robot **/
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

  ros::NodeHandle nh_;

  ros::Publisher odom_pub;
  ros::Subscriber nxt_jointState_sub;

  tf::TransformBroadcaster odom_broadcaster;

  double l;
  double wheel_radius;
  /** Odometry **/
  double x;
  double y;
  double th;
  
  ros::Time current_time;
  ros::Time last_time;
};

base_odometry_publisher::base_odometry_publisher() 
{
  odom_pub = nh_.advertise<nav_msgs::Odometry>("nxt_odom", 10);
  nxt_jointState_sub = nh_.subscribe<sensor_msgs::JointState>("joint_states", 10, &base_odometry_publisher::joint_state_callback , this);
  
  x = 0.0;
  y = 0.0;
  th = 0.0;
  
  if(!nh_.getParam("base_parameters/wheel_basis", l))
  {
    l = 0.0625;
  }
  if( (!nh_.getParam("base_parameters/r_wheel_joint", wheel_radius)) && (!nh_.getParam("base_parameters/l_wheel_joint", wheel_radius)) ) 
  {
    wheel_radius = 0.028;   
  }

  last_time = ros::Time::now();
  current_time = ros::Time::now();
}

void base_odometry_publisher::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  current_time = ros::Time::now();
  double v = (msg->velocity[1] * wheel_radius + msg->velocity[0] * wheel_radius) / 2;
  double w = (msg->velocity[1] * wheel_radius - msg->velocity[0] * wheel_radius) / (l*2); 
  double dt = (current_time - last_time).toSec();
  double delta_x = dt * v * cos(th + ((dt * w) / 2));
  double delta_y = dt * v * sin(th + ((dt * w) / 2));
  double delta_th = dt * w; 

  x += delta_x;
  y += delta_y;
  th += delta_th;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster.sendTransform(odom_trans);
  
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "base_link";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = v * cos(th);
  odom.twist.twist.linear.y = v * sin(th);
  odom.twist.twist.angular.z = w;

  odom_pub.publish(odom);

  last_time = current_time;
  //std::cout << current_time - last_time << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_odometry_publisher");
  base_odometry_publisher nxt_odom;
  ros::spin();
}
