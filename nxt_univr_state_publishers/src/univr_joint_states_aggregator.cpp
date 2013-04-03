#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include <time.h>
#include <cstdlib>
#include <deque>

#define DQ_SIZE 15

using namespace std;

class univr_joint_states_aggregator
{
  public:
    univr_joint_states_aggregator();
  private:
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Publisher states_pub;
    ros::Subscriber state_sub;
    ros::Time last_pub_time[3];
    ros::Time current_time[3];
    
    sensor_msgs::JointState jstates;

    std::string mobile;
    std::string motor_name;
    std::vector< std::deque<float> > deques;
    
    int index;
    float sum;
};

univr_joint_states_aggregator::univr_joint_states_aggregator()
{
  index = -1;
  sum = 0.0;
  deques.resize(3);
  
  for (int i=0; i < 3; ++i)
  {
    std::deque<float> median_filter(DQ_SIZE,0.0);
    deques[i] = median_filter;
    last_pub_time[i] = current_time[i] = ros::Time::now();
  }
  state_sub = nh_.subscribe<sensor_msgs::JointState>("joint_state",1, &univr_joint_states_aggregator::joint_state_callback, this);
  states_pub = nh_.advertise<sensor_msgs::JointState>("joint_states",5);
  
  jstates.name.resize(3);
  jstates.position.resize(3);
  jstates.velocity.resize(3);
  jstates.effort.resize(3);
  
  if ( !ros::param::get("/mobile", mobile) )
  {
    ROS_ERROR("/mobile parameter not found");
  }
  else if ( mobile == "false" )
  {
    jstates.name[0].assign("arm_joint_1");
    jstates.name[1].assign("arm_joint_2");
    jstates.name[2].assign("arm_joint_3");
  }
  else if ( mobile == "true" )
  {
    jstates.name[0].assign("motor_joint_left");
    jstates.name[1].assign("motor_joint_right");
    jstates.name[2].assign("motor_sensor");      
  }
  else
  {
    ROS_ERROR("Parameter /mobile not correctly parsed");
  }
  for (int i = 0; i < 3; ++i)
  {
    jstates.position[i] = 0.0;
    jstates.velocity[i] = 0.0;
    jstates.effort[i] = 0.0;
  }
  jstates.header.stamp = ros::Time::now(); 
};

void univr_joint_states_aggregator::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  motor_name = msg->name[0].c_str();
  
  if (motor_name == "arm_joint_1" || motor_name == "motor_joint_left")
  {
    index = 0;
  }
  else if (motor_name == "arm_joint_2" || motor_name == "motor_joint_right")
  {
    index = 1;
  }
  else if (motor_name == "arm_joint_3" || motor_name == "motor_sensor")
  {
    index = 2;
  }
  else
  {
    ROS_ERROR("Message not recognized.");
    index = -1;
  }
  if (index != -1)
  {
    jstates.position[index] = msg->position[0];
    jstates.effort[index] = msg->effort[0];
    
    deques[index].pop_front();
    deques[index].push_back(msg->velocity[0]);
    sum = 0;
    for ( int i=0; i < DQ_SIZE; ++i)
    {
      sum += deques[index][i];      
    }
    jstates.velocity[index] = sum/DQ_SIZE;
    
    current_time[index] = ros::Time::now();
    jstates.header.stamp = current_time[index];
    states_pub.publish(jstates);
    last_pub_time[index] = ros::Time::now();
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "univr_joint_states_aggregator");
    univr_joint_states_aggregator aggregator;
    ros::spin();
}
