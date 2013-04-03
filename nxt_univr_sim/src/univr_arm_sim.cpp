#include <ros/ros.h>
#include "/opt/ros/electric/stacks/common_msgs/sensor_msgs/msg_gen/cpp/include/sensor_msgs/JointState.h"
#include "/opt/ros/electric/stacks/nxt/nxt_msgs/msg_gen/cpp/include/nxt_msgs/JointCommand.h"
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>

using namespace std;

class nxt_sim
{
    public:
        nxt_sim();
        void publishStates();
    private:
        void cmd_callback(const sensor_msgs::JointState::ConstPtr& cmd);
        
        ros::NodeHandle nh;
        ros::Publisher position;
        ros::Subscriber command;
        sensor_msgs::JointState states;
        float old_pos[3];
        float next_pos[3];
        ros::Time last_publish;
        ros::Time aj1_update, aj2_update, aj3_update;
        double dt;
};

nxt_sim::nxt_sim()
{    
    // link
    position = nh.advertise<sensor_msgs::JointState>("joint_states",60);
    command = nh.subscribe<sensor_msgs::JointState>("position_controller",60, &nxt_sim::cmd_callback, this);
    
    // JointState(s) message
    states.name.resize(3);
    states.position.resize(3);
    states.velocity.resize(3);
    states.effort.resize(3);
    states.name[0].assign("arm_joint_1");
    states.name[1].assign("arm_joint_2");
    states.name[2].assign("arm_joint_3");
    for (int i = 0; i < 3; ++i)
    {
        nxt_sim::next_pos[i] = 0.0;
        states.position[i] = 0.0;
    }
    aj1_update = ros::Time::now();
    aj2_update = ros::Time::now();
    aj3_update = ros::Time::now();
    nxt_sim::states.header.stamp = ros::Time::now();
}
    
void nxt_sim::publishStates()
{
    for(int k=0; k<3; ++k)
    {
        // get former position from k-th joint
        old_pos[k] = nxt_sim::states.position[k];
        // set next position for k-th joint
        nxt_sim::states.position[k] = nxt_sim::next_pos[k];
    
        // get delay from former to latter message
        last_publish = nxt_sim::states.header.stamp;
        dt = (ros::Time::now().toSec() - last_publish.toSec()); 
        // set velocity
        nxt_sim::states.velocity[k] = ((nxt_sim::states.position[k] - old_pos[k])/dt);
    }
    // update message stamp
    nxt_sim::states.header.stamp = ros::Time::now();
    // publish stateS
    position.publish(states);
    
}
    
void nxt_sim::cmd_callback(const sensor_msgs::JointState::ConstPtr& cmd)
{
  // if more than 1/30th of a second has passed, let's update data
  std::string motor_name = cmd->name[0].c_str();

    if (motor_name == "arm_joint_1" && ((ros::Time::now().toSec() - aj1_update.toSec()) > (1/60)) )
    {
        nxt_sim::next_pos[0] = cmd->position[0];    
        aj1_update = ros::Time::now();
    }
    else if (motor_name == "arm_joint_2" && ((ros::Time::now().toSec() - aj2_update.toSec()) > (1/60)) )
    {
        nxt_sim::next_pos[1] = cmd->position[0];        
        aj2_update = ros::Time::now();
    }
    else if (motor_name == "arm_joint_3" && ((ros::Time::now().toSec() - aj3_update.toSec()) > (1/60)) )
    {
        nxt_sim::next_pos[2] = cmd->position[0];        
        aj3_update = ros::Time::now();
    }
    else
    {
        ROS_ERROR("Message not recognized, not moving");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nxt_sim");
    nxt_sim nxtsim;
    
    ros::Rate loop_rate(100);
    
    while( ros::ok() )
    {
        ros::spinOnce();
        nxtsim.publishStates();
        loop_rate.sleep();
    }
}
    
    
