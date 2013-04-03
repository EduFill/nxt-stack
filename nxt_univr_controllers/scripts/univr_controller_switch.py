#!/usr/bin/env python
import roslib; roslib.load_manifest('nxt_controllers')  
import rospy
import math
import thread
import numpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nxt_msgs.msg import Range, JointCommand
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SwitchController:
  def __init__(self):
    
    self.pos_pub = rospy.Publisher('position_setpoint', JointState)
    self.vel_pub = rospy.Publisher('velocity_setpoint', Twist)
        
    rospy.Subscriber('position_controller', JointState, self.pos_control_switch)
    rospy.Subscriber('cmd_vel',Twist, self.vel_control_switch)
    
  def pos_control_switch(self, msg):
    
    p_setpoint = JointState()
    p_setpoint.name = [None]*1
    p_setpoint.position = [None]*1
    p_setpoint.name[0] = msg.name[0]
    p_setpoint.position[0] = msg.position[0]
    
    self.pos_pub.publish(p_setpoint)
    print 'Publishing position setpoint'
    
  def vel_control_switch(self, msg):
    
    v_setpoint = Twist()
    v_setpoint.angular.z = msg.angular.z
    v_setpoint.linear.x = msg.linear.x
    
    self.vel_pub.publish(v_setpoint)
    print 'Publishing velocity setpoint'
  
def main():
    rospy.init_node('univr_controllers_switch')
    controller_switch = SwitchController()
    rospy.spin()

if __name__ == '__main__':
    main()
