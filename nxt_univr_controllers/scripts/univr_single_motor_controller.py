#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_controllers')  
import rospy
import math
import thread
from sensor_msgs.msg import JointState
from nxt_msgs.msg import JointCommand

class JointPositionController:
  def __init__(self):
    self.motor1initialized = False
    self.motor1vel = 0
    self.motor1error = 0
    self.motor1integral_err = 0
    self.motor1prev_err = 0
    self.motor1delta_err = 0

    # Motor 1
    self.motor1KP = 1.0 # 0.55
    self.motor1KI = 0.0
    self.motor1KD = 0.5

    self.motor1name = 'arm_joint_1'
            
    # desired joint position (PID setpoint)
    rospy.Subscriber('position_controller', JointState, self.jnt_pos_cb)
        
    # joint interaction
    self.pub = rospy.Publisher('joint_command', JointCommand)
    rospy.Subscriber('joint_states', JointState, self.jnt_state_cb)
    
  def jnt_pos_cb(self, msg):
    if msg.name[0] == self.motor1name:
      self.motor1pos_desi = msg.position[0]

  def jnt_state_cb(self, msg):
    for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
      if name == self.motor1name:
        if not self.motor1initialized:
          self.motor1pos_desi = pos
          self.motor1initialized = True
        
        self.motor1error = self.motor1pos_desi - pos
        self.motor1delta_err = self.motor1prev_err - self.motor1error
        self.motor1integral_err += self.motor1error
        
        if self.motor1integral_err > 0.5:
          self.motor1integral_err = 0.5
        if self.motor1integral_err < -0.5: 
          self.motor1integral_err = -0.5

        self.motor1p_out = self.motor1error * self.motor1KP
        self.motor1i_out = self.motor1integral_err * self.motor1KI
        self.motor1d_out = self.motor1delta_err * self.motor1KD	
        self.motor1output = self.motor1p_out + self.motor1i_out + self.motor1d_out
        
        if self.motor1output > 0.65:
          self.motor1output = 0.65
        if self.motor1output < -0.65:
          self.motor1output = -0.65

        self.motor1prev_err = self.motor1error
        
        cmd = JointCommand()
        cmd.name = self.motor1name
        cmd.effort = self.motor1output ;
        #print 'Motor Joint 1 at %f, going to %f, commanding joint %f'%(pos,self.motor1pos_desi, cmd.effort)
        self.pub.publish(cmd)
      
def main():
    rospy.init_node('position_controller')
    jnt_pos_controller = JointPositionController()
    rospy.spin()


if __name__ == '__main__':
    main()
