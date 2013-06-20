#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('nxt_univr_controllers')  
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

    self.motor2initialized = False
    self.motor2vel = 0
    self.motor2error = 0
    self.motor2integral_err = 0
    self.motor2prev_err = 0
    self.motor2delta_err = 0

    self.motor3initialized = False
    self.motor3vel = 0
    self.motor3error = 0
    self.motor3integral_err = 0
    self.motor3prev_err = 0
    self.motor3delta_err = 0

    # Motor 1
    self.motor1KP = 0.6 # 0.55
    self.motor1KI = 0.0
    self.motor1KD = 0.6

    #	Motor 2
    self.motor2KP = 1.5
    self.motor2KI = 0.0
    self.motor2KD = 0.0

    #	Motor 3
    self.motor3KP = 0.75
    self.motor3KI = 0.0
    self.motor3KD = 0.0
    
    # get joint name
    #self.name = rospy.get_param('name', 'motor_1')

    #Motor 1
    self.motor1name = 'arm_joint_1'
    #Motor 2
    self.motor2name = 'arm_joint_2'
    #Motor 3 
    self.motor3name = 'arm_joint_3'
        
    # desired joint position (PID setpoint)
    rospy.Subscriber('position_controller', JointState, self.jnt_pos_cb)
        
    # joint interaction
    self.pub = rospy.Publisher('joint_command', JointCommand)
    rospy.Subscriber('joint_states', JointState, self.jnt_state_cb)
    
  def jnt_pos_cb(self, msg):
    if msg.name[0] == self.motor1name:
      self.motor1pos_desi = msg.position[0]
      #print 'Setpoint callback triggered from msg name %s: %f;'%(self.motor1name, self.motor1pos_desi)
    
    if msg.name[1] == self.motor2name:
      self.motor2pos_desi = msg.position[1]
      #print 'Setpoint callback triggered from msg name %s: %f;'%(self.motor2name, self.motor2pos_desi)
    
    if msg.name[2] == self.motor3name:
      self.motor3pos_desi = msg.position[2]
      #print 'Setpoint callback triggered from msg name %s: %f;'%(self.motor3name, self.motor3pos_desi)
    

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

      if name == self.motor2name:
        
        if not self.motor2initialized:
          self.motor2pos_desi = pos
          self.motor2initialized = True

        self.motor2error = self.motor2pos_desi - pos
        self.motor2delta_err = self.motor2prev_err - self.motor2error
        self.motor2integral_err += self.motor2error
        
        if self.motor2integral_err > 0.5:
          self.motor2integral_err = 0.5
        if self.motor2integral_err < -0.5:
          self.motor2integral_err = -0.5

        self.motor2p_out = self.motor2error * self.motor2KP
        self.motor2i_out = self.motor2integral_err * self.motor2KI
        self.motor2d_out = self.motor2delta_err * self.motor2KD
        self.motor2output = self.motor2p_out + self.motor2i_out + self.motor2d_out

        if self.motor2output > 0.65:
          self.motor2output = 0.65
        if self.motor2output < -0.65:
          self.motor2output = -0.65

        self.motor2prev_err = self.motor2error

        cmd = JointCommand()
        cmd.name = self.motor2name
        cmd.effort = self.motor2output ;
        #print 'Motor Joint 2 at %f, going to %f, commanding joint %f'%(pos,self.motor2pos_desi, cmd.effort)
        self.pub.publish(cmd)

      if name == self.motor3name:
        
        if not self.motor3initialized:
          self.motor3pos_desi = pos
          self.motor3initialized = True

        self.motor3error = self.motor3pos_desi - pos
        self.motor3delta_err = self.motor3prev_err - self.motor3error
        self.motor3integral_err += self.motor3error
        if self.motor3integral_err > 0.5:
          self.motor3integral_err = 0.5
        if self.motor3integral_err < -0.5:
          self.motor3integral_err = -0.5

        self.motor3p_out = self.motor3error * self.motor3KP
        self.motor3i_out = self.motor3integral_err * self.motor3KI
        self.motor3d_out = self.motor3delta_err * self.motor3KD
        self.motor3output = self.motor3p_out + self.motor3i_out + self.motor3d_out
        
        if self.motor3output > 0.65:
          self.motor3output = 0.65
        if self.motor3output < -0.65:
          self.motor3output = -0.65

        self.motor3prev_err = self.motor3error

        cmd = JointCommand()
        cmd.name = self.motor3name
        cmd.effort = self.motor3output ;
        #print 'Motor Joint 3 at %f, going to %f, commanding joint %f'%(pos,self.motor3pos_desi, cmd.effort)
        self.pub.publish(cmd)


def main():
    rospy.init_node('position_controller')
    jnt_pos_controller = JointPositionController()
    rospy.spin()


if __name__ == '__main__':
    main()
