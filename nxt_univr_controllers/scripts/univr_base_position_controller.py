#!/usr/bin/env python
import roslib; roslib.load_manifest('nxt_controllers')  
import rospy
import math
import thread

from sensor_msgs.msg import JointState
from nxt_msgs.msg import  JointCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class JointPositionController:
  def __init__(self):
    self.wheel_radius = 0.028
    self.moveStatus = False
    self.rotateStatus = False
    self.lookStatus = False
    self.distance = 0
    self.move_desi = 0
    self.rotate_desi = 0
    self.position = 0
    self.motor_name = 'motor_sensor'
    self.velR = 0
    self.velL = 0
    
    self.move_vel = 0
    self.move_error = 0
    self.move_integral_err = 0
    self.move_prev_err = 0
    self.move_delta_err = 0
    
    self.rotate_vel = 0
    self.rotate_error = 0
    self.rotate_integral_err = 0
    self.rotate_prev_err = 0
    self.rotate_delta_err = 0
    self.previous_theta = 0
    self.theta = 0
    
    # controllers switch
    self.isActive = False

    # Move
    self.move_KP = 6.0
    self.move_KI = 0.25
    self.move_KD = 2.0

    # Rotate
    self.rotate_KP = 1.5
    self.rotate_KI = 0.0
    self.rotate_KD = 0.0
    self.theta = 0.0

    # Time
    self.last_time = rospy.get_time()

    # joint interaction
    self.pub = rospy.Publisher('joint_command', JointCommand)
    # joint states feedback
    rospy.Subscriber('joint_states', JointState, self.jointStates_callback)
    # odometry feedback
    rospy.Subscriber('nxt_odom', Odometry, self.nxt_odometry_callback)
    # setpoint subscriber
    rospy.Subscriber('position_setpoint', JointState, self.mobile_control_callback)
    # controller swtich subscriber
    rospy.Subscriber('velocity_setpoint', Twist, self.controller_status_callback)
    
       
  def controller_status_callback(self, msg):
  # If I receive a velocity setpoint then position control is not needed
    self.isActive = False
    self.moveStatus = False  
    self.rotateStatus = False  
    self.lookStatus = False

  def jointStates_callback(self, msg):
    if self.isActive:
      if self.moveStatus:
        for name, vel in zip(msg.name, msg.velocity):
          if name == 'motor_joint_left':
            self.velL = vel 
          if name == 'motor_joint_right':
            self.velR = vel
        
        current_time = rospy.get_time()
        self.distance += ((self.velR * self.wheel_radius + self.velL * self.wheel_radius) / 2) * ( current_time - self.last_time )           
        self.last_time = current_time
        self.move_error = self.move_desi - self.distance
        cmd = JointCommand()
        
        if abs(self.move_error) > 0.01:
          self.move_delta_err = self.move_prev_err - self.move_error
          self.move_integral_err += self.move_error
          if self.move_integral_err > 1:
            self.move_integral_err = 1
          if self.move_integral_err < -1: 
            self.move_integral_err = -1

          self.move_p_out = self.move_error * self.move_KP
          self.move_i_out = self.move_integral_err * self.move_KI
          self.move_d_out = self.move_delta_err * self.move_KD

          self.move_output = self.move_p_out + self.move_i_out + self.move_d_out
          if self.move_output > 0.7:
            self.move_output = 0.7
          if self.move_output < -0.7:
            self.move_output = -0.7

          self.move_prev_err = self.move_error
          #self.joint_speed_delta = self.velL - self.velR
          #self.effort_compensation = self.joint_speed_delta * self.js_KP

          cmd.name = "motor_joint_left"
          cmd.effort = self.move_output
          self.pub.publish(cmd)
          cmd.name = "motor_joint_right"
          self.pub.publish(cmd)
        else:
          cmd.name = "motor_joint_left"
          cmd.effort = 0.0
          self.pub.publish(cmd)
          
          cmd.name = "motor_joint_right"
          self.pub.publish(cmd)
          #self.moveStatus = False
          
        print 'Distance delta: [%f], Torque: [%f]'%(self.move_error , cmd.effort)
      
      if self.lookStatus:
        for name, pos in zip(msg.name, msg.position):
          if name == 'motor_sensor':
            self.position = pos
        
        self.move_error = self.move_desi - self.position

        cmd = JointCommand()    
        cmd.name = self.motor_name
        cmd.effort = 1.25 * self.move_error

        if cmd.effort > 0.65:
          cmd.effort = 0.65
        if cmd.effort < -0.65:
          cmd.effort = -0.65


        self.pub.publish(cmd)

  def mobile_control_callback(self, msg):
    self.isActive = True
    if msg.name[0] == 'rotate':
      self.rotate_desi = msg.position[0]
      self.moveStatus = False
      self.lookStatus = False
      self.rotateStatus = True

    if msg.name[0] == 'move':
      self.move_desi = msg.position[0]
      self.distance = 0
      self.rotateStatus = False
      self.lookStatus = False
      self.moveStatus = True
      
    if msg.name[0] == 'look':
      self.move_desi = msg.position[0]
      self.rotateStatus = False
      self.moveStatus = False
      self.lookStatus = True
                 
  
  def nxt_odometry_callback(self, msg):
    if self.isActive:
      if self.rotateStatus:
        cmd = JointCommand()
        r, p, y = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.theta = y
        beta = self.rotate_desi
        alpha = beta - self.theta
        
        if alpha > math.pi:
          alpha = -(2*math.pi) + alpha
        if alpha < -math.pi:
          alpha = 2*math.pi + alpha

        #print 'align error: %f'%(alpha)
        self.rotate_error = alpha
        
        if abs(self.rotate_error) > 0.057:
          self.rotate_delta_err = self.rotate_prev_err - self.rotate_error
          self.rotate_integral_err += self.rotate_error
          
          if self.rotate_integral_err > 1:
            self.rotate_integral_err = 1
          if self.rotate_integral_err < -1: 
            self.rotate_integral_err = -1

          self.rotate_p_out = (self.rotate_error * self.rotate_KP)
          self.rotate_i_out = self.rotate_integral_err * self.rotate_KI
          self.rotate_d_out = self.rotate_delta_err * self.rotate_KD
          self.rotate_output = (self.rotate_p_out + self.rotate_i_out + self.rotate_d_out)
          
          if self.rotate_output > 0.65:
            self.rotate_output = 0.65
          if self.rotate_output < -0.65:
            self.rotate_output = -0.65

          cmd.name = "motor_joint_right"
          cmd.effort = self.rotate_output
          #cmd.effort = 0
          self.pub.publish(cmd)
          cmd.name = "motor_joint_left"
          #cmd.effort = 0
          cmd.effort = -self.rotate_output
          self.pub.publish(cmd)
        else:
          cmd.name = "motor_joint_right"
          cmd.effort = 0
          self.pub.publish(cmd)
          cmd.name = "motor_joint_left"
          cmd.effort = 0
          self.pub.publish(cmd)
          
          
        #print 'rad(target): [%f]; rad(current): [%f]; error: [%f]; left torque: [%f]'%(beta, theta, self.rotate_error, cmd.effort)

              
def main():
    rospy.init_node('base_position_controller')
    jnt_pos_controller = JointPositionController()
    rospy.spin()
      
if __name__ == '__main__':
  main()
