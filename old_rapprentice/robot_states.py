#!/usr/bin/env python
import numpy as np
import time

import roslib
import rospy
import math
import rospy
roslib.load_manifest('joint_states_listener')
from joint_states_listener.srv import ReturnJointStates
roslib.load_manifest('jacobian_listener')
from jacobian_listener.srv import ReturnJacobian
roslib.load_manifest('torque_listener')
from torque_listener.srv import ReturnTorque
roslib.load_manifest('velocity_listener')
from velocity_listener.srv import ReturnVelocity

from std_msgs.msg import Float64MultiArray



import IPython

r_arm_joint_names = ['r_shoulder_pan_joint',   
                    'r_shoulder_lift_joint',  
                    'r_upper_arm_roll_joint', 
                    'r_elbow_flex_joint',     
                    'r_forearm_roll_joint',   
                    'r_wrist_flex_joint',     
                    'r_wrist_roll_joint']
l_arm_joint_names = ['l_shoulder_pan_joint',   
                    'l_shoulder_lift_joint',  
                    'l_upper_arm_roll_joint', 
                    'l_elbow_flex_joint',     
                    'l_forearm_roll_joint',   
                    'l_wrist_flex_joint',     
                    'l_wrist_roll_joint']



# In the order consistent with arm_joint_names
# Values taken from http://wiki.ros.org/robot_mechanism_controllers/JTCartesian%20Controller
joint_feedforward = [3.33, 1.16, 0.1, 0.25, 0.133, 0.0727, 0.0727]

torso_joint_names = ['torso_lift_joint']

r_arm_link_names = ['r_shoulder_pan_link',
                   'r_shoulder_lift_link',
                   'r_upper_arm_link',
                   'r_elbow_flex_link',
                   'r_forearm_link',
                   'r_wrist_flex_link',
                   'r_gripper_tool_frame']

l_arm_link_names = ['l_shoulder_pan_link',
                   'l_shoulder_lift_link',
                   'l_upper_arm_link',
                   'l_elbow_flex_link',
                   'l_forearm_link',
                   'l_wrist_flex_link',
                   'l_gripper_tool_frame']


def call_return_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    return (resp.position, resp.velocity, resp.effort)


def call_return_jacobian_r():
    rospy.wait_for_service("return_jacobian_r")
    try:
        s = rospy.ServiceProxy("return_jacobian_r", ReturnJacobian)
        resp = s()
    except rospy.ServiceException, e:
        print "error when calling return_jacobian_r: %s"%e
    return (resp.jacobian)


def call_return_jacobian_l():
    rospy.wait_for_service("return_jacobian_l")
    try:
        s = rospy.ServiceProxy("return_jacobian_l", ReturnJacobian)
        resp = s()
    except rospy.ServiceException, e:
        print "error when calling return_jacobian_l: %s"%e
    return (resp.jacobian)

def call_return_torque():
    rospy.wait_for_service("return_torque")
    try:
        s = rospy.ServiceProxy("return_torque", ReturnTorque)
        resp = s()
    except rospy.ServiceException, e:
        print "error when calling return_torque: %s"%e
    return (resp.torque)
def call_return_velocity():
    rospy.wait_for_service("return_velocity")
    try:
        s = rospy.ServiceProxy("return_velocity", ReturnVelocity)
        resp = s()
    except rospy.ServiceException, e:
        print "error when calling return_velocity: %s"%e
    return (resp.velocity)

def compute_end_effector_force(J, effort):
  M = np.diag(np.array(joint_feedforward)) # Convert joint feedforward values into diagonal array
  M_inv = np.linalg.inv(M)
  J = np.matrix(J)
  T = (np.matrix(effort)).T # Treat efforts as torques

  #F = np.linalg.inv((J*J.T)) * J  * T # Version without approx mass matrix
  F = np.linalg.inv((J*M_inv*J.T)) * J * M_inv * T # Version with approximated mass matrix
  
  return F;

def compute_end_effector_velocity(J):
  (position, velocity, effort) = call_return_joint_states(r_arm_joint_names)
  dtheta = (np.matrix(velocity)).T
  dx = J * dtheta
  return dx



def talker():
    pub_r = rospy.Publisher('pr2_jacobian_r', Float64MultiArray)
    #pub_l = rospy.Publisher('pr2_jacobian_l', Float64MultiArray)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        msg_r = Float64MultiArray()
        #msg_l = Float64MultiArray()
        msg_r.data = (np.array(call_return_jacobian_r()))
        #msg_l.data = (np.array(call_return_jacobian_l()))
        pub_r.publish(msg_r)
        #pub_l.publish(msg_l)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
