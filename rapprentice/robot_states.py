#!/usr/bin/env python
import numpy as np
import time

import roslib
import rospy
from hd_utils import conversions
import math
import rospy
roslib.load_manifest('joint_states_listener')
from joint_states_listener.srv import ReturnJointStates
roslib.load_manifest('jacobian_listener')
from jacobian_listener.srv import ReturnJacobian
from std_msgs.msg import Float64MultiArray



import IPython

arm_joint_names = ['r_shoulder_pan_joint',   
                   'r_shoulder_lift_joint',  
                   'r_upper_arm_roll_joint', 
                   'r_elbow_flex_joint',     
                   'r_forearm_roll_joint',   
                   'r_wrist_flex_joint',     
                   'r_wrist_roll_joint']



# In the order consistent with arm_joint_names
# Values taken from http://wiki.ros.org/robot_mechanism_controllers/JTCartesian%20Controller
joint_feedforward = [3.33, 1.16, 0.1, 0.25, 0.133, 0.0727, 0.0727]

torso_joint_names = ['torso_lift_joint']

arm_link_names = ['r_shoulder_pan_link',
                  'r_shoulder_lift_link',
                  'r_upper_arm_link',
                  'r_elbow_flex_link',
                  'r_forearm_link',
                  'r_wrist_flex_link',
                  'r_gripper_tool_frame']



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


def call_return_jacobian():
    rospy.wait_for_service("return_jacobian")
    try:
        s = rospy.ServiceProxy("return_jacobian", ReturnJacobian)
        resp = s()
    except rospy.ServiceException, e:
        print "error when calling return_jacobian: %s"%e
    return (resp.jacobian)


def compute_end_effector_force(J):
  (position, velocity, effort) = call_return_joint_states(arm_joint_names)
  M = np.diag(np.array(joint_feedforward)) # Convert joint feedforward values into diagonal array
  M_inv = np.linalg.inv(M)

  T = (np.matrix(effort)).T # Treat efforts as torques
  F = np.linalg.inv((J*M_inv*J.T)) * J * M_inv * T
  
  return F;

def compute_end_effector_velocity(J):
  (position, velocity, effort) = call_return_joint_states(arm_joint_names)
  dtheta = (np.matrix(velocity)).T
  dx = J * dtheta
  return dx
