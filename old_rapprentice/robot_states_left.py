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

from std_msgs.msg import Float64MultiArray



import IPython

def call_return_jacobian_l():
    rospy.wait_for_service("return_jacobian_l")
    try:
        s = rospy.ServiceProxy("return_jacobian_l", ReturnJacobian)
        resp = s()
    except rospy.ServiceException, e:
        print "error when calling return_jacobian_l: %s"%e
    return (resp.jacobian)


def talker():
    #pub_r = rospy.Publisher('pr2_jacobian_r', Float64MultiArray)
    pub_l = rospy.Publisher('pr2_jacobian_l', Float64MultiArray)
    rospy.init_node('talker2', anonymous=True)
    r = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        #msg_r = Float64MultiArray()
        msg_l = Float64MultiArray()
        #msg_r.data = (np.array(call_return_jacobian_r()))
        msg_l.data = (np.array(call_return_jacobian_l()))
        #pub_r.publish(msg_r)
        pub_l.publish(msg_l)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
