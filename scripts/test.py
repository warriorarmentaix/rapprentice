#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, String
import IPython
import time



rospy.init_node("test")

length = 5

q = np.zeros((7, 1))
q[3] = 9
qs = np.asarray([q for i in range(length)])
qs = np.resize(qs, (1, 7*length))[0]

Kp = np.zeros((7, 7))
Kp[3][4] = 3
Kp[5][6] = 4
Kps = np.asarray([Kp for i in range(length)])
Kps = np.resize(Kps, (1, 49*length))[0]
Kvs = Kps


force = np.zeros((6, 1))
force[2][0] = 5
forces = np.asarray([force for i in range(length)])
forces = np.resize(forces, (1, 6*length))[0]

secs = 10.0
use_force = 1.0

data = np.zeros((1, len(qs) + len(Kps) + len(Kvs) + len(forces) + 2))
data[0][0:len(qs)] = qs
data[0][len(qs):len(qs)+len(Kps)] = Kps
data[0][len(qs)+len(Kps):len(qs)+len(Kps)+len(Kvs)] = Kvs
data[0][len(qs)+len(Kps)+len(Kvs):len(qs)+len(Kps)+len(Kvs)+len(forces)] = forces
data[0][-2] = use_force
data[0][-1] = secs


msg = Float64MultiArray()
msg.data = data[0].tolist()
print len(msg.data)
pub = rospy.Publisher("/controller_data", Float64MultiArray)
time.sleep(1)
pub.publish(msg)
print("published")