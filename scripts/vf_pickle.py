#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import cPickle as pickle
import IPython
import rosbag
from rapprentice import bag_proc, robot_states

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("demo_name", type=str)
args = parser.parse_args()


# Load recorded and warped end effector forces from the cPickle file
pickle_file = open("../data/" + args.demo_name + ".pickle")
dic = pickle.load(pickle_file)
old_eefs = dic['old_eefs']
warped_eefs = dic['new_eefs']
pickle_file.close()

# Plot
sub = plt.subplot(211)
sub.set_title('recorded forces')
axis = np.arange(len(old_eefs))
p1, = plt.plot(axis, old_eefs[:,0], color='r')
p2, = plt.plot(axis, old_eefs[:,1], color='g')
p3, = plt.plot(axis, old_eefs[:,2], color='b')
plt.legend([p1, p2, p3], ["x", "y", "z"])

sub = plt.subplot(212)
sub.set_title('warped forces')
axis = np.arange(len(warped_eefs))
p1, = plt.plot(axis, warped_eefs[:,0], color='r')
p2, = plt.plot(axis, warped_eefs[:,1], color='g')
p3, = plt.plot(axis, warped_eefs[:,2], color='b')
plt.legend([p1, p2, p3], ["x", "y", "z"])




plt.show()
