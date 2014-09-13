#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import cPickle as pickle
import IPython
import rosbag
from rapprentice import bag_proc, robot_states

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--bag_name", type=str)
args = parser.parse_args()


# Load pid and jtf torques from the bag file
bag_file = rosbag.Bag("../data/" + args.bag_name + ".bag")
pid, jtf = bag_proc.extract_torques(bag_file)
traj_length = min(len(pid), len(jtf))
pid = np.array(pid[0:traj_length])
jtf = np.array(jtf[0:traj_length])
bag_file.close()



# Plot

for i in range(7):
	plot_num = 710 + i+1
	sub = plt.subplot(plot_num)
	sub.axes.get_xaxis().set_visible(False)
	sub.set_title(robot_states.arm_joint_names[i])
	axis = np.arange(traj_length)
	p1, = plt.plot(axis, pid[:,i], color='r')
	p2, = plt.plot(axis, jtf[:,i], color='g')
	plt.legend([p1, p2], ["pid", "jtf"])
plt.show()