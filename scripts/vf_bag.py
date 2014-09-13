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


# Load jacobians and efforts from the bag file
bag_file = rosbag.Bag"("../"data/" + args.demo_name + ".bag")
jacobians = np.asarray(bag_proc.extract_jacobians(bag_file))
joint_names, stamps, traj, efforts_all = bag_proc.extract_joints(bag_file)
efforts = np.asarray(bag_proc.select_with_names(efforts_all, joint_names, robot_states.arm_joint_names))
bag_file.close()

effs = bag_proc.compute_end_effector_forces(jacobians, efforts)
effs = np.array(effs)
effs = effs[:,0:3,:]

# Plot
sub = plt.subplot(111)
sub.set_title('recorded forces')
axis = np.arange(len(effs))
p1, = plt.plot(axis, effs[:,0], color='r')
p2, = plt.plot(axis, effs[:,1], color='g')
p3, = plt.plot(axis, effs[:,2], color='b')
plt.legend([p1, p2, p3], ["x", "y", "z"])




plt.show()
