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
parser.add_argument("--pickle_name", type=str)
args = parser.parse_args()

# Load recorded and warped end effector forces from the cPickle file
pickle_file = open("../data/" + args.pickle_name + ".pickle")
dic = pickle.load(pickle_file)
old_eefs = dic['old_eefs']
warped_eefs = dic['new_eefs']
pickle_file.close()

# Load jacobians and efforts from the bag file
bag_file = rosbag.Bag("../data/" + args.bag_name + ".bag")
jacobians = np.asarray(bag_proc.extract_jacobians(bag_file))
joint_names, stamps, traj, efforts_all = bag_proc.extract_joints(bag_file)
pid, jtf = bag_proc.extract_torques(bag_file)
jtf = np.array(jtf)
pid = np.array(pid)
efforts = np.asarray(bag_proc.select_with_names(efforts_all, joint_names, robot_states.arm_joint_names))
bag_file.close()

# Compute playback end effector forces using jacobians and efforts
traj_length = min(len(jacobians), len(efforts), len(jtf)/10)
ds_inds = np.arange(0, len(jtf), 10)
jtf_ds = jtf[ds_inds]
pid_ds = pid[ds_inds]
playback_eefs = np.asarray(bag_proc.compute_end_effector_forces(jacobians[0:traj_length], efforts[0:traj_length]))
jtf_eefs = np.asarray(bag_proc.compute_end_effector_forces(jacobians[0:traj_length], jtf_ds[0:traj_length]))
pid_eefs = np.asarray(bag_proc.compute_end_effector_forces(jacobians[0:traj_length], pid_ds[0:traj_length]))
pid_eefs = pid_eefs / 10.0
IPython.embed()

# Plot
sub = plt.subplot(511)
sub.set_title('recorded forces')
axis = np.arange(len(old_eefs))
p1, = plt.plot(axis, old_eefs[:,0], color='r')
p2, = plt.plot(axis, old_eefs[:,1], color='g')
p3, = plt.plot(axis, old_eefs[:,2], color='b')
plt.legend([p1, p2, p3], ["x", "y", "z"])

sub = plt.subplot(512)
sub.set_title('warped forces')
axis = np.arange(len(warped_eefs))
p1, = plt.plot(axis, warped_eefs[:,0], color='r')
p2, = plt.plot(axis, warped_eefs[:,1], color='g')
p3, = plt.plot(axis, warped_eefs[:,2], color='b')
plt.legend([p1, p2, p3], ["x", "y", "z"])

sub = plt.subplot(513)
sub.set_title('desired forces')
axis = np.arange(len(jtf_eefs))
p1, = plt.plot(axis, jtf_eefs[:,0], color='r')
p2, = plt.plot(axis, jtf_eefs[:,1], color='g')
p3, = plt.plot(axis, jtf_eefs[:,2], color='b')
plt.legend([p1, p2, p3], ["x", "y", "z"])

sub = plt.subplot(514)
sub.set_title('pid forces')
axis = np.arange(len(pid_eefs))
p1, = plt.plot(axis, pid_eefs[:,0], color='r')
p2, = plt.plot(axis, pid_eefs[:,1], color='g')
p3, = plt.plot(axis, pid_eefs[:,2], color='b')
plt.legend([p1, p2, p3], ["x", "y", "z"])

sub = plt.subplot(515)
sub.set_title('actual forces')
axis = np.arange(len(playback_eefs))
p1, = plt.plot(axis, playback_eefs[:,0], color='r')
p2, = plt.plot(axis, playback_eefs[:,1], color='g')
p3, = plt.plot(axis, playback_eefs[:,2], color='b')
plt.legend([p1, p2, p3], ["x", "y", "z"])

plt.show()
