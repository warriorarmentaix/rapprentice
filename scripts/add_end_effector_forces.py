import argparse
import IPython
import h5py
import numpy as np
from old_rapprentice import robot_states
import openravepy
import rospy
from old_rapprentice import PR2
import cPickle as pickle


parser = argparse.ArgumentParser()
parser.add_argument("h5file", type=str)
args = parser.parse_args()



rospy.init_node("exec_task",disable_signals=True)
pr2 = PR2.PR2()
env = pr2.env
robot = pr2.robot

force_file = open("force.pickle", 'wa')
force_data = {}

demofile = h5py.File(args.h5file, 'r')
for (key, seg_info) in demofile.items():
	print key
	end_effector_forces = {}
	end_effector_forces['l'] = []
	end_effector_forces['r'] = []
	for lr in 'lr':
		manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
		efforts_name = {"l":"efforts_l", "r":"efforts_r"}[lr]
		eef_name = {"l":"eef_l", "r":"eef_r"}[lr]
		arm = robot.GetManipulator(manip_name)
		traj = np.asarray(seg_info[manip_name])
		jacobians = []
		for i in range(traj.shape[0]):
			vals = traj[i,:]
			robot.SetDOFValues(vals, robot.GetManipulator(manip_name).GetArmIndices())
			jacobians.append(np.vstack((arm.CalculateJacobian(), arm.CalculateAngularVelocityJacobian())))
		jacobians = np.asarray(jacobians)
		efforts = seg_info[efforts_name]
		for i in range(efforts.shape[0]):
			forces = robot_states.compute_end_effector_force(jacobians[i], efforts[i])
			forces = np.asarray(forces.T)[0]
			end_effector_forces[lr].append(forces)
	
	force_data[str(key)] = end_effector_forces
pickle.dump(force_data, force_file)
force_file.close()