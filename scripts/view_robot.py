#!/usr/bin/env python
import trajoptpy, openravepy
import os, numpy as np, h5py, time, os.path as osp
from old_rapprentice import robot_states, PR2

from hd_rapprentice import registration, animate_traj, ros2rave, plotting_openrave
from hd_utils.defaults import cad_files_dir
from hd_utils import conversions
import IPython
import roslib
roslib.load_manifest("tf")
import tf
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped, Vector3
import pylab
from pylab import *
import time
import openravepy



#ros stuffs
def mirror_arm_joints(x):
    "mirror image of joints (r->l or l->r)"
    return np.r_[-x[0],x[1],-x[2],x[3],-x[4],x[5],-x[6]]

def scale(v, factor):
	return v/factor

L_POSTURES = dict(        
    untucked = [0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0],
    tucked = [0.06, 1.25, 1.79, -1.68, -1.73, -0.10, -0.09],
    up = [ 0.33, -0.35,  2.59, -0.15,  0.59, -1.41, -0.27],
    side = [  1.832,  -0.332,   1.011,  -1.437,   1.1  ,  -2.106,  3.074]
)


fake_env = openravepy.Environment()
fake_env.StopSimulation()
fake_env.Load("robots/pr2-beta-static.zae")
fake_robot = fake_env.GetRobots()[0]
viewer = trajoptpy.GetViewer(fake_env)

fake_env.Load(osp.join(cad_files_dir, 'table_sim.xml'))
body = fake_env.GetKinBody('table')
viewer.SetTransparency(body,0.4)


rospy.init_node("visualize",disable_signals=True)
pr2 = PR2.PR2()
env = pr2.env
robot = pr2.robot
listener = tf.TransformListener()

time.sleep(1)

def update():
	
	trans, rot = listener.lookupTransform("/base_link", "/r_gripper_tool_frame", rospy.Time(0))
	t = time.time()


    # Set robot in simulation
	(arm_position, arm_velocity, arm_effort) = robot_states.call_return_joint_states(robot_states.r_arm_joint_names)
	r_vals = arm_position
	robot.SetDOFValues(r_vals, robot.GetManipulator('rightarm').GetArmIndices())
	fake_robot.SetDOFValues(r_vals, robot.GetManipulator('rightarm').GetArmIndices())
	(torso_position, torso_velocity, torso_effort) = robot_states.call_return_joint_states(robot_states.torso_joint_names)
	t_vals = torso_position
	robot.SetDOFValues(t_vals, [12]) #Torso manipular index
	fake_robot.SetDOFValues(t_vals, [12])


	#Get end effector force & torque
	arm = robot.GetManipulator('rightarm')
	J = np.vstack((arm.CalculateJacobian(), arm.CalculateAngularVelocityJacobian()))
	print J
	eff_force = robot_states.compute_end_effector_force(J, arm_effort).T
	eff_force =  np.array(eff_force)[0]
	force = eff_force[:3]
	torque = eff_force[3:]


	"""
	# Publish force
	header = Header()
	header.stamp = rospy.Time.now()
	pub.publish(Vector3Stamped(header,Vector3(force[0],force[1],force[2])))
	"""

	# Plot end effector force & torque
	handles = []
	
	f_start = np.array([0,0,0]) + trans
	f_plot = scale(np.array(force),100) # for plotting
	f_end = scale(np.array(force),100) + trans
	#handles.append(fake_env.drawlinestrip(np.array([f_start, f_end]), 10, (1,0,0,1)))

	t_start = np.array([0,0,0]) + trans
	t_plot = scale(np.array(torque),27) # for plotting
	t_end = scale(np.array(torque),27) + trans
	#handles.append(fake_env.drawlinestrip(np.array([t_start, t_end]), 10, (0,1,0,1)))
	


	# Get end effector velocity
	eff_velocity = robot_states.compute_end_effector_velocity(J).T

	eff_velocity = np.array(eff_velocity)[0]
	trans_vel = eff_velocity[:3]
	v_start = np.array([0,0,0]) + trans
	v_plot = scale(np.array(trans_vel),3) # for plotting
	v_end = scale(np.array(trans_vel),3) + trans
	handles.append(fake_env.drawlinestrip(np.array([v_start, v_end]*20), 10, (0,0,1,1)))

	viewer.Step()

	return f_plot, t_plot, v_plot

xAchse=pylab.arange(0,100,1)
yAchse=pylab.array([0]*100)

fig = pylab.figure(1)
ax = fig.add_subplot(3, 1, 1)
bx = fig.add_subplot(3, 1, 2)
cx = fig.add_subplot(3, 1, 3)

ax.grid(True)
bx.grid(True)
cx.grid(True)

ax.set_title("Force")
ax.axis([0,100,0,100])
line1=ax.plot(xAchse, yAchse, '-', label='force', color='red')

bx.set_title("Torque")
bx.axis([0,100,0,100])
line2=bx.plot(xAchse, yAchse, '-', label='torque', color='green')


cx.set_title("Velocity")
cx.axis([0,100,0,100])
line3=cx.plot(xAchse, yAchse, '-', label='velocity', color='blue')
manager = pylab.get_current_fig_manager()


valuesforce=[]
valuesforce = [0 for x in range(100)]
valuestorque=[]
valuestorque = [0 for x in range(100)]
valuesvelocity=[]
valuesvelocity = [0 for x in range(100)]
def UpdateValue(arg):
	force, torque, velocity = update()
	force_mag = np.linalg.norm(force)
	valuesforce.append(force_mag)
	torque_mag = np.linalg.norm(torque)
	valuestorque.append(torque_mag)
	velocity_mag = np.linalg.norm(velocity)
	valuesvelocity.append(velocity_mag)
	


def RealtimePloter(arg):
	CurrentXAxis=pylab.arange(len(valuesforce)-100,len(valuesforce),1)
	line1[0].set_data(CurrentXAxis,pylab.array(valuesforce[-100:]))
	line2[0].set_data(CurrentXAxis,pylab.array(valuestorque[-100:]))
	line3[0].set_data(CurrentXAxis,pylab.array(valuesvelocity[-100:]))
	ax.axis([CurrentXAxis.min(),CurrentXAxis.max(),0,0.4])
	bx.axis([CurrentXAxis.min(),CurrentXAxis.max(),0,0.2])
	cx.axis([CurrentXAxis.min(),CurrentXAxis.max(),0,0.1])
	manager.canvas.draw()

timer = fig.canvas.new_timer(interval=1)
timer.add_callback(RealtimePloter, ())
timer2 = fig.canvas.new_timer(interval=1)
timer2.add_callback(UpdateValue, ())
timer.start()
timer2.start()

pylab.show()
