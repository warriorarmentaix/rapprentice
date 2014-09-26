#!/usr/bin/env python
import trajoptpy, openravepy
import os, numpy as np, h5py, time, os.path as osp
from rapprentice import robot_states

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



L_POSTURES = dict(        
    untucked = [0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0],
    tucked = [0.06, 1.25, 1.79, -1.68, -1.73, -0.10, -0.09],
    up = [ 0.33, -0.35,  2.59, -0.15,  0.59, -1.41, -0.27],
    side = [  1.832,  -0.332,   1.011,  -1.437,   1.1  ,  -2.106,  3.074]
)


rospy.init_node('torque_velocity_visualization')
listener = tf.TransformListener()
time.sleep(1)

def update():
	
	velocities = np.array(robot_states.call_return_velocity())
	torques = np.array(robot_states.call_return_torque())
	

	return torques, velocities

xAchse=pylab.arange(0,100,1)
yAchse=pylab.array([0]*100)

fig = pylab.figure(1)

"""
torque_plots = [None for i in range(7)]
for i in range(7):
	torque_plots[i] = fig.add_subplot(7, 1, i)
"""

velocity_plots = [None for i in range(7)]
for i in range(7):
	velocity_plots[i] = fig.add_subplot(7, 1, i)

"""
torque_lines = [None for i in range(7)]
for i in range(7):
	torque_plots[i].axis([0,100,0,100])
	torque_lines[i] = torque_plots[i].plot(xAchse, yAchse, '-', color='red')
"""

velocity_lines = [None for i in range(7)]
for i in range(7):
	velocity_plots[i].axis([0,100,0,100])
	velocity_lines[i] = velocity_plots[i].plot(xAchse, yAchse, '-', color='red')


manager = pylab.get_current_fig_manager()

torque_values = [None for i in range(7)]
velocity_values = [None for i in range(7)]
for i in range(7):
	torque_values[i] = [0 for x in range(100)]
	velocity_values[i] = [0 for x in range(100)]

def UpdateValue(arg):
	torques, velocities = update()
	for i in range(7):
		torque_values[i].append(torques[i])
		velocity_values[i].append(velocities[i])
	


def RealtimePloter(arg):
	CurrentXAxis=pylab.arange(len(torque_values[0])-100,len(torque_values[0]),1)
	for i in range(7):
		#torque_lines[i][0].set_data(CurrentXAxis,pylab.array(torque_values[i][-100:]))
		velocity_lines[i][0].set_data(CurrentXAxis,pylab.array(velocity_values[i][-100:]))
		#torque_plots[i].axis([CurrentXAxis.min(),CurrentXAxis.max(),0,0.2])
		velocity_plots[i].axis([CurrentXAxis.min(),CurrentXAxis.max(),0,1])
	manager.canvas.draw()

timer = fig.canvas.new_timer(interval=1)
timer.add_callback(RealtimePloter, ())
timer2 = fig.canvas.new_timer(interval=1)
timer2.add_callback(UpdateValue, ())
timer.start()
timer2.start()

pylab.show()
