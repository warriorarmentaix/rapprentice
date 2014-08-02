import numpy as np
import time

import openravepy as rave
import roslib
roslib.load_manifest("tf")
import tf.transformations as tft


import IPython


lr = 'r'

arm_joint_names = ['_shoulder_pan_joint',   
                   '_shoulder_lift_joint',  
                   '_upper_arm_roll_joint', 
                   '_elbow_flex_joint',     
                   '_forearm_roll_joint',   
                   '_wrist_flex_joint',     
                   '_wrist_roll_joint']     


arm_joint_axes =  [[0,0,1],
                   [0,1,0],
                   [1,0,0],
                   [0,1,0],
                   [1,0,0],
                   [0,1,0],
                   [1,0,0]]

arm_link_trans = [[0., 0.188 if lr == 'l' else -0.188, 0.],
                  [ 0.1,  0.,  0.],
                  [ 0.,  0.,  0.],
                  [ 0.4,  0.,  0.],
                  [ 0.,  0.,  0.],
                  [ 0.321,  0.,  0.],
                  [0.18, 0, 0]]

start_link = 'torso_lift_link'

arm_link_names = ['_shoulder_pan_link',
                  '_shoulder_lift_link',
                  '_upper_arm_link',
                  '_elbow_flex_link',
                  '_forearm_link',
                  '_wrist_flex_link',
                  '_gripper_tool_frame']



def fk(robot, lr, joint_values):
    pose_mat = robot.GetLink(start_link).GetTransform()
    

    R = np.eye(4)
    for i, j in enumerate(joint_values):
        rot = tft.rotation_matrix(j, arm_joint_axes[i])
        trans = arm_link_trans[i]
        R[:3,:3] = rot[:3,:3]
        R[:3,3] = trans
        pose_mat = np.dot(pose_mat, R)
        
    return pose_mat

def test_fk():
    env = rave.Environment()
    env.Load("robots/pr2-beta-static.zae")
    robot = env.GetRobots()[0]

if __name__ == '__main__':
    test_fk()
