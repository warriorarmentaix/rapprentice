#!/usr/bin/env python

import argparse
from rapprentice import util
usage="""

Run in simulation with a translation and a rotation of fake data:
./do_task.py ~/Data/sampledata/overhand/overhand.h5 --fake_data_segment=overhand0_seg00 --execution=0  --animation=1 --select_manual --fake_data_transform .1 .1 .1 .1 .1 .1

Run in simulation choosing the closest demo, single threaded
./do_task.py ~/Data/all.h5 --fake_data_segment=demo1-seg00 --execution=0  --animation=1  --parallel=0 

Actually run on the robot without pausing or animating 
./do_task.py ~/Data/overhand2/all.h5 --execution=1 --animation=0

"""
# parser = argparse.ArgumentParser(usage=usage)
# parser.add_argument("h5file", type=str)
# parser.add_argument("--cloud_proc_func", default="filter_green")
# parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")
# parser.add_argument("--useJK", type=int, default=0)
# parser.add_argument("--execution", type=int, default=0)
# parser.add_argument("--animation", type=int, default=0)
# parser.add_argument("--parallel", type=int, default=1)
# parser.add_argument("--pid", type=int, default=0)

# parser.add_argument("--prompt", action="store_true")
# parser.add_argument("--show_neighbors", action="store_true")
# parser.add_argument("--select_manual", action="store_true")
# parser.add_argument("--log", action="store_true")
# parser.add_argument("--no_ds", action="store_true")
# parser.add_argument("--trajopt_ds", type=int, default=10)
# parser.add_argument("--visualize", action="store_true")

# parser.add_argument("--use_force", type=int, default=0)

# parser.add_argument("--fake_data_segment",type=str)
# parser.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
#     default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")

# parser.add_argument("--interactive",action="store_true")



parser = util.ArgumentParser()
    


parser.add_argument("h5file", type=str)
parser.add_argument("--cloud_proc_func", default="filter_green")
parser.add_argument("--cloud_proc_mod", default="old_rapprentice.cloud_proc_funcs")
parser.add_argument("--useJK", type=int, default=0)
parser.add_argument("--execution", type=int, default=0)
#parser.add_argument("--animation", type=int, default=0)
parser.add_argument("--parallel", type=int, default=1)

parser.add_argument("--prompt", action="store_true")
parser.add_argument("--show_neighbors", action="store_true")
parser.add_argument("--select_manual", action="store_true")
parser.add_argument("--log", action="store_true")
parser.add_argument("--no_ds", action="store_true")
parser.add_argument("--trajopt_ds", type=int, default=10)
parser.add_argument("--visualize", action="store_true")
parser.add_argument("--kinematics", action="store_true")
parser.add_argument("--force", action="store_true")
parser.add_argument("--pr2", action="store_true")
parser.add_argument("--multiplier", type=int, default=10)



#parser.add_argument("--interactive",action="store_true")





parser.add_argument("--animation", type=int, default=0, help="animates if it is non-zero. the viewer is stepped according to this number")
parser.add_argument("--interactive", action="store_true", help="step animation and optimization if specified")
parser.add_argument("--camera_matrix_file", type=str, default='../.camera_matrix.txt')
parser.add_argument("--window_prop_file", type=str, default='../.win_prop.txt')
parser.add_argument("--std_dev", type=float, default=1, help="number of standard deviations plotted for the covariance")

subparsers = parser.add_subparsers(dest='subparser_name')
parser_eval = subparsers.add_parser('eval')

parser_eval.add_argument('actionfile', type=str)
parser_eval.add_argument("reg_type", type=str, choices=['segment', 'rpm', 'bij'], default='bij')
parser_eval.add_argument("--downsample_size", type=float, default=0.025)

parser_eval.add_argument("--fake_data_segment",type=str, default='knot2handstester0_seg00')
parser_eval.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
    default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")
parser_eval.add_argument("--gpu", action="store_true", default=False)

parser_eval.add_argument("--beta_pos", type=float, default=1000000.0)
parser_eval.add_argument("--beta_rot", type=float, default=100.0)
parser_eval.add_argument("--gamma", type=float, default=1000.0)
parser_eval.add_argument("--use_collision_cost", type=int, default=1)

parser_eval.add_argument("--pos_coef", type=float, default=1, help="coefficient for dtw position cost")
parser_eval.add_argument("--rot_coef", type=float, default=.1, help="coefficient for dtw rotation cost")
parser_eval.add_argument("--pos_vel_coef", type=float, default=0, help="coefficient for dtw position velocity cost")
parser_eval.add_argument("--rot_vel_coef", type=float, default=0, help="coefficient for dtw rotation velocity cost")
parser_eval.add_argument("--force_coef", type=float, default=1, help="coefficient for dtw force cost")
parser_eval.add_argument("--torque_coef", type=float, default=1, help="coefficient for dtw torque cost")

parser_eval.add_argument("--downsample_traj", type=int, default=1, help="downsample demonstration trajectory by this factor")
parser_eval.add_argument("--max_num_demos", type=int, default=10, help="maximum number of demos to combine")

args = parser.parse_args()

#if args.fake_data_segment is None: assert args.execution==1

###################





"""
Workflow:
1. Fake data + animation only
    --fake_data_segment=xxx --execution=0
2. Fake data + Gazebo. Set Gazebo to initial state of fake data segment so we'll execute the same thing.
    --fake_data_segment=xxx --execution=1
    This is just so we know the robot won't do something stupid that we didn't catch with openrave only mode.
3. Real data + Gazebo
    --execution=1 
    The problem is that the gazebo robot is in a different state from the real robot, in particular, the head tilt angle. TODO: write a script that       sets gazebo head to real robot head
4. Real data + Real execution.
    --execution=1

The question is, do you update the robot's head transform.
If you're using fake data, don't update it.

"""





from old_rapprentice import math_utils as mu

try:
    from old_rapprentice import pr2_trajectories, PR2, robot_states
    import rospy
except ImportError:
    print "Couldn't import ros stuff"

import cloudprocpy, trajoptpy, openravepy
import os, numpy as np, h5py, time
from numpy import asarray
import numpy as np
import cPickle as pickle
import importlib
from std_msgs.msg import Float64MultiArray, String, Float32
from old_rapprentice import registration, colorize, berkeley_pr2, \
     animate_traj, ros2rave, plotting_openrave, task_execution, \
     planning, tps, func_utils, resampling, clouds, conversions as conv, retiming, \
     robot_states
import analyze_data as analyze_data

from core import sim_util, demonstration
from core.demonstration import SceneState, AugmentedTrajectory, Demonstration

cloud_proc_mod = importlib.import_module(args.cloud_proc_mod)
cloud_proc_func = getattr(cloud_proc_mod, args.cloud_proc_func)


import IPython
        
    
def redprint(msg):
    print colorize.colorize(msg, "red", bold=True)
    

def print_matrix(m):
    for i in range(m.shape[0]):
        print [str(m[i,j]) for j in range(m.shape[1])]


def set_gripper_maybesim(lr, value):
    gripper = {"l":Globals.pr2.lgrip, "r":Globals.pr2.rgrip}[lr]
    gripper.set_angle(value)

class Globals:
    robot = None
    env = None
    torso_val = None

    pr2 = None
    pub = None

def main():
                        

    if args.execution:
        rospy.init_node("exec_task",disable_signals=True)
        Globals.pr2 = PR2.PR2()
        Globals.env = Globals.pr2.env
        Globals.robot = Globals.pr2.robot
        
    else:
        Globals.env = openravepy.Environment()
        Globals.env.StopSimulation()
        Globals.env.Load("robots/pr2-beta-static.zae")    
        Globals.robot = Globals.env.GetRobots()[0]

    if not args.eval.fake_data_segment:
        grabber = cloudprocpy.CloudGrabber()
        grabber.startRGBD()

    Globals.viewer = trajoptpy.GetViewer(Globals.env)


    
    
    demos = analyze_data.setup_demos(args)
    trajoptpy.SetInteractive(args.interactive)
    lfd_env, sim = analyze_data.setup_lfd_environment_sim(args, demos)
    reg_factory = analyze_data.setup_registration(args, demos, sim)
    


    # for now, use the cloud of the first demo as the current cloud
    if args.pr2:
        grabber = cloudprocpy.CloudGrabber()
        grabber.startRGBD()
        rgb, depth = grabber.getRGBD()
        T_w_k = berkeley_pr2.get_kinect_transform(Globals.robot)
        full_cloud = cloud_proc_func(rgb, depth, T_w_k)
    else:
        full_cloud = demos.values()[0].scene_state.cloud

    scene_state = demonstration.SceneState(full_cloud, downsample_size=args.eval.downsample_size)

    regs, demos = analyze_data.register_scenes(reg_factory, scene_state)
    
    trajectory_transferer = analyze_data.MultipleDemosPoseTrajectoryTransferer(sim, args.eval.pos_coef, args.eval.rot_coef, args.eval.pos_vel_coef, args.eval.rot_vel_coef, args.eval.force_coef, args.eval.torque_coef, 
                                                                  args.eval.beta_pos, args.eval.beta_rot, args.eval.gamma, args.eval.use_collision_cost, 
                                                                  downsample_traj=args.eval.downsample_traj)
    n_demos = min(args.eval.max_num_demos, len(reg_factory.demos))
    aug_traj = trajectory_transferer.transfer(regs[:n_demos], demos[:n_demos], plotting=args.animation, plotting_std_dev=args.std_dev)
    
    

    
    #import cPickle as pickle
    """
    data_file = open("data.pickle", 'wa')
    data = {}
    data['cloud'] = full_cloud
    data['traj'] = aug_traj
    pickle.dump(data, data_file)
    data_file.close()
    IPython.embed()
    """
    """
    data_file = open("data.pickle", 'r')
    data = pickle.load(data_file)
    aug_traj = data['traj']
    full_cloud = data['cloud']


    IPython.embed()
    """



    covariance = {}
    covariances = {}

    covariances['l'] = []
    covariances['r'] = []
    
    import h5py
    right = h5py.File("../../../Downloads/covars_left.h5")
    covariances['r'] = right['covars']
    left = h5py.File("../../../Downloads/covars.h5")
    covariances['l'] = left['covars']
    


    """
    import cPickle
    data_file = open("costs.pkl", 'r')
    data = pickle.load(data_file)
    lr2Cts = data['lr2Cts']
    lr2cts = data['lr2cts']
    covariances['l'] = lr2Cts['l']
    covariances['r'] = lr2Cts['r']
    """
    length_part = min(covariances['r'].shape[0], covariances['l'].shape[0], aug_traj.lr2dof_mu_traj['r'].shape[0], aug_traj.lr2arm_traj['r'].shape[0], aug_traj.lr2close_finger_traj['l'].shape[0], aug_traj.lr2open_finger_traj['l'].shape[0], aug_traj.lr2dof_sigma_traj['l'].shape[0])


    gripper_part = []
    for i in range(length_part):
        if aug_traj.lr2close_finger_traj['r'][i]:
            gripper_part.append(-1.0)
        elif aug_traj.lr2open_finger_traj['r'][i]:
            gripper_part.append(1.0)
        elif aug_traj.lr2close_finger_traj['l'][i]:
            gripper_part.append(-2.0)
        elif aug_traj.lr2open_finger_traj['r'][i]:
            gripper_part.append(1.0)
        else:
            gripper_part.append(0.0)


    """
    Covariance diag averages for when the arm is moving freely
         5.67193641e-03,   1.29050216e-02,   1.31052249e-03,
         3.02826704e-01,   1.15117729e-01,   2.49747355e-01,
         4.78830682e-07,   4.51310940e-07,   6.50562967e-08,
         4.61426095e-07,   6.83632264e-07,   3.60633106e-06,
         4.29387966e+01,   2.18745951e+01,   9.47029488e+00,
         2.95483087e-01,   8.96258053e-01,   3.61227108e-02
    """




    #covariances from averages of freely moving arm, with multipler

    """
    for lr in 'lr':
        covariance[lr] = np.diag([6e-3, 1e-2, 1e-3, 3e-1, 1e-1, 2e-1, 5e-7, 5e-7, 7e-8, 5e-7, 7e-7, 4e-6, 4e1, 2e1, 9, 3e-1, 9e-1, 4e-2])

    covariances['l'] = [covariance['l'] + lam * np.eye(18) for i in range(length_part)]
    covariances['r'] = [covariance['r'] + lam * np.eye(18) for i in range(length_part)]
    """





    """
    pad = 1e-1 * np.eye(18)
    pad[6:12, 6:12] = pad[6:12, 6:12] + 1.0*np.eye(6)
    for i in range(length_part):
        covariances['l'].append(np.diag(np.diag(aug_traj.lr2dof_sigma_traj['l'][i])) + pad)
        covariances['r'].append(np.diag(np.diag(aug_traj.lr2dof_sigma_traj['r'][i])) + pad)
    """
    




    covariances['r'] = covariances['r'][0:length_part]
    covariances['l'] = covariances['l'][0:length_part]

    F_part = {}
    traj_part = {}
    for lr in 'lr':
        F_part[lr] = aug_traj.lr2dof_mu_traj[lr][0:length_part,12:18]
        traj_part[lr] = aug_traj.lr2arm_traj[lr][0:length_part,:]


    redprint("Press enter to use current position as starting point")
    raw_input()
    arm_positions = {}
    (arm_position, arm_velocity, arm_effort) = robot_states.call_return_joint_states(robot_states.r_arm_joint_names)
    arm_positions['r'] = arm_position
    diff_r = np.array(arm_position - traj_part['r'][0,:])
    maximum_r = max(abs(diff_r))
    (arm_position, arm_velocity, arm_effort) = robot_states.call_return_joint_states(robot_states.l_arm_joint_names)
    arm_positions['l'] = arm_position
    diff_l = np.array(arm_position - traj_part['l'][0,:])
    maximum_l = max(abs(diff_l))
    maximum = max(maximum_l, maximum_r)

    speed = (20.0/360.0*2*(np.pi))
    time_needed = maximum / speed
    
    F = {}
    traj = {}
    gripper = {}
    for lr in 'lr':
        initial_pos_traj = mu.interp2d(np.arange(0, time_needed, 0.01), np.array([0,time_needed]), np.array([arm_positions[lr], traj_part[lr][0,:]]))
        initial_force_traj = np.array([np.zeros(6) for i in range(initial_pos_traj.shape[0])])
        initial_traj_length = initial_pos_traj.shape[0]

        traj[lr] = np.concatenate((initial_pos_traj, traj_part[lr]), axis=0)
        F[lr] = np.concatenate((initial_force_traj, F_part[lr]), axis=0)
        
    initial_gripper_traj = np.array([0.0 for i in range(initial_pos_traj.shape[0])])
    gripper = np.concatenate((initial_gripper_traj, gripper_part), axis=0)

    if args.visualize:
        demofile = h5py.File(args.h5file, 'r')
        seg_info = demofile[demofile.keys()[0]]
        r2r = ros2rave.RosToRave(Globals.robot, asarray(seg_info["joint_states"]["name"]))
        r2r.set_values(Globals.robot, asarray(seg_info["joint_states"]["position"][200]))

        for i in range(0, traj['l'].shape[0], 30):
            for lr in 'lr':
                handles = []
                handles.append(Globals.env.plot3(full_cloud,5,np.array([(0,1,0,1) for x in full_cloud])))
                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                vals = traj[lr][i,:]
                Globals.robot.SetDOFValues(vals, Globals.robot.GetManipulator(manip_name).GetArmIndices())
            
                hmats_index = {"l":-28, "r":-3}
                hmats = Globals.robot.GetLinkTransformations()
                trans, rot = conv.hmat_to_trans_rot(hmats[hmats_index[lr]])
                f_start = np.array([0,0,0]) + trans
                f_end = F[lr][i][0:3]/100 + trans
                handles.append(Globals.env.drawlinestrip(np.array([f_start, f_end]), 10, (1,0,0,1)))
                Globals.viewer.Step()
            
            redprint(i)
            Globals.viewer.Idle()




    pgains = np.asarray([2400.0, 1200.0, 1000.0, 700.0, 300.0, 300.0, 300.0])
    dgains = np.asarray([18.0, 10.0, 6.0, 4.0, 6.0, 4.0, 4.0])
    pgainsdiag = np.diag(np.asarray([-2400.0, -1200.0, -1000.0, -700.0, -300.0, -300.0, -300.0]))
    dgainsdiag = np.diag(np.asarray([-18.0, -10.0, -6.0, -4.0, -6.0, -4.0, -4.0]))
    m = np.array([3.33, 1.16, 0.1, 0.25, 0.133, 0.0727, 0.0727]) * 2 # masses in joint space (feed forward)


    length_total = F['l'].shape[0]


    costs = {}
    JKpJ = {}
    JKvJ = {}

    for lr in 'lr':
        redprint(lr)
        costs[lr] = []
        JKpJ[lr] = []
        JKvJ[lr] = []

        for i in range(length_part):
            #costs[lr].append(np.linalg.inv(covariances[lr][i]))
            costs[lr].append(covariances[lr][i])

        Ms = []
        allCs = []

        manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
        arm = Globals.robot.GetManipulator(manip_name)
        jacobians = []
        for i in range(traj[lr].shape[0]):
            vals = traj[lr][i,:]
            Globals.robot.SetDOFValues(vals, Globals.robot.GetManipulator(manip_name).GetArmIndices())
            jacobians.append(np.vstack((arm.CalculateJacobian(), arm.CalculateAngularVelocityJacobian())))
        jacobians = asarray(jacobians)

        for t in range(length_part):
            M = np.zeros((18, 21))
            J = jacobians[t]
            M[0:6,0:7] = J
            M[6:12,7:14] = J
            mdiag = np.diag(m) # Convert joint feedforward values into diagonal array
            mdiag_inv = np.linalg.inv(mdiag)
            M[12:18,14:21] = np.linalg.inv((J.dot(mdiag_inv).dot(np.transpose(J)))).dot(J).dot(mdiag_inv)
            Ms.append(M)

        for t in range(length_part):
            topad = np.zeros((21,21))
            topad[0:7,0:7] = np.diag(pgains) * 0.001
            topad[7:14,7:14] = np.diag(dgains) * 0.001
            topad[14:21,14:21] = np.eye(7) * 0.001
            #allCs.append(np.transpose(Ms[t]).dot(Ct).dot(Ms[t]) + topad)
            allCs.append(np.transpose(Ms[t]).dot(costs[lr][t]).dot(Ms[t]) + topad)


        Kps = []
        Kvs = []
        Ks = []
        Qt = None
        Vs = []
        for t in range(length_part-1, -1, -1):
            if Qt is None:
                Qt = allCs[t]
            else:
                Ft = np.zeros((14, 21))
                for j in range(14):
                    Ft[j][j] = 1.0
                deltat = 0.01
                for j in range(7):
                    Ft[j][j+7] = deltat
                for j in range(7):
                    Ft[j+7][j+14] = deltat/m[j]  
                for j in range(7):
                    Ft[j][j+14] = ((deltat)**2)/m[j]
                Qt = allCs[t] + (np.transpose(Ft).dot(Vs[length_part-2-t]).dot(Ft))
            Qxx = Qt[0:14, 0:14]
            Quu = Qt[14:21, 14:21]
            Qxu = Qt[0:14, 14:21]
            Qux = Qt[14:21, 0:14]
            Quuinv = np.linalg.inv(Quu)
            Vt = Qxx - Qxu.dot(Quuinv).dot(Qux)
            Vt = 0.5*(Vt + np.transpose(Vt))
            Kt = -1*(Quuinv.dot(Qux))
            Ks.append(Kt)
            Kps.append(Kt[:, 0:7])
            Kvs.append(Kt[:, 7:14])
            Vs.append(Vt)

        Kps = Kps[::-1]
        Kvs = Kvs[::-1]


        JKpJ[lr] = np.asarray(Kps)
        JKvJ[lr] = np.asarray(Kvs)


    # Pad initial traj with PD gains
    addkp = np.asarray([pgainsdiag for i in range(initial_traj_length)])
    addkv = np.asarray([dgainsdiag for i in range(initial_traj_length)])
    for lr in 'lr':
        JKpJ[lr] = np.concatenate((JKpJ[lr], addkp), axis=0)
        JKvJ[lr] = np.concatenate((JKvJ[lr], addkv), axis=0)

    Kps = []
    Kvs = []
    for i in range(length_total):
        Kps.append(np.zeros((6,6)))
        Kvs.append(np.zeros((6,6)))


    Kps = np.asarray(Kps)
    Kvs = np.asarray(Kvs)

    if args.force:
        for lr in 'lr':
            JKpJ[lr] = np.asarray([pgainsdiag/args.multiplier for i in range(length_total)])
            JKvJ[lr] = np.asarray([dgainsdiag/args.multiplier for i in range(length_total)])


    if args.kinematics:
        for lr in 'lr':
            JKpJ[lr] = np.asarray([pgainsdiag for i in range(length_total)])
            JKvJ[lr] = np.asarray([dgainsdiag for i in range(length_total)])
            F[lr] = np.asarray([np.zeros((6, 1)) for i in range(length_total)])
    


    # Not testing one arm
    """
    for lr in 'r':
        JKpJ[lr] = np.asarray([pgainsdiag for i in range(length_total)])
        JKvJ[lr] = np.asarray([dgainsdiag for i in range(length_total)])
    """

    Kps = np.resize(Kps, (1, 36 * length_total))[0]
    Kvs = np.resize(Kvs, (1, 36 * length_total))[0]
    

    
    
    for lr in 'lr':
        JKvJ[lr] = np.resize(JKvJ[lr], (1, 49*length_total))[0]
        JKpJ[lr] = np.resize(JKpJ[lr], (1, 49*length_total))[0]
        traj[lr] = np.resize(traj[lr], (1, traj[lr].shape[0]*7))[0] 
        F[lr] = np.resize(F[lr], (1, F[lr].shape[0]*6))[0] 
    gripper = np.resize(gripper, (1, gripper.shape[0]))[0]


    # [traj, Kp, Kv, F, use_force, seconds]
    data = np.zeros((1, length_total*(7+49+49+6+36+36+7+49+49+6+36+36+1)+2))
    data[0][0:length_total*7] = traj['r']
    data[0][length_total*7:length_total*(7+49)] = JKpJ['r']
    data[0][length_total*(7+49):length_total*(7+49+49)] = JKvJ['r']
    data[0][length_total*(7+49+49):length_total*(7+49+49+6)] = F['r']
    data[0][length_total*(7+49+49+6):length_total*(7+49+49+6+36)] = Kps
    data[0][length_total*(7+49+49+6+36):length_total*(7+49+49+6+36+36)] = Kvs
    data[0][length_total*(7+49+49+6+36+36):length_total*(7+49+49+6+36+36+7)] = traj['l']
    data[0][length_total*(7+49+49+6+36+36+7):length_total*(7+49+49+6+36+36+7+49)] = JKpJ['l']
    data[0][length_total*(7+49+49+6+36+36+7+49):length_total*(7+49+49+6+36+36+7+49+49)] = JKvJ['l']
    data[0][length_total*(7+49+49+6+36+36+7+49+49):length_total*(7+49+49+6+36+36+7+49+49+6)] = F['l']
    data[0][length_total*(7+49+49+6+36+36+7+49+49+6):length_total*(7+49+49+6+36+36+7+49+49+6+36)] = Kps
    data[0][length_total*(7+49+49+6+36+36+7+49+49+6+36):length_total*(7+49+49+6+36+36+7+49+49+6+36+36)] = Kvs
    data[0][length_total*(7+49+49+6+36+36+7+49+49+6+36+36):length_total*(7+49+49+6+36+36+7+49+49+6+36+36+1)] = gripper
    data[0][-2] = 1.0
    data[0][-1] = 1.0
    msg = Float64MultiArray()
    msg.data = data[0].tolist()
    pub = rospy.Publisher("/controller_data", Float64MultiArray)
    redprint("Press enter to start trajectory")
    raw_input()
    for lr in 'lr':
        set_gripper_maybesim(lr, 0.08)
    time.sleep(5)
    pub.publish(msg)
    listener()





def callback(msg):
    if msg.data == 1.0:
        set_gripper_maybesim('r', 0.08)
    elif msg.data == -1.0:
        set_gripper_maybesim('r', 0.0)
    elif msg.data == 2.0:
        set_gripper_maybesim('l', 0.08)
    elif msg.data == -2.0:
        set_gripper_maybesim('l', 0.0)
    
def listener():
    rospy.Subscriber("/my_controller_name/gripper_trajectory", Float32, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



"""
            redprint("Press enter to set gripper")
            raw_input()
            for lr in 'lr':
                set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))

"""
if __name__ == "__main__":

    main()
