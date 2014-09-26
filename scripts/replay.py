#!/usr/bin/env python
import subprocess, signal
from rapprentice.colorize import colorize
import time, os, shutil
from rapprentice.call_and_print import call_and_print
from rapprentice.yes_or_no import yes_or_no
from rapprentice import robot_states
import os.path as osp
import itertools
import yaml
import argparse
from std_msgs.msg import Float64MultiArray, String
import IPython

usage="""

Run in simulation with a translation and a rotation of fake data:
./do_task.py ~/Data/sampledata/overhand/overhand.h5 --fake_data_segment=overhand0_seg00 --execution=0  --animation=1 --select_manual --fake_data_transform .1 .1 .1 .1 .1 .1

Run in simulation choosing the closest demo, single threaded
./do_task.py ~/Data/all.h5 --fake_data_segment=demo1-seg00 --execution=0  --animation=1  --parallel=0 

Actually run on the robot without pausing or animating 
./do_task.py ~/Data/overhand2/all.h5 --execution=1 --animation=0

"""
parser = argparse.ArgumentParser(usage=usage)
parser.add_argument("h5file", type=str)
parser.add_argument("demo_prefix")
parser.add_argument("--downsample", default=3, type=int)
parser.add_argument("--new_demo", default="None")
parser.add_argument("--cloud_proc_func", default="extract_red")
parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")
    
parser.add_argument("--execution", type=int, default=0)
parser.add_argument("--animation", type=int, default=0)
parser.add_argument("--parallel", type=int, default=1)
parser.add_argument("--useHenry", type=int, default=1)
parser.add_argument("--prompt", action="store_true")
parser.add_argument("--show_neighbors", action="store_true")
parser.add_argument("--select_manual", action="store_true")
parser.add_argument("--log", action="store_true")

parser.add_argument("--fake_data_segment",type=str)
parser.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
    default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")

parser.add_argument("--interactive",action="store_true")

args = parser.parse_args()

if args.fake_data_segment is None: assert args.execution==1

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


from rapprentice import registration, colorize, berkeley_pr2, \
     animate_traj, ros2rave, plotting_openrave, task_execution, \
     planning, tps, func_utils, resampling, clouds
from rapprentice import math_utils as mu
from rapprentice.yes_or_no import yes_or_no

try:
    from rapprentice import pr2_trajectories, PR2
    import rospy
except ImportError:
    print "Couldn't import ros stuff"

import cloudprocpy, trajoptpy, openravepy
import os, numpy as np, h5py, time
from numpy import asarray
import importlib

cloud_proc_mod = importlib.import_module(args.cloud_proc_mod)
cloud_proc_func = getattr(cloud_proc_mod, args.cloud_proc_func)
        
    
def redprint(msg):
    print(msg)
    
def split_trajectory_by_gripper(seg_info):
    rgrip = asarray(seg_info["r_gripper_joint"])
    lgrip = asarray(seg_info["l_gripper_joint"])

    thresh = .04 # open/close threshold

    n_steps = len(lgrip)


    # indices BEFORE transition occurs
    l_openings = np.flatnonzero((lgrip[1:] >= thresh) & (lgrip[:-1] < thresh))
    r_openings = np.flatnonzero((rgrip[1:] >= thresh) & (rgrip[:-1] < thresh))
    l_closings = np.flatnonzero((lgrip[1:] < thresh) & (lgrip[:-1] >= thresh))
    r_closings = np.flatnonzero((rgrip[1:] < thresh) & (rgrip[:-1] >= thresh))

    before_transitions = np.r_[l_openings, r_openings, l_closings, r_closings]
    after_transitions = before_transitions+1
    seg_starts = np.unique(np.r_[0, after_transitions])
    seg_ends = np.unique(np.r_[before_transitions, n_steps-1])

    return seg_starts, seg_ends

def binarize_gripper(angle):
    open_angle = .08
    closed_angle = 0    
    thresh = .04
    if angle > thresh: return open_angle
    else: return closed_angle


    

    
def set_gripper_maybesim(lr, value):
    if args.execution:
        gripper = {"l":Globals.pr2.lgrip, "r":Globals.pr2.rgrip}[lr]
        gripper.set_angle(value)
        Globals.pr2.join_all()
    else:
        Globals.robot.SetDOFValues([value*5], [Globals.robot.GetJoint("%s_gripper_l_finger_joint"%lr).GetDOFIndex()])
    return True
        
def exec_traj_maybesim(bodypart2traj):
    # if args.animation:
    #     dof_inds = []
    #     trajs = []
    #     for (part_name, traj) in bodypart2traj.items():
    #         manip_name = {"larm":"leftarm","rarm":"rightarm"}[part_name]
    #         dof_inds.extend(Globals.robot.GetManipulator(manip_name).GetArmIndices())            
    #         trajs.append(traj)
    #     full_traj = np.concatenate(trajs, axis=1)
    #     Globals.robot.SetActiveDOFs(dof_inds)
    #     animate_traj.animate_traj(full_traj, Globals.robot, restore=False,pause=True)
    if args.execution:
        #if not args.prompt or yes_or_no("execute?"):
        pr2_trajectories.follow_body_traj(Globals.pr2, bodypart2traj)
        #else:
        #    return False

    return True


def find_closest_manual(demofile, _new_xyz):
    "for now, just prompt the user"
    seg_names = demofile.keys()
    
    choice_ind = 0
    chosen_seg = seg_names[choice_ind] 
    return chosen_seg

def registration_cost(xyz0, xyz1):
    scaled_xyz0, _ = registration.unit_boxify(xyz0)
    scaled_xyz1, _ = registration.unit_boxify(xyz1)
    f,g = registration.tps_rpm_bij(scaled_xyz0, scaled_xyz1, rot_reg=1e-3, n_iter=30)
    cost = registration.tps_reg_cost(f) + registration.tps_reg_cost(g)
    return cost

DS_SIZE = .025

@func_utils.once
def get_downsampled_clouds(demofile):
    return [clouds.downsample(seg["cloud_xyz"], DS_SIZE) for seg in demofile.values()]

def find_closest_auto(demofile, new_xyz):
    if args.parallel:
        from joblib import Parallel, delayed
    demo_clouds = [asarray(seg["cloud_xyz"]) for seg in demofile.values()]
    keys = demofile.keys()
    if args.parallel:
        costs = Parallel(n_jobs=3,verbose=100)(delayed(registration_cost)(demo_cloud, new_xyz) for demo_cloud in demo_clouds)
    else:
        costs = []
        for (i,ds_cloud) in enumerate(demo_clouds):
            costs.append(registration_cost(ds_cloud, new_xyz))
            print "completed %i/%i"%(i+1, len(demo_clouds))
    
    print "costs\n",costs
    if args.show_neighbors:
        nshow = min(5, len(keys))
        import cv2, rapprentice.cv_plot_utils as cpu
        sortinds = np.argsort(costs)[:nshow]
        near_rgbs = [asarray(demofile[keys[i]]["rgb"]) for i in sortinds]
        bigimg = cpu.tile_images(near_rgbs, 1, nshow)
        cv2.imshow("neighbors", bigimg)
        print "press any key to continue"
        cv2.waitKey()
        
    ibest = np.argmin(costs)
    return keys[ibest]
            
            
def arm_moved(joint_traj):    
    if len(joint_traj) < 2: return False
    return ((joint_traj[1:] - joint_traj[:-1]).ptp(axis=0) > .01).any()
        
def tpsrpm_plot_cb(x_nd, y_md, targ_Nd, corr_nm, wt_n, f):
    ypred_nd = f.transform_points(x_nd)
    handles = []
    handles.append(Globals.env.plot3(ypred_nd, 3, (0,1,0)))
    handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, x_nd.min(axis=0), x_nd.max(axis=0), xres = .1, yres = .1, zres = .04))
    Globals.viewer.Step()


def unif_resample(traj, max_diff, wt = None):        
    """
    Resample a trajectory so steps have same length in joint space    
    """
    import scipy.interpolate as si
    tol = .005
    if wt is not None: 
        wt = np.atleast_2d(wt)
        traj = traj*wt
        
        
    dl = mu.norms(traj[1:] - traj[:-1],1)
    l = np.cumsum(np.r_[0,dl])
    goodinds = np.r_[True, dl > 1e-8]
    deg = min(3, sum(goodinds) - 1)
    if deg < 1: return traj, np.arange(len(traj))
    
    nsteps = max(int(np.ceil(float(l[-1])/max_diff)),2)
    newl = np.linspace(0,l[-1],nsteps)

    ncols = traj.shape[1]
    colstep = 10
    traj_rs = np.empty((nsteps,ncols)) 
    for istart in xrange(0, traj.shape[1], colstep):
        (tck,_) = si.splprep(traj[goodinds, istart:istart+colstep].T,k=deg,s = tol**2*len(traj),u=l[goodinds])
        traj_rs[:,istart:istart+colstep] = np.array(si.splev(newl,tck)).T
    if wt is not None: traj_rs = traj_rs/wt

    newt = np.interp(newl, l, np.arange(len(traj)))

    return traj_rs, newt


###################


class Globals:
    robot = None
    env = None

    pr2 = None

def main():
    demofile = h5py.File(args.h5file, 'r')
    
    trajoptpy.SetInteractive(args.interactive)

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

    if not args.fake_data_segment:
        grabber = cloudprocpy.CloudGrabber()
        grabber.startRGBD()

    Globals.viewer = trajoptpy.GetViewer(Globals.env)

    #####################
    
    started_bag = False
    started_video = False

    localtime   = time.localtime()
    time_string  = time.strftime("%Y-%m-%d-%H-%M-%S", localtime)

    os.chdir(osp.dirname(args.new_demo))


    if not osp.exists(args.new_demo):
        yn = yes_or_no("master file does not exist. create?")
        basename = raw_input("what is the base name?\n").strip()
        if yn:
            with open(args.new_demo,'w') as fh: fh.write("""
    name: %s
    h5path: %s
    bags:
            """%(basename, basename+".h5"))
        else:
            print "exiting."
            exit(0)
    with open(args.new_demo, "r") as fh: master_info = yaml.load(fh)
    if master_info["bags"] is None: master_info["bags"] = []
    for suffix in itertools.chain("", (str(i) for i in itertools.count())):
        demo_name = args.demo_prefix + suffix
        if not any(bag["demo_name"] == demo_name for bag in master_info["bags"]):
            break
        print demo_name

    timestampfile = demo_name+"timestamps.txt"
    fht = open(timestampfile,"w")
    try:    
        bag_cmd = "rosbag record /joint_states /joy -O %s"%demo_name
        #print colorize(bag_cmd, "green")
        bag_handle = subprocess.Popen(bag_cmd, shell=True)
        time.sleep(1)
        poll_result = bag_handle.poll() 
        print "poll result", poll_result
        if poll_result is not None:
            raise Exception("problem starting video recording")
        else: started_bag = True
        
        video_cmd = "record_rgbd_video --out=%s --downsample=%i"%(demo_name, args.downsample)
        #print colorize(video_cmd, "green")
        video_handle = subprocess.Popen(video_cmd, shell=True)
        started_video = True


       

        
        
            
        
            
            #grab_end(new_xyz)
        fht.write('look:' + str(rospy.get_rostime().secs))
        
        
        ################################    
        redprint("Finding closest demonstration")
       
        seg_name = find_closest_manual(demofile, None)
        
        
        seg_info = demofile[seg_name]
        

        redprint("Generating end-effector trajectory")    

        old_xyz = np.squeeze(seg_info["cloud_xyz"])
        

        scaled_old_xyz, src_params = registration.unit_boxify(old_xyz)
        

        link2eetraj = {}
        for lr in 'lr':
            link_name = "%s_gripper_tool_frame"%lr
            old_ee_traj = asarray(seg_info[link_name]["hmat"])
           
    
        miniseg_starts, miniseg_ends = split_trajectory_by_gripper(seg_info)    
        success = True
        for (i_miniseg, (i_start, i_end)) in enumerate(zip(miniseg_starts, miniseg_ends)):
            
            if args.execution=="real": Globals.pr2.update_rave()


            ################################    
            redprint("Generating joint trajectory for segment %s, part %i"%(seg_name, i_miniseg))
            
            # figure out how we're gonna resample stuff
            lr2oldtraj = {}
            for lr in 'lr':
                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]                 
                old_joint_traj = asarray(seg_info[manip_name][i_start:i_end+1])
                
                #print (old_joint_traj[1:] - old_joint_traj[:-1]).ptp(axis=0), i_start, i_end
                lr2oldtraj[lr] = old_joint_traj   

        
            ### Generate fullbody traj
            bodypart2traj = {}            
            for (lr,old_joint_traj) in lr2oldtraj.items():

                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                 
                #old_joint_traj_rs = mu.interp2d(timesteps_rs, np.arange(len(old_joint_traj)), old_joint_traj)

                ee_link_name = "%s_gripper_tool_frame"%lr
                #new_ee_traj = link2eetraj[ee_link_name][i_start:i_end+1]          
                #new_ee_traj_rs = resampling.interp_hmats(timesteps_rs, np.arange(len(new_ee_traj)), new_ee_traj)
                if args.execution: Globals.pr2.update_rave()
                #new_joint_traj = planning.plan_follow_traj(Globals.robot, manip_name,
                # Globals.robot.GetLink(ee_link_name), new_ee_traj_rs,old_joint_traj_rs)
                part_name = {"l":"larm", "r":"rarm"}[lr]
                bodypart2traj[part_name] = old_joint_traj
                
            traj = {}

            for lr in 'lr':
                part_name = {"l":"larm", "r":"rarm"}[lr]
                traj[lr] = bodypart2traj[part_name]

            if i_miniseg ==0:
                
                redprint("Press enter to use current position as starting point")
                raw_input()
                arm_positions = {}
                (arm_position, arm_velocity, arm_effort) = robot_states.call_return_joint_states(robot_states.r_arm_joint_names)
                arm_positions['r'] = arm_position
                diff_r = np.array(arm_position - traj['r'][0,:])
                maximum_r = max(abs(diff_r))
                (arm_position, arm_velocity, arm_effort) = robot_states.call_return_joint_states(robot_states.l_arm_joint_names)
                arm_positions['l'] = arm_position
                diff_l = np.array(arm_position - traj['l'][0,:])
                maximum_l = max(abs(diff_l))
                maximum = max(maximum_l, maximum_r)

                speed = (20.0/360.0*2*(np.pi))
                time_needed = maximum / speed
                
                
                initial_pos_traj = {}
                
                for lr in 'lr':
                    part_name = {"l":"larm", "r":"rarm"}[lr]
                    initial_pos_traj[part_name] = mu.interp2d(np.arange(0, time_needed, 0.01), np.array([0,time_needed]), np.array([arm_positions[lr], traj[lr][0,:]]))
                
                #initial_traj_length = initial_pos_traj.shape[0]

                #traj[lr] = np.concatenate((initial_pos_traj, traj_part[lr]), axis=0)
               
                #=======================send to controller======================
                length_total = initial_pos_traj["larm"].shape[0]
                qs_l = np.resize(initial_pos_traj["larm"], (1, initial_pos_traj["larm"].shape[0]*7))[0] #resize the data to 1x7n (n being the number of steps)
                qs_r = np.resize(initial_pos_traj["rarm"], (1, initial_pos_traj["rarm"].shape[0]*7))[0]
                #F = np.array(np.matrix(F).T) # 6 x n
                F = np.zeros((length_total,6))
                F = np.resize(F, (1, F.shape[0]*6))[0] #resize the data to 1x3n
                gripper = np.zeros((length_total,1))
                gripper = np.resize(gripper, (1, gripper.shape[0]*1))[0]
                # Controller code in joint space

                pgains = np.asarray([2400.0, 1200.0, 1000.0, 700.0, 300.0, 300.0, 300.0])
                dgains = np.asarray([18.0, 10.0, 6.0, 4.0, 6.0, 4.0, 4.0])
                # Gains as Kps and Kvs for testing

                Kps = []
                Kvs = []
                for i in range(length_total):
                    Kps.append(np.zeros((6,6)))
                    Kvs.append(np.zeros((6,6)))
                toAddJkpJ = np.diag(np.asarray([-2400.0, -1200.0, -1000.0, -700.0, -300.0, -300.0, -300.0]))
                toAddJkvJ = np.diag(np.asarray([-18.0, -10.0, -6.0, -4.0, -6.0, -4.0, -4.0]))
                #toAddJkvJ = np.diag(np.asarray([0, 0, 0, 0, 0, 0, 0]))
                #length = complete_force_traj.shape[0]
                JKpJ = np.asarray([toAddJkpJ for i in range(length_total)])
                JKpJ = np.resize(JKpJ, (1, 49*length_total))[0]

                JKvJ = np.asarray([toAddJkvJ for i in range(length_total)])
                JKvJ = np.resize(JKvJ, (1, 49*length_total))[0]
                

                # [traj, Kp, Kv, F, use_force, seconds]
                
                Kps = np.resize(Kps, (1, 36 * length_total))[0]
                Kvs = np.resize(Kvs, (1, 36 * length_total))[0]

                
                #JKpJ = np.resize(JKpJ, (1, 49 * num_steps))[0]
                #JKvJ = np.resize(JKvJ, (1, 49 * num_steps))[0]
                stamps = asarray(seg_info['stamps'])
                data = np.zeros((1, length_total*(7+49+49+6+36+36+7+49+49+6+36+36+1)+2))
                data[0][0:length_total*7] = qs_r
                data[0][length_total*7:length_total*(7+49)] = JKpJ
                data[0][length_total*(7+49):length_total*(7+49+49)] = JKvJ
                data[0][length_total*(7+49+49):length_total*(7+49+49+6)] = F
                data[0][length_total*(7+49+49+6):length_total*(7+49+49+6+36)] = Kps
                data[0][length_total*(7+49+49+6+36):length_total*(7+49+49+6+36+36)] = Kvs
                data[0][length_total*(7+49+49+6+36+36):length_total*(7+49+49+6+36+36+7)] = qs_l
                data[0][length_total*(7+49+49+6+36+36+7):length_total*(7+49+49+6+36+36+7+49)] = JKpJ
                data[0][length_total*(7+49+49+6+36+36+7+49):length_total*(7+49+49+6+36+36+7+49+49)] = JKvJ
                data[0][length_total*(7+49+49+6+36+36+7+49+49):length_total*(7+49+49+6+36+36+7+49+49+6)] = F
                data[0][length_total*(7+49+49+6+36+36+7+49+49+6):length_total*(7+49+49+6+36+36+7+49+49+6+36)] = Kps
                data[0][length_total*(7+49+49+6+36+36+7+49+49+6+36):length_total*(7+49+49+6+36+36+7+49+49+6+36+36)] = Kvs
                data[0][length_total*(7+49+49+6+36+36+7+49+49+6+36+36):length_total*(7+49+49+6+36+36+7+49+49+6+36+36+1)] = gripper
                data[0][-2] = 0
                data[0][-1] = stamps[i_end] - stamps[i_start]
                msg = Float64MultiArray()
                msg.data = data[0].tolist()
                pub = rospy.Publisher("/controller_data", Float64MultiArray)
                redprint("Press enter to start trajectory")
                raw_input()
                time.sleep(1)

                pub.publish(msg)
                time.sleep(1)


                #===================end send to controller=======================

                raw_input("CAME TO START POSITION")
                time.sleep(1)
                fht.write('\nstart:' + str(rospy.get_rostime().secs))

            ################################    
            redprint("Executing joint trajectory for segment %s, part %i using arms '%s'"%(seg_name, i_miniseg, bodypart2traj.keys()))
            if not args.useHenry:
                
                for lr in 'lr':
                    success &= set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))
                    # Doesn't actually check if grab occurred, unfortunately
                print('1')
                fht.write('\nstart:' + str(rospy.get_rostime().secs))
                print('2')
                if len(bodypart2traj) > 0:
                    success &= exec_traj_maybesim(bodypart2traj)
                print('3')
                fht.write('\nstop:' + str(rospy.get_rostime().secs))
                print('4')
            else:
                

                for lr in 'lr':
                    redprint("Press enter to set gripper")
                    raw_input()
                    set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))
                    # Doesn't actually check if grab occurred, unfortunately

                if bodypart2traj!={}:
                    length = i_end - i_start + 1
                    length_total = length
                    traj_r = traj['r']
                    qs_r = np.resize(traj_r, (1, traj_r.shape[0]*7))[0] #resize the data to 1x7n (n being the number of steps)

                    traj_l = traj['l']
                    qs_l = np.resize(traj_l, (1, traj_l.shape[0]*7))[0] #resize the data to 1x7n (n being the number of steps)
                    #F = np.array(np.matrix(F).T) # 6 x n
                    F = np.zeros((length_total,6))
                    F = np.resize(F, (1, F.shape[0]*6))[0] #resize the data to 1x3n
                    gripper = np.zeros((length_total,1))
                    gripper = np.resize(gripper, (1, gripper.shape[0]*1))[0]
                    # Controller code in joint space

                    pgains = np.asarray([2400.0, 1200.0, 1000.0, 700.0, 300.0, 300.0, 300.0])
                    dgains = np.asarray([18.0, 10.0, 6.0, 4.0, 6.0, 4.0, 4.0])
                    # Gains as Kps and Kvs for testing

                    Kps = []
                    Kvs = []
                    for i in range(length_total):
                        Kps.append(np.zeros((6,6)))
                        Kvs.append(np.zeros((6,6)))
                    toAddJkpJ = np.diag(np.asarray([-2400.0, -1200.0, -1000.0, -700.0, -300.0, -300.0, -300.0]))
                    toAddJkvJ = np.diag(np.asarray([-18.0, -10.0, -6.0, -4.0, -6.0, -4.0, -4.0]))
                    #toAddJkvJ = np.diag(np.asarray([0, 0, 0, 0, 0, 0, 0]))
                    #length = complete_force_traj.shape[0]
                    JKpJ = np.asarray([toAddJkpJ for i in range(length_total)])
                    JKpJ = np.resize(JKpJ, (1, 49*length_total))[0]

                    JKvJ = np.asarray([toAddJkvJ for i in range(length_total)])
                    JKvJ = np.resize(JKvJ, (1, 49*length_total))[0]
                    

                    # [traj, Kp, Kv, F, use_force, seconds]
                    
                    Kps = np.resize(Kps, (1, 36 * length_total))[0]
                    Kvs = np.resize(Kvs, (1, 36 * length_total))[0]

                    
                    #JKpJ = np.resize(JKpJ, (1, 49 * num_steps))[0]
                    #JKvJ = np.resize(JKvJ, (1, 49 * num_steps))[0]
                    stamps = asarray(seg_info['stamps'])
                    data = np.zeros((1, length_total*(7+49+49+6+36+36+7+49+49+6+36+36+1)+2))
                    data[0][0:length_total*7] = qs_r
                    data[0][length_total*7:length_total*(7+49)] = JKpJ
                    data[0][length_total*(7+49):length_total*(7+49+49)] = JKvJ
                    data[0][length_total*(7+49+49):length_total*(7+49+49+6)] = F
                    data[0][length_total*(7+49+49+6):length_total*(7+49+49+6+36)] = Kps
                    data[0][length_total*(7+49+49+6+36):length_total*(7+49+49+6+36+36)] = Kvs
                    data[0][length_total*(7+49+49+6+36+36):length_total*(7+49+49+6+36+36+7)] = qs_l
                    data[0][length_total*(7+49+49+6+36+36+7):length_total*(7+49+49+6+36+36+7+49)] = JKpJ
                    data[0][length_total*(7+49+49+6+36+36+7+49):length_total*(7+49+49+6+36+36+7+49+49)] = JKvJ
                    data[0][length_total*(7+49+49+6+36+36+7+49+49):length_total*(7+49+49+6+36+36+7+49+49+6)] = F
                    data[0][length_total*(7+49+49+6+36+36+7+49+49+6):length_total*(7+49+49+6+36+36+7+49+49+6+36)] = Kps
                    data[0][length_total*(7+49+49+6+36+36+7+49+49+6+36):length_total*(7+49+49+6+36+36+7+49+49+6+36+36)] = Kvs
                    data[0][length_total*(7+49+49+6+36+36+7+49+49+6+36+36):length_total*(7+49+49+6+36+36+7+49+49+6+36+36+1)] = gripper
                    data[0][-2] = 0
                    data[0][-1] = stamps[i_end] - stamps[i_start]
                    msg = Float64MultiArray()
                    msg.data = data[0].tolist()
                    pub = rospy.Publisher("/controller_data", Float64MultiArray)
                    redprint("Press enter to start trajectory")
                    raw_input()
                    time.sleep(1)

                    pub.publish(msg)
                    time.sleep(1)


            #if not success: break


            
            print("Segment %s result: %s"%(seg_name, success))

        print("exit loop")
        for i in range(100):
            time.sleep(1)
    except KeyboardInterrupt:
        redprint("=================================DONE================================")
        raw_input()
        raw_input()
        raw_input()
        fht.write('\nstop:' + str(rospy.get_rostime().secs))
        fht.write('\ndone:' + str(rospy.get_rostime().secs))
        fht.close()
        time.sleep(3)
        if started_bag:
            print "stopping bag"
            bag_handle.send_signal(signal.SIGINT)
            #bag_handle.wait()
            started_bag = False
        if started_video:
            print "stopping video"
            video_handle.send_signal(signal.SIGINT)
            #video_handle.wait()
            started_video = False
        bagfilename = demo_name+".bag"
        
        annfilename = demo_name+".ann.yaml"
        
        call_and_print("generate_ann.py %s %s %s"%(bagfilename, annfilename, timestampfile),check=False)
        with open(args.new_demo,"a") as fh1:
            fh1.write("\n"
                "- bag_file: %(bagfilename)s\n"
                "  annotation_file: %(annfilename)s\n"
                "  video_dir: %(videodir)s\n"
                "  demo_name: %(demoname)s"%dict(bagfilename=bagfilename, annfilename=annfilename, videodir=demo_name, demoname=demo_name))
        
        return
if __name__ == "__main__":
    main()
