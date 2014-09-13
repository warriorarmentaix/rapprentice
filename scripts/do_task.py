#!/usr/bin/env python

import argparse
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
parser.add_argument("--cloud_proc_func", default="filter_green")
parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")
    
parser.add_argument("--execution", type=int, default=0)
parser.add_argument("--animation", type=int, default=0)
parser.add_argument("--parallel", type=int, default=1)
parser.add_argument("--pid", type=int, default=0)

parser.add_argument("--prompt", action="store_true")
parser.add_argument("--show_neighbors", action="store_true")
parser.add_argument("--select_manual", action="store_true")
parser.add_argument("--log", action="store_true")
parser.add_argument("--no_ds", action="store_true")
parser.add_argument("--trajopt_ds", type=int, default=10)
parser.add_argument("--visualize", action="store_true")

parser.add_argument("--use_force", type=int, default=0)

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
     planning, tps, func_utils, resampling, clouds, conversions as conv, retiming
from rapprentice import math_utils as mu
from rapprentice.yes_or_no import yes_or_no

try:
    from rapprentice import pr2_trajectories, PR2, robot_states
    import rospy
except ImportError:
    print "Couldn't import ros stuff"

import cloudprocpy, trajoptpy, openravepy
import os, numpy as np, h5py, time
from numpy import asarray
import numpy as np
import cPickle as pickle
import importlib
from std_msgs.msg import Float64MultiArray, String

cloud_proc_mod = importlib.import_module(args.cloud_proc_mod)
cloud_proc_func = getattr(cloud_proc_mod, args.cloud_proc_func)
        
    
def redprint(msg):
    print colorize.colorize(msg, "red", bold=True)
    
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
    if args.animation:
        dof_inds = []
        trajs = []
        for (part_name, traj) in bodypart2traj.items():
            manip_name = {"larm":"leftarm","rarm":"rightarm"}[part_name]
            dof_inds.extend(Globals.robot.GetManipulator(manip_name).GetArmIndices())            
            trajs.append(traj)
        full_traj = np.concatenate(trajs, axis=1)
        Globals.robot.SetActiveDOFs(dof_inds)
        animate_traj.animate_traj(full_traj, Globals.robot, restore=False,pause=True)
    if args.execution:
        if not args.prompt or yes_or_no("execute?"):
            pr2_trajectories.follow_body_traj(Globals.pr2, bodypart2traj)
        else:
            return False

    return True


def find_closest_manual(demofile, _new_xyz):
    "for now, just prompt the user"
    seg_names = demofile.keys()
    print "choose from the following options (type an integer)"
    for (i, seg_name) in enumerate(seg_names):
        print "%i: %s"%(i,seg_name)
    choice_ind = task_execution.request_int_in_range(len(seg_names))
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
    torso_val = None

    pr2 = None
    pub = None

def main():
    import IPython
    demofile = h5py.File(args.h5file, 'r')
    trajoptpy.SetInteractive(args.interactive)


    if args.log:
        LOG_DIR = osp.join(osp.expanduser("~/Data/do_task_logs"), datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
        os.mkdir(LOG_DIR)
        LOG_COUNT = 0
                        

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

    while True:
        
    
        redprint("Acquire point cloud")
        if args.fake_data_segment:
            fake_seg = demofile[args.fake_data_segment]
            new_xyz = np.squeeze(fake_seg["cloud_xyz"])
            hmat = openravepy.matrixFromAxisAngle(args.fake_data_transform[3:6])
            hmat[:3,3] = args.fake_data_transform[0:3]
            new_xyz = new_xyz.dot(hmat[:3,:3].T) + hmat[:3,3][None,:]
            r2r = ros2rave.RosToRave(Globals.robot, asarray(fake_seg["joint_states"]["name"]))
            r2r.set_values(Globals.robot, asarray(fake_seg["joint_states"]["position"][0]))
            #Globals.pr2.head.set_pan_tilt(0,1.2)
            #Globals.pr2.rarm.goto_posture('side')
            #Globals.pr2.larm.goto_posture('side')            
            #Globals.pr2.join_all()
            #time.sleep(2)
        else:
            #Globals.pr2.head.set_pan_tilt(0,1.2)
            #Globals.pr2.rarm.goto_posture('side')
            #Globals.pr2.larm.goto_posture('side')            
            #Globals.pr2.join_all()
            #time.sleep(2)

            
            Globals.pr2.update_rave()
            
            rgb, depth = grabber.getRGBD()
            T_w_k = berkeley_pr2.get_kinect_transform(Globals.robot)
            new_xyz = cloud_proc_func(rgb, depth, T_w_k)
            print "got new xyz"
            redprint(new_xyz)
            #grab_end(new_xyz)

    
        if args.log:
            LOG_COUNT += 1
            import cv2
            cv2.imwrite(osp.join(LOG_DIR,"rgb%i.png"%LOG_COUNT), rgb)
            cv2.imwrite(osp.join(LOG_DIR,"depth%i.png"%LOG_COUNT), depth)
            np.save(osp.join(LOG_DIR,"xyz%i.npy"%LOG_COUNT), new_xyz)
        

        


        ################################    
        redprint("Finding closest demonstration")
        if args.select_manual:
            seg_name = find_closest_manual(demofile, new_xyz)
        else:
            seg_name = find_closest_auto(demofile, new_xyz)
        
        seg_info = demofile[seg_name]
        redprint("closest demo: %s"%(seg_name))
        if "done" in seg_name:
            redprint("DONE!")
            break
    
    
        if args.log:
            with open(osp.join(LOG_DIR,"neighbor%i.txt"%LOG_COUNT),"w") as fh: fh.write(seg_name) 
        ################################



        ### Old end effector forces at r_gripper_tool_frame (eliminating the torques for now)
        old_eefs = seg_info['end_effector_forces'][:,0:3,:]


        redprint("Generating end-effector trajectory")    

        handles = []
        old_xyz = np.squeeze(seg_info["cloud_xyz"])
       


        scaled_old_xyz, src_params = registration.unit_boxify(old_xyz)
        scaled_new_xyz, targ_params = registration.unit_boxify(new_xyz)        
        f,_ = registration.tps_rpm_bij(scaled_old_xyz, scaled_new_xyz, plot_cb = tpsrpm_plot_cb,
                                       plotting=5 if args.animation else 0,rot_reg=np.r_[1e-4,1e-4,1e-1], n_iter=50, reg_init=10, reg_final=.1)
        f = registration.unscale_tps(f, src_params, targ_params)
        
        old_xyz_transformed = f.transform_points(old_xyz)

        #handles.append(Globals.env.plot3(old_xyz_transformed,5, np.array([(0,0,1,1) for i in old_xyz_transformed])))

        handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, old_xyz.min(axis=0)-np.r_[0,0,.1], old_xyz.max(axis=0)+np.r_[0,0,.1], xres = .1, yres = .1, zres = .04))        

        link2eetraj = {}
        for lr in 'r':
            link_name = "%s_gripper_tool_frame"%lr
            old_ee_traj = asarray(seg_info[link_name]["hmat"])
            new_ee_traj = f.transform_hmats(old_ee_traj)
            link2eetraj[link_name] = new_ee_traj
            #print old_ee_traj
            old_ee_pos = old_ee_traj[:,0:3,3]
            #print old_ee_pos

            # End effector forces as oppossed to end effector trajectories
            dfdxs = f.compute_jacobian(old_ee_pos)
            new_eefs = []
            for i in xrange(len(old_eefs)):
                dfdx = np.matrix(dfdxs[i])
                old_eef = np.matrix(old_eefs[i])
                new_eefs.append(dfdx * old_eef)
            old_eefs = asarray(old_eefs)[:,:,0]
            new_eefs = asarray(new_eefs)[:,:,0]

            force_data = {}
            force_data['old_eefs'] = old_eefs
            force_data['new_eefs'] = new_eefs
            force_file = open("trial.pickle", 'wa')
            pickle.dump(force_data, force_file)
            force_file.close()
            new_ee_traj_x = new_ee_traj
            
        
        miniseg_starts, miniseg_ends = split_trajectory_by_gripper(seg_info)    
        success = True
        
        
        print colorize.colorize("mini segments:", "red"), miniseg_starts, miniseg_ends
        for (i_miniseg, (i_start, i_end)) in enumerate(zip(miniseg_starts, miniseg_ends)):
            
            if args.execution=="real": Globals.pr2.update_rave()


            ################################    
            redprint("Generating joint trajectory for segment %s, part %i"%(seg_name, i_miniseg))
            
            
            
            # figure out how we're gonna resample stuff
            lr2oldtraj = {}
            for lr in 'r':
                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                old_joint_traj = asarray(seg_info[manip_name][i_start:i_end+1])
                #print (old_joint_traj[1:] - old_joint_traj[:-1]).ptp(axis=0), i_start, i_end
                #if arm_moved(old_joint_traj): NOT SURE WHY BUT THIS IS RETURNING FALSE
                lr2oldtraj[lr] = old_joint_traj

            if args.visualize:
                    r2r = ros2rave.RosToRave(Globals.robot, asarray(seg_info["joint_states"]["name"]))
                    r2r.set_values(Globals.robot, asarray(seg_info["joint_states"]["position"][0]))
                    for i in range(0, lr2oldtraj['r'].shape[0], 10):
                        handles = []
                        handles.append(Globals.env.plot3(new_xyz,5,np.array([(0,1,0,1) for x in new_xyz])))
                        handles.append(Globals.env.plot3(old_xyz,5,np.array([(1,0,0,1) for x in old_xyz])))
                        handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
                        # Setting robot arm to joint trajectory
                        r_vals = lr2oldtraj['r'][i,:]
                        Globals.robot.SetDOFValues(r_vals, Globals.robot.GetManipulator('rightarm').GetArmIndices())
                        # Plotting forces from r_gripper_tool_frame
                        hmats = Globals.robot.GetLinkTransformations()
                        trans, rot = conv.hmat_to_trans_rot(hmats[-3])
                        f_start = np.array([0,0,0]) + trans
                        f_end = np.array(old_eefs[i])/100 + trans
                        handles.append(Globals.env.drawlinestrip(np.array([f_start, f_end]), 10, (1,0,0,1)))
                        
                        redprint(i)
                        Globals.viewer.Step()
            if len(lr2oldtraj) > 0:
                old_total_traj = np.concatenate(lr2oldtraj.values(), 1)
                JOINT_LENGTH_PER_STEP = .1
                # FIRST RESAMPLING HAPPENS HERE: JOINT_LENGTH
                _, timesteps_rs = unif_resample(old_total_traj, JOINT_LENGTH_PER_STEP) # Timesteps only, can use to inter eefs for first time
            ####
            new_eefs_segment = asarray(new_eefs[i_start:i_end+1,:]) # Extract miniseg, and re-sample

            if args.no_ds:
                new_eefs_segment_rs = new_eefs_segment
            else:
                new_eefs_segment_rs = mu.interp2d(timesteps_rs, np.arange(len(new_eefs_segment)), new_eefs_segment)
        
            ### Generate fullbody traj
            bodypart2traj = {}
            for (lr,old_joint_traj) in lr2oldtraj.items():

                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                ee_link_name = "%s_gripper_tool_frame"%lr
                new_ee_traj = link2eetraj[ee_link_name][i_start:i_end+1]
                if args.no_ds:
                    old_joint_traj_rs = old_joint_traj
                    new_ee_traj_rs = new_ee_traj
                    ds_inds = np.arange(0, len(new_ee_traj_rs), args.trajopt_ds)
                    new_ee_traj_rs = new_ee_traj_rs[ds_inds]
                    
                    old_joint_traj_rs = old_joint_traj_rs[ds_inds]
                    new_joint_traj = planning.plan_follow_traj(Globals.robot, manip_name,
                        Globals.robot.GetLink(ee_link_name), new_ee_traj_rs,old_joint_traj_rs)
                    new_joint_traj = mu.interp2d(np.arange(len(old_joint_traj)), np.arange(0, len(new_joint_traj) * args.trajopt_ds, args.trajopt_ds), new_joint_traj)
                else:
                    old_joint_traj_rs = mu.interp2d(timesteps_rs, np.arange(len(old_joint_traj)), old_joint_traj)
                    new_ee_traj_rs = resampling.interp_hmats(timesteps_rs, np.arange(len(new_ee_traj)), new_ee_traj)
                    new_joint_traj = planning.plan_follow_traj(Globals.robot, manip_name,
                        Globals.robot.GetLink(ee_link_name), new_ee_traj_rs,old_joint_traj_rs)
                if args.execution: Globals.pr2.update_rave()
                part_name = {"l":"larm", "r":"rarm"}[lr]
                bodypart2traj[part_name] = new_joint_traj
                redprint("Joint trajectory has length: " + str(len(bodypart2traj[part_name])))
                
            redprint("Executing joint trajectory for segment %s, part %i using arms '%s'"%(seg_name, i_miniseg, bodypart2traj.keys()))


            redprint("Press enter to set gripper")
            raw_input()

            for lr in 'r':
                set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))

            if args.pid:

                if not args.fake_data_segment: # If running on PR2, add initial state as waypoint and rough interpolate
                    # Add initial position
                    (arm_position, arm_velocity, arm_effort) = robot_states.call_return_joint_states(robot_states.arm_joint_names)
                    traj_length = bodypart2traj['rarm'].shape[0]
                    complete_joint_traj = np.zeros((traj_length+1, 7)) # To include initial state as a way point
                    complete_joint_traj[1:,:] = bodypart2traj['rarm']
                    complete_joint_traj[0,:] = arm_position
                    bodypart2traj['rarm'] = complete_joint_traj

                    # Add initial eff
                    J = np.matrix(np.resize(np.array(robot_states.call_return_jacobian()), (6, 7))) # Jacobian
                    eff = robot_states.compute_end_effector_force(J, arm_effort).T
                    eff =  np.array(eff)[0]
                    init_force = eff[:3]
                    complete_force_traj = np.zeros((traj_length+1, 3))
                    complete_force_traj[1:,:] = new_eefs_segment_rs
                    complete_force_traj[0,:] = init_force

                else:
                    complete_force_traj = new_eefs_segment_rs
                # SECOND RESAMPLING HAPPENS HERE: JOINT VELOCITIES
                if args.no_ds:
                    traj = bodypart2traj['rarm']
                    stamps = asarray(seg_info['stamps'])
                    times = np.array([stamps[i_end] - stamps[i_start]])
                    force = complete_force_traj
                else:
                    times, times_up, traj = pr2_trajectories.return_timed_traj(Globals.pr2, bodypart2traj) # use times and times_up to interpolate the second time
                    force = mu.interp2d(times_up, times, complete_force_traj)
                
                #Globals.robot.SetDOFValues(asarray(fake_seg["joint_states"]["position"][0]))
                if args.visualize:
                    r2r = ros2rave.RosToRave(Globals.robot, asarray(seg_info["joint_states"]["name"]))
                    r2r.set_values(Globals.robot, asarray(seg_info["joint_states"]["position"][0]))
                    for i in range(0, traj.shape[0], 10):
                        handles = []
                        handles.append(Globals.env.plot3(new_xyz,5,np.array([(0,1,0,1) for x in new_xyz])))
                        handles.append(Globals.env.plot3(old_xyz,5,np.array([(1,0,0,1) for x in old_xyz])))
                        handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
                        handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))
                        handles.append(Globals.env.drawlinestrip(new_ee_traj_rs[:,:3,3], 2, (0,0,1,1)))
                        # Setting robot arm to joint trajectory
                        r_vals = traj[i,:]
                        Globals.robot.SetDOFValues(r_vals, Globals.robot.GetManipulator('rightarm').GetArmIndices())
                        
                        # Plotting forces from r_gripper_tool_frame
                        hmats = Globals.robot.GetLinkTransformations()
                        trans, rot = conv.hmat_to_trans_rot(hmats[-3])
                        f_start = np.array([0,0,0]) + trans
                        f_end = np.array(force[i])/100 + trans
                        handles.append(Globals.env.drawlinestrip(np.array([f_start, f_end]), 10, (1,0,0,1)))
                        
                        redprint(i)
                        Globals.viewer.Step()


                traj = np.array(np.matrix(traj).T) # 7 x n
                redprint(traj)

                traj = np.resize(traj, (1, traj.shape[1]*7)) #resize the data to 1x7n (n being the number of steps)
                force = np.array(np.matrix(force).T) # 3 x n
                force = np.resize(force, (1, force.shape[1]*3)) #resize the data to 1x3n
                #[traj, force, secs]
                traj_force_secs = np.zeros((1, traj.shape[1] + force.shape[1] + 2))
                traj_force_secs[0,0:traj.shape[1]] = traj
                traj_force_secs[0,traj.shape[1]:traj.shape[1]+force.shape[1]] = force
                traj_force_secs[0,traj.shape[1]+force.shape[1]] = times[len(times)-1]

                traj_force_secs[0,traj.shape[1]+force.shape[1]+1] = args.use_force

                msg = Float64MultiArray()
                msg.data = traj_force_secs[0].tolist()
                pub = rospy.Publisher("/joint_positions_forces_secs", Float64MultiArray)
                redprint("Press enter to start trajectory")
                raw_input()
                time.sleep(1)
                pub.publish(msg)
                time.sleep(1)
            else:
                #if not success: break
                
                if len(bodypart2traj) > 0:
                    exec_traj_maybesim(bodypart2traj)
            
            #if not success: break

        #redprint("Segment %s result: %s"%(seg_name, success))
        

if __name__ == "__main__":

    main()
