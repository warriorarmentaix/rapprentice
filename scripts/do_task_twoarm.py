#!/usr/bin/env python

import argparse
from lfmd_rapprentice import util
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
parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")
parser.add_argument("--useJK", type=int, default=0)
parser.add_argument("--execution", type=int, default=0)
#parser.add_argument("--animation", type=int, default=0)
parser.add_argument("--parallel", type=int, default=1)
parser.add_argument("--pid", type=int, default=0)

parser.add_argument("--prompt", action="store_true")
parser.add_argument("--show_neighbors", action="store_true")
parser.add_argument("--select_manual", action="store_true")
parser.add_argument("--log", action="store_true")
parser.add_argument("--no_ds", action="store_true")
parser.add_argument("--trajopt_ds", type=int, default=10)
parser.add_argument("--visualize", action="store_true")
parser.add_argument("--pdgains", action="store_true")

parser.add_argument("--use_force", type=int, default=0)


#parser.add_argument("--interactive",action="store_true")





parser.add_argument("--animation", type=int, default=0, help="animates if it is non-zero. the viewer is stepped according to this number")
parser.add_argument("--interactive", action="store_true", help="step animation and optimization if specified")
parser.add_argument("--camera_matrix_file", type=str, default='../.camera_matrix.txt')
parser.add_argument("--window_prop_file", type=str, default='../.win_prop.txt')

subparsers = parser.add_subparsers(dest='subparser_name')
parser_eval = subparsers.add_parser('eval')

parser_eval.add_argument('actionfile', type=str)
parser_eval.add_argument("reg_type", type=str, choices=['segment', 'rpm', 'bij'], default='bij')
parser_eval.add_argument("--downsample_size", type=float, default=0.025)

parser_eval.add_argument("--fake_data_segment",type=str, default='knot2handstester0_seg00')
parser_eval.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
    default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")
parser_eval.add_argument("--gpu", action="store_true", default=False)

parser_eval.add_argument("--pos_coef", type=float, default=1, help="coefficient for dtw position cost")
parser_eval.add_argument("--rot_coef", type=float, default=.1, help="coefficient for dtw rotation cost")
parser_eval.add_argument("--pos_vel_coef", type=float, default=0, help="coefficient for dtw position velocity cost")
parser_eval.add_argument("--rot_vel_coef", type=float, default=0, help="coefficient for dtw rotation velocity cost")
parser_eval.add_argument("--force_coef", type=float, default=1, help="coefficient for dtw force cost")
parser_eval.add_argument("--torque_coef", type=float, default=1, help="coefficient for dtw torque cost")

parser_eval.add_argument("--downsample_traj", type=int, default=1, help="downsample demonstration trajectory by this factor")
parser_eval.add_argument("--std_dev", type=float, default=1, help="number of standard deviations plotted for the covariance")
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
from rapprentice import robot_states
import analyze_data_old as analyze_data
import IPython

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
    #choice_ind = task_execution.request_int_in_range(len(seg_names))
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

def print_matrix(m):
    for i in range(m.shape[0]):
        print [str(m[i,j]) for j in range(m.shape[1])]

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
    

    demos = analyze_data.setup_demos(args)
    trajoptpy.SetInteractive(args.interactive)
    lfd_env, sim = analyze_data.setup_lfd_environment_sim(args, demos)
    reg_factory = analyze_data.setup_registration(args, demos, sim)
    # for now, use the scene state of the first demo as the current scene state
    analyze_data.analyze_data(args, reg_factory, demos.values()[0].scene_state, plotting=args.animation, sim=sim)


    IPython.embed()

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

    if not args.eval.fake_data_segment:
        grabber = cloudprocpy.CloudGrabber()
        grabber.startRGBD()

    Globals.viewer = trajoptpy.GetViewer(Globals.env)

    #####################

    while True:
        
    
        redprint("Acquire point cloud")
        if args.eval.fake_data_segment:
            fake_seg = demofile[args.eval.fake_data_segment]
            new_xyz = np.squeeze(fake_seg["cloud_xyz"])
            hmat = openravepy.matrixFromAxisAngle(args.eval.fake_data_transform[3:6])
            hmat[:3,3] = args.eval.fake_data_transform[0:3]
            new_xyz = new_xyz.dot(hmat[:3,:3].T) + hmat[:3,3][None,:]
            r2r = ros2rave.RosToRave(Globals.robot, asarray(fake_seg["joint_states"]["name"]))
            r2r.set_values(Globals.robot, asarray(fake_seg["joint_states"]["position"][0]))

        else:

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


        """
        if args.visualize:
                
            r2r = ros2rave.RosToRave(Globals.robot, asarray(seg_info["joint_states"]["name"]))
            r2r.set_values(Globals.robot, asarray(seg_info["joint_states"]["position"][0]))
            for i in range(0, asarray(seg_info["leftarm"]).shape[0], 10):
                #handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
                #handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))
                #handles.append(Globals.env.drawlinestrip(new_ee_traj_rs[:,:3,3], 2, (0,0,1,1)))
                # Setting robot arm to joint trajectory
                for lr in 'lr':
                    manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                    traj = asarray(seg_info[manip_name])
                    vals = traj[i,:]
                    Globals.robot.SetDOFValues(vals, Globals.robot.GetManipulator(manip_name).GetArmIndices())                       
                    
                redprint(i)
                Globals.viewer.Idle()

        """

        ### Old end effector forces at r/l_gripper_tool_frame (including torques)
        old_forces = {}
        old_torques = {}
        end_effector_forces = {}
        for lr in 'lr':
            end_effector_forces[lr] = []
            manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
            efforts_name = {"l":"efforts_l", "r":"efforts_r"}[lr]
            arm = Globals.robot.GetManipulator(manip_name)
            traj = np.asarray(seg_info[manip_name])
            jacobians = []
            for i in range(traj.shape[0]):
                vals = traj[i,:]
                Globals.robot.SetDOFValues(vals, Globals.robot.GetManipulator(manip_name).GetArmIndices())
                jacobians.append(np.vstack((arm.CalculateJacobian(), arm.CalculateAngularVelocityJacobian())))
            jacobians = np.asarray(jacobians)
            efforts = seg_info[efforts_name]
            for i in range(efforts.shape[0]):
                forces = robot_states.compute_end_effector_force(jacobians[i], efforts[i])
                forces = np.asarray(forces.T)[0]
                end_effector_forces[lr].append(forces)
            end_effector_forces[lr] = np.asarray(end_effector_forces[lr])
            old_forces[lr] = end_effector_forces[lr][:,0:3]
            old_torques[lr] = end_effector_forces[lr][:,3:6]

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
        #old_eefs = {}
        new_eefs = {}
        for lr in 'lr':
            link_name = "%s_gripper_tool_frame"%lr
            old_ee_traj = asarray(seg_info[link_name]["hmat"])
            new_ee_traj = f.transform_hmats(old_ee_traj)
            link2eetraj[link_name] = new_ee_traj
            #print old_ee_traj
            old_ee_pos = old_ee_traj[:,0:3,3]
            #print old_ee_pos

            # End effector forces as oppossed to end effector trajectories
            dfdxs = f.compute_jacobian(old_ee_pos)
            new_eefs[lr] = []
            for i in xrange(len(old_forces[lr])):
                dfdx = np.matrix(dfdxs[i])
                old_force = np.matrix(old_forces[lr][i]).T
                old_torque = np.matrix(old_torques[lr][i]).T
                
                new_force = np.array((dfdx * old_force).T)[0]
                new_torque = np.array((dfdx * old_torque).T)[0]
                #old_eefs[lr].append(np.hstack((old_force, old_torque)))
                new_eefs[lr].append(np.hstack((new_force, new_torque)))
            new_eefs[lr] = np.array(new_eefs[lr])
            
            
        miniseg_starts, miniseg_ends = split_trajectory_by_gripper(seg_info)    
        success = True
        
        
        print colorize.colorize("mini segments:", "red"), miniseg_starts, miniseg_ends
        for (i_miniseg, (i_start, i_end)) in enumerate(zip(miniseg_starts, miniseg_ends)):
            
            if args.execution: Globals.pr2.update_rave()


            ################################    
            redprint("Generating joint trajectory for segment %s, part %i"%(seg_name, i_miniseg))
            
            
            
            # figure out how we're gonna resample stuff
            lr2oldtraj = {}
            for lr in 'lr':
                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                old_joint_traj = asarray(seg_info[manip_name][i_start:i_end+1])
                lr2oldtraj[lr] = old_joint_traj

            
            if len(lr2oldtraj) > 0:
                old_total_traj = np.concatenate(lr2oldtraj.values(), 1)
                JOINT_LENGTH_PER_STEP = .1
                # FIRST RESAMPLING HAPPENS HERE: JOINT_LENGTH
                _, timesteps_rs = unif_resample(old_total_traj, JOINT_LENGTH_PER_STEP) # Timesteps only, can use to inter eefs for first time
            ####
            new_eefs_segment = {}
            for lr in 'lr':
                new_eefs_segment[lr] = asarray(new_eefs[lr][i_start:i_end+1,:]) # Extract miniseg, and re-sample

            if args.no_ds:
                new_eefs_segment_rs = new_eefs_segment
            else:
                for lr in 'lr':
                    new_eefs_segment_rs = {}
                    new_eefs_segment_rs[lr] = mu.interp2d(timesteps_rs, np.arange(len(new_eefs_segment[lr])), new_eefs_segment[lr])
        


            """
            if args.visualize:
                r2r = ros2rave.RosToRave(Globals.robot, asarray(seg_info["joint_states"]["name"]))
                r2r.set_values(Globals.robot, asarray(seg_info["joint_states"]["position"][0]))
                for i in range(0, asarray(lr2oldtraj["l"]).shape[0], 10):
                    #handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
                    #handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))
                    #handles.append(Globals.env.drawlinestrip(new_ee_traj_rs[:,:3,3], 2, (0,0,1,1)))
                    # Setting robot arm to joint trajectory
                    for lr in 'lr':
                        part_name = {"l":"larm", "r":"rarm"}[lr]
                        manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                        traj = lr2oldtraj[lr]
                        vals = traj[i,:]
                        Globals.robot.SetDOFValues(vals, Globals.robot.GetManipulator(manip_name).GetArmIndices())                       
                        
                    redprint(i)
                    Globals.viewer.Idle()
            """

            if args.execution: Globals.pr2.update_rave()
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
                
                part_name = {"l":"larm", "r":"rarm"}[lr]
                bodypart2traj[part_name] = new_joint_traj
                redprint("Joint trajectory has length: " + str(len(bodypart2traj[part_name])))
            

            """
            if args.visualize:
                
                r2r = ros2rave.RosToRave(Globals.robot, asarray(seg_info["joint_states"]["name"]))
                r2r.set_values(Globals.robot, asarray(seg_info["joint_states"]["position"][0]))
                for lr in 'lr':
                    link_name = "%s_gripper_tool_frame"%lr
                    old_ee_traj = asarray(seg_info[link_name]["hmat"])
                    new_ee_traj = link2eetraj[link_name]
                    #handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
                    handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))
                for i in range(0, asarray(bodypart2traj["larm"]).shape[0], 10):
                    #handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))
                    #handles.append(Globals.env.drawlinestrip(new_ee_traj_rs[:,:3,3], 2, (0,0,1,1)))
                    # Setting robot arm to joint trajectory
                    for lr in 'lr':
                        part_name = {"l":"larm", "r":"rarm"}[lr]
                        manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                        traj = bodypart2traj[part_name]
                        vals = traj[i,:]
                        Globals.robot.SetDOFValues(vals, Globals.robot.GetManipulator(manip_name).GetArmIndices())                       
                        
                    redprint(i)
                    Globals.viewer.Idle()
            """

            redprint("Executing joint trajectory for segment %s, part %i using arms '%s'"%(seg_name, i_miniseg, bodypart2traj.keys()))

            redprint("Press enter to set gripper")
            raw_input()
            for lr in 'lr':
                set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))

            if args.pid:

                redprint("Press enter to use current position as starting point")
                raw_input()
                arm_positions = {}
                part_name = {"l":"larm", "r":"rarm"}[lr]
                (arm_position, arm_velocity, arm_effort) = robot_states.call_return_joint_states(robot_states.r_arm_joint_names)
                arm_positions['r'] = arm_position
                diff_r = np.array(arm_position - bodypart2traj['rarm'][0,:])
                maximum_r = max(abs(diff_r))
                (arm_position, arm_velocity, arm_effort) = robot_states.call_return_joint_states(robot_states.l_arm_joint_names)
                arm_positions['l'] = arm_position
                diff_l = np.array(arm_position - bodypart2traj['larm'][0,:])
                maximum_l = max(abs(diff_l))
                maximum = max(maximum_l, maximum_r)

                speed = (30.0/360.0*2*(np.pi))
                time_needed = maximum / speed
                print time_needed
                complete_force_traj = {}
                for lr in 'lr':
                    part_name = {"l":"larm", "r":"rarm"}[lr]
                    initial_pos_traj = mu.interp2d(np.arange(0, time_needed, 0.01), np.array([0,time_needed]), np.array([arm_positions[lr], bodypart2traj[part_name][0,:]]))
                    initial_force_traj = np.array([np.zeros(6) for i in range(initial_pos_traj.shape[0])])
                    temptraj = bodypart2traj[part_name]
                    bodypart2traj[part_name] = np.concatenate((initial_pos_traj, temptraj), axis=0)
                    complete_force_traj[lr] = np.concatenate((initial_force_traj, new_eefs[lr][i_start:i_end+1]), axis=0)

                initial_traj_length = initial_pos_traj.shape[0]
                
                # SECOND RESAMPLING HAPPENS HERE: JOINT VELOCITIES
                traj = {}
                F = {}
                if args.no_ds:
                    for lr in 'lr':
                        part_name = {"l":"larm", "r":"rarm"}[lr]
                        traj[lr] = bodypart2traj[part_name]
                        F[lr] = complete_force_traj[lr]
                    stamps = asarray(seg_info['stamps'])
                    times = np.array([stamps[i_end] - stamps[i_start]])
                else:
                    times, times_up, traj = pr2_trajectories.return_timed_traj(Globals.pr2, bodypart2traj) # use times and times_up to interpolate the second time
                    F = mu.interp2d(times_up, times, complete_force_traj)
                
                #Globals.robot.SetDOFValues(asarray(fake_seg["joint_states"]["position"][0]))
                if args.visualize:
                    r2r = ros2rave.RosToRave(Globals.robot, asarray(seg_info["joint_states"]["name"]))
                    r2r.set_values(Globals.robot, asarray(seg_info["joint_states"]["position"][i_start]))
                    for i in range(0, traj['l'].shape[0], 10):
                        handles = []
                        handles.append(Globals.env.plot3(new_xyz,5,np.array([(0,1,0,1) for x in new_xyz])))
                        handles.append(Globals.env.plot3(old_xyz,5,np.array([(1,0,0,1) for x in old_xyz])))
                        #handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
                        #handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))
                        #handles.append(Globals.env.drawlinestrip(new_ee_traj_rs[:,:3,3], 2, (0,0,1,1)))
                        # Setting robot arm to joint trajectory
                        for lr in 'lr':
                            manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                            vals = traj[lr][i,:]
                            Globals.robot.SetDOFValues(vals, Globals.robot.GetManipulator(manip_name).GetArmIndices())
                        
                            hmats_index = {"l":-28, "r":-3}
                            hmats = Globals.robot.GetLinkTransformations()
                            trans, rot = conv.hmat_to_trans_rot(hmats[hmats_index[lr]])
                            f_start = np.array([0,0,0]) + trans
                            f_end = F[lr][i][0:3]/100 + trans
                            handles.append(Globals.env.drawlinestrip(np.array([f_start, f_end]), 10, (1,0,0,1)))
                        
                        redprint(i)
                        Globals.viewer.Step()
                
                
                pgains = np.asarray([2400.0, 1200.0, 1000.0, 700.0, 300.0, 300.0, 300.0])
                dgains = np.asarray([18.0, 10.0, 6.0, 4.0, 6.0, 4.0, 4.0])
                pgainsdiag = np.diag(np.asarray([-2400.0, -1200.0, -1000.0, -700.0, -300.0, -300.0, -300.0]))
                dgainsdiag = np.diag(np.asarray([-18.0, -10.0, -6.0, -4.0, -6.0, -4.0, -4.0]))
                m = np.array([3.33, 1.16, 0.1, 0.25, 0.133, 0.0727, 0.0727]) # masses in joint space (feed forward)
                #IPython.embed()
                #covariances, means = analyze_data.run_analyze_data(args)
                #IPython.embed()
                """
                vel_factor = 1e-3
                #new costs
                covariances, means = analyze_data.run_analyze_data(args)
                CostsNew = []
                counterCovs = 0
                endTraj = False
                covThiss = []
                for covMat in covariances:
                    covThis = np.zeros((18,18))
                    covThis[0:6,0:6] = covMat[0:6,0:6]
                    covThis[12:18,12:18] = covMat[6:12, 6:12]
                    covThis[0:6, 12:18] = covMat[0:6, 6:12]
                    covThis[12:18, 0:6] = covMat[6:12, 0:6]
                    covThis[6:12, 6:12] = np.eye(6)*vel_factor
                    covThis = np.diag(np.diag(covThis))
                    covThiss.append(covThis)
                    #if len(covThiss) <= 69 and len(covThiss) >= 47:
                    #    covThis[12:18,12:18] = np.diag([0.13, 0.06, 0.07, 0.005, 0.01, 0.004])
                    for j in range(args.eval.downsample_traj):
                        invCov = np.linalg.inv(covThis)
                        CostsNew.append(invCov)
                        counterCovs = counterCovs + 1
                        if counterCovs >= traj_length:
                            endTraj = True
                            break
                    if endTraj:
                        break
                
                
                x = np.ones(6) * 1 
                v = np.ones(6) * 1e-3
                a = np.ones(6) * 1e-6
                Ct = np.diag(np.hstack((x, v, a))) # in end effector space
                """

                num_steps = i_end - i_start + 1
                total_length = num_steps + initial_traj_length

                CostsNew = []
                allCs = []
                cov = np.zeros((18,18))
                cov[0:6,0:6] = np.diag([1, 1, 1, 1, 1, 1])
                cov[6:12,6:12] = np.diag([1, 1, 1, 1, 1, 1])
                cov[12:18,12:18] = np.diag([1, 1, 1, 1, 1, 1])
                for i in range(num_steps):
                    CostsNew.append(np.linalg.inv(cov))

                JKpJ = {}
                JKvJ = {}

                for lr in 'lr':
                    Ms = []
                    stamps = asarray(seg_info['stamps'][i_start:i_end+1])

                    manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                    arm = Globals.robot.GetManipulator("rightarm")
                    jacobians = []
                    for i in range(traj[lr].shape[0]):
                        r_vals = traj[lr][i,:]
                        Globals.robot.SetDOFValues(r_vals, Globals.robot.GetManipulator(manip_name).GetArmIndices())
                        jacobians.append(np.vstack((arm.CalculateJacobian(), arm.CalculateAngularVelocityJacobian())))
                    jacobians =  asarray(jacobians)

                    for t in range(num_steps):
                        M = np.zeros((18, 21))
                        J = jacobians[t]
                        M[0:6,0:7] = J
                        M[6:12,7:14] = J
                        mdiag = np.diag(m) # Convert joint feedforward values into diagonal array
                        mdiag_inv = np.linalg.inv(mdiag)
                        M[12:18,14:21] = np.linalg.inv((J.dot(mdiag_inv).dot(np.transpose(J)))).dot(J).dot(mdiag_inv)
                        Ms.append(M)
                    for t in range(num_steps):
                        topad = np.zeros((21,21))
                        topad[0:7,0:7] = np.diag(pgains) * 0.01
                        topad[7:14,7:14] = np.diag(dgains) * 0.01
                        topad[14:21,14:21] = np.eye(7) * 0.01
                        #allCs.append(np.transpose(Ms[t]).dot(Ct).dot(Ms[t]) + topad)
                        allCs.append(np.transpose(Ms[t]).dot(CostsNew[t]).dot(Ms[t]) + topad)


                    Kps = []
                    Kvs = []
                    Ks = []
                    Qt = None
                    Vs = []
                    for t in range(num_steps-1, -1, -1):
                        if Qt is None:
                            Qt = allCs[t]
                        else:
                            Ft = np.zeros((14, 21))
                            for j in range(14):
                                Ft[j][j] = 1.0
                            deltat = abs(stamps[t+1] - stamps[t])
                            #print(deltat)
                            for j in range(7):
                                Ft[j][j+7] = deltat
                            for j in range(7):
                                Ft[j+7][j+14] = deltat/m[j]  
                            for j in range(7):
                                Ft[j][j+14] = ((deltat)**2)/m[j]
                            Qt = allCs[t] + (np.transpose(Ft).dot(Vs[num_steps-2-t]).dot(Ft))
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

                    Ks = Ks[::-1]
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
                Fx = []
                for i in range(num_steps + initial_traj_length):
                    Kps.append(np.zeros((6,6)))
                    Kvs.append(np.zeros((6,6)))
                    Fx.append(np.zeros((6, 1)))

                Kps = np.asarray(Kps)
                Kvs = np.asarray(Kvs)
                Fx = np.asarray(Fx)

                # Gains as JKpJ and JKvJ for testing

                if args.pdgains:
                    for lr in 'lr':
                        JKpJ[lr] = np.asarray([pgainsdiag for i in range(total_length)])
                        JKvJ[lr] = np.asarray([dgainsdiag for i in range(total_length)])
                

                #IPython.embed()


                Kps = np.resize(Kps, (1, 36 * total_length))[0]
                Kvs = np.resize(Kvs, (1, 36 * total_length))[0]
                Fx = np.resize(Fx, (1, 6 * total_length))[0]
                

                
                
                for lr in 'lr':
                    JKvJ[lr] = np.resize(JKvJ[lr], (1, 49*total_length))[0]
                    JKpJ[lr] = np.resize(JKpJ[lr], (1, 49*total_length))[0]
                    traj[lr] = np.resize(traj[lr], (1, traj[lr].shape[0]*7))[0] #resize the data to 1x7n (n being the number of steps)
                    F[lr] = np.resize(F[lr], (1, F[lr].shape[0]*6))[0] #resize the data to 1x6n


                # [traj, Kp, Kv, F, use_force, seconds]
                data = np.zeros((1, total_length*(7+49+49+6+36+36+7+49+49+6+36+36)+2))
                data[0][0:total_length*7] = traj['r']
                data[0][total_length*7:total_length*(7+49)] = JKpJ['r']
                data[0][total_length*(7+49):total_length*(7+49+49)] = JKvJ['r']
                data[0][total_length*(7+49+49):total_length*(7+49+49+6)] = F['r']
                data[0][total_length*(7+49+49+6):total_length*(7+49+49+6+36)] = Kps
                data[0][total_length*(7+49+49+6+36):total_length*(7+49+49+6+36+36)] = Kvs
                data[0][total_length*(7+49+49+6+36+36):total_length*(7+49+49+6+36+36+7)] = traj['l']
                data[0][total_length*(7+49+49+6+36+36+7):total_length*(7+49+49+6+36+36+7+49)] = JKpJ['l']
                data[0][total_length*(7+49+49+6+36+36+7+49):total_length*(7+49+49+6+36+36+7+49+49)] = JKvJ['l']
                data[0][total_length*(7+49+49+6+36+36+7+49+49):total_length*(7+49+49+6+36+36+7+49+49+6)] = F['l']
                data[0][total_length*(7+49+49+6+36+36+7+49+49+6):total_length*(7+49+49+6+36+36+7+49+49+6+36)] = Kps
                data[0][total_length*(7+49+49+6+36+36+7+49+49+6+36):total_length*(7+49+49+6+36+36+7+49+49+6+36+36)] = Kvs
                data[0][-2] = args.use_force
                data[0][-1] = 0
                msg = Float64MultiArray()
                msg.data = data[0].tolist()
                pub = rospy.Publisher("/controller_data", Float64MultiArray)
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
