
from old_rapprentice import ros2rave, func_utils, berkeley_pr2
import fastrapp
import numpy as np
import cv2
import openravepy
import os.path as osp
from old_rapprentice import robot_states
import matplotlib.pyplot as plt
import IPython


def extract_torques(bag):
    jtf_torques = []
    pid_torques = []
    for (_, msg, _) in bag.read_messages(topics=['/my_controller_name/pid_torques']):        
        pid_torques.append(np.array(msg.data))
    for (_, msg, _) in bag.read_messages(topics=['/my_controller_name/jtf_torques']):        
        jtf_torques.append(np.array(msg.data))
    assert len(jtf_torques) > 0
    assert len(pid_torques) > 0
    return pid_torques, jtf_torques

def extract_joints(bag):
    traj = []
    stamps = []
    velocities = []
    efforts = []
    for (_, msg, _) in bag.read_messages(topics=['/joint_states']):  
        traj.append(msg.position)
        velocities.append(msg.velocity)
        efforts.append(msg.effort)
        stamps.append(msg.header.stamp.to_sec())
    assert len(traj) > 0
    names = msg.name
    
    return names, stamps, traj, velocities, efforts

def select_with_names(values, names, selections):
    selected_values = []
    indices = []

    for selection in selections:
        indices.append(names.index(selection))
    assert len(values[0]) == len(names)
    for i in xrange(len(values)):
        sv = []
        for j in indices:
            sv.append(values[i][j])
        sv = np.array(sv)
        selected_values.append(sv)
    return selected_values


def compute_end_effector_forces(jacobians_ds, efforts_ds):
    effs = []
    for i, _ in enumerate(jacobians_ds):
        effs.append(robot_states.compute_end_effector_force(jacobians_ds[i], efforts_ds[i-1]))
    return effs

def extract_joy(bag):
    """sounds morbid
    """

    stamps = []
    meanings = []
    button2meaning = {
        12: "look",
        0: "start",
        3: "stop",
        7: "l_open",
        5: "l_close",
        15: "r_open",
        13: "r_close",
        14: "done"
    }
    check_buttons = button2meaning.keys()
    message_stream = bag.read_messages(topics=['/joy'])
    (_,last_msg,_) = message_stream.next()
    for (_, msg, _) in message_stream:
        for i in check_buttons:
            if msg.buttons[i] and not last_msg.buttons[i]:
                stamps.append(msg.header.stamp.to_sec())
                meanings.append(button2meaning[i])
        last_msg = msg
        
    return stamps, meanings

        
def find_disjoint_subsequences(li, seq):
    """
    Returns a list of tuples (i,j,k,...) so that seq == (li[i], li[j], li[k],...)
    Greedily find first tuple, then second, etc.
    """
    subseqs = []
    cur_subseq_inds = []
    for (i_el, el) in enumerate(li):
        if el == seq[len(cur_subseq_inds)]:
            cur_subseq_inds.append(i_el)
            if len(cur_subseq_inds) == len(seq):
                subseqs.append(cur_subseq_inds)
                cur_subseq_inds = []
    return subseqs
    
def joy_to_annotations(stamps, meanings):
    """return a list of dicts giving info for each segment
    [{"look": 1234, "start": 2345, "stop": 3456},...]
    """
    out = []
    ind_tuples = find_disjoint_subsequences(meanings, ["look","start","stop"])
    for tup in ind_tuples:
        out.append({"look":stamps[tup[0]], "start":stamps[tup[1]], "stop":stamps[tup[2]]})
    
    done_inds = [i for (i,meaning) in enumerate(meanings) if meaning=="done"]
    for ind in done_inds:
        out.append({"done":None,"look":stamps[ind], "start":stamps[ind], "stop":stamps[ind]+1})
    
    return out

def add_kinematics_to_group(group, linknames, manipnames, jointnames, robot):
    "do forward kinematics on those links"
    if robot is None: robot = get_robot()
    r2r = ros2rave.RosToRave(robot, group["joint_states"]["name"])
    link2hmats = dict([(linkname, []) for linkname in linknames])
    links = [robot.GetLink(linkname) for linkname in linknames]
    rave_traj = []
    rave_inds = r2r.rave_inds
    for ros_vals in group["joint_states"]["position"]:
        r2r.set_values(robot, ros_vals)
        rave_vals = r2r.convert(ros_vals)
        robot.SetDOFValues(rave_vals, rave_inds)
        rave_traj.append(rave_vals)
        for (linkname,link) in zip(linknames, links):
            link2hmats[linkname].append(link.GetTransform())
    for (linkname, hmats) in link2hmats.items():
        group.create_group(linkname)
        group[linkname]["hmat"] = np.array(hmats)
        
    rave_traj = np.array(rave_traj)
    rave_ind_list = list(rave_inds)
    for manipname in manipnames:
        arm_inds = robot.GetManipulator(manipname).GetArmIndices()
        group[manipname] = rave_traj[:,[rave_ind_list.index(i) for i in arm_inds]]
        
    for jointname in jointnames:
        joint_ind = robot.GetJointIndex(jointname)
        group[jointname] = rave_traj[:,rave_ind_list.index(joint_ind)]
        
    
    
    
@func_utils.once
def get_robot():
    env = openravepy.Environment()
    env.Load("robots/pr2-beta-static.zae")
    robot = env.GetRobots()[0]
    return robot
    
def add_bag_to_hdf(bag, annotations, hdfroot, demo_name):
    joint_names, stamps, traj, velocities, efforts = extract_joints(bag)
    traj = np.asarray(traj)
    stamps = np.asarray(stamps)
    velocities = np.asarray(stamps)
    efforts = np.asarray(efforts)
    accelerations = np.diff()
    #jacobians_r = extract_jacobians_r(bag)
    #jacobians_r = np.array(jacobians_r)
    #jacobians_l = extract_jacobians_l(bag)
    #jacobians_l = np.array(jacobians_l)

    robot = get_robot()

    for seg_info in annotations:


        group = hdfroot.create_group(demo_name + "_" + seg_info["name"])
    
        start = seg_info["start"]
        stop = seg_info["stop"]
        
        [i_start, i_stop] = np.searchsorted(stamps, [start, stop])
        
        sample_inds = range(i_start, i_stop-1)
        print "trajectory has length", len(sample_inds)

        traj_ds = traj[sample_inds,:]
        stamps_ds = stamps[sample_inds]
        efforts_ds = efforts[sample_inds]
        efforts_r_arm = np.array(select_with_names(efforts_ds, joint_names, robot_states.r_arm_joint_names))
        efforts_l_arm = np.array(select_with_names(efforts_ds, joint_names, robot_states.l_arm_joint_names))



        # match / interpolate end effector forces at these times
        group["description"] = seg_info["description"]
        group["stamps"] = stamps_ds
        group.create_group("joint_states")
        group["joint_states"]["name"] = joint_names
        group["joint_states"]["position"] = traj_ds
        group["efforts_r"] = efforts_r_arm_ds
        group["efforts_l"] = efforts_l_arm_ds
        link_names = ["l_gripper_tool_frame","r_gripper_tool_frame","l_gripper_r_finger_tip_link","l_gripper_l_finger_tip_frame","r_gripper_r_finger_tip_link","r_gripper_l_finger_tip_frame"]
        special_joint_names = ["l_gripper_joint", "r_gripper_joint"]
        manip_names = ["leftarm", "rightarm"]
        
        add_kinematics_to_group(group, link_names, manip_names, special_joint_names, robot)







        

def get_video_frames(video_dir, frame_stamps):
    video_stamps = np.loadtxt(osp.join(video_dir,"stamps.txt"))
    frame_inds = np.searchsorted(video_stamps, frame_stamps)
    
    rgbs = []
    depths = []
    for frame_ind in frame_inds:
        rgb = cv2.imread(osp.join(video_dir,"rgb%i.jpg"%frame_ind))
        assert rgb is not None
        rgbs.append(rgb)
        depth = cv2.imread(osp.join(video_dir,"depth%i.png"%frame_ind),2)
        assert depth is not None
        depths.append(depth)
    return rgbs, depths


def add_rgbd_to_hdf(video_dir, annotations, hdfroot, demo_name):
    
    frame_stamps = [seg_info["look"] for seg_info in annotations]
    
    rgb_imgs, depth_imgs = get_video_frames(video_dir, frame_stamps)
    
    for (i_seg, seg_info) in enumerate(annotations):        
        group = hdfroot[demo_name + "_" + seg_info["name"]]
        group["rgb"] = rgb_imgs[i_seg]
        group["depth"] = depth_imgs[i_seg]
        robot = get_robot()
        r2r = ros2rave.RosToRave(robot, group["joint_states"]["name"])
        r2r.set_values(robot, group["joint_states"]["position"][0])
        T_w_h = robot.GetLink("head_plate_frame").GetTransform()
        T_w_k = T_w_h.dot(berkeley_pr2.T_h_k)
        group["T_w_k"] = T_w_k


