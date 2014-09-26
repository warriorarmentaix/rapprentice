#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("demo_prefix")
parser.add_argument("master_file")
parser.add_argument("--downsample", default=3, type=int)
args = parser.parse_args()

import subprocess, signal
from old_rapprentice.colorize import colorize
import time, os, shutil
from old_rapprentice.call_and_print import call_and_print
from old_rapprentice.yes_or_no import yes_or_no
import os.path as osp
import itertools
import yaml


if "localhost" in os.getenv("ROS_MASTER_URI"):
    raise Exception("on localhost!")


started_bag = False
started_video = False

localtime   = time.localtime()
time_string  = time.strftime("%Y-%m-%d-%H-%M-%S", localtime)

os.chdir(osp.dirname(args.master_file))


if not osp.exists(args.master_file):
    yn = yes_or_no("master file does not exist. create?")
    basename = raw_input("what is the base name?\n").strip()
    if yn:
        with open(args.master_file,"w") as fh: fh.write("""
name: %s
h5path: %s
bags:
        """%(basename, basename+".h5"))
    else:
        print "exiting."
        exit(0)
with open(args.master_file, "r") as fh: master_info = yaml.load(fh)
if master_info["bags"] is None: master_info["bags"] = []
for suffix in itertools.chain("", (str(i) for i in itertools.count())):
    demo_name = args.demo_prefix + suffix
    if not any(bag["demo_name"] == demo_name for bag in master_info["bags"]):
        break
    print demo_name

try:

    bag_cmd = "rosbag record /pr2_jacobian_r /pr2_jacobian_l /joint_states /joy -O %s"%demo_name
    print colorize(bag_cmd, "green")
    bag_handle = subprocess.Popen(bag_cmd, shell=True)
    time.sleep(1)
    poll_result = bag_handle.poll() 
    print "poll result", poll_result
    if poll_result is not None:
        raise Exception("problem starting video recording")
    else: started_bag = True
    
    video_cmd = "record_rgbd_video --out=%s --downsample=%i"%(demo_name, args.downsample)
    print colorize(video_cmd, "green")
    video_handle = subprocess.Popen(video_cmd, shell=True)
    started_video = True

    
    time.sleep(9999)    

except KeyboardInterrupt:
    print colorize("got control-c", "green")

finally:
    
    if started_bag:
        print "stopping bag"
        bag_handle.send_signal(signal.SIGINT)
        bag_handle.wait()
    if started_video:
        print "stopping video"
        video_handle.send_signal(signal.SIGINT)
        video_handle.wait()


bagfilename = demo_name+".bag"
if yes_or_no("save demo?"):
    annfilename = demo_name+".ann.yaml"
    call_and_print("generate_annotations.py %s %s"%(bagfilename, annfilename),check=False)
    with open(args.master_file,"a") as fh:
        fh.write("\n"
            "- bag_file: %(bagfilename)s\n"
            "  annotation_file: %(annfilename)s\n"
            "  video_dir: %(videodir)s\n"
            "  demo_name: %(demoname)s"%dict(bagfilename=bagfilename, annfilename=annfilename, videodir=demo_name, demoname=demo_name))
else:
    if osp.exists(demo_name): shutil.rmtree(demo_name) #video dir
    if osp.exists(bagfilename): os.unlink(bagfilename)
