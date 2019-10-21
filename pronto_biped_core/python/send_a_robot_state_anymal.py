#!/usr/bin/python
# A very simple process to combine the floating base estimate
# with the kinematics and output the combined message
# input: POSE_BODY and ATLAS_STATE, output: EST_ROBOT_STATE
#
# currently this only works/used for Thor Mang

import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np
import numpy.random as random

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
#sys.path.append(home_dir + "/otherprojects/pronto-distro/build/lib/python2.7/site-packages")
#sys.path.append(home_dir + "/otherprojects/pronto-distro/build/lib/python2.7/dist-packages")

from bot_core.robot_state_t import robot_state_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)


# hyq:

joint_name_list = ['LF_HAA', 'LF_HFE', 'LF_KFE', 'RF_HAA', 'RF_HFE', 'RF_KFE', 'LH_HAA', 'LH_HFE', 'LH_KFE', 'RH_HAA', 'RH_HFE', 'RH_KFE','actuated_lidar_front_dynamixel']
#joint_name_list = ['ptu_tilt','ptu_pan','hokuyo_joint']#'lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint', 'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint', 'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint', 'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint', 'ptu_pan', 'ptu_tilt']


def send_state():
  o = robot_state_t()
  o.utime = timestamp_now ()
  o.num_joints = len(joint_name_list)
  #o.joint_name = ["" for x in range(o.num_joints)]
  o.joint_name = joint_name_list
  o.joint_position = random.rand(o.num_joints) -0.5

  #o.joint_position = [0.0, 0.0, -1.3962634015945001, 0.0, 0.0, -1.3962634015945001, 0.0, -2.220446049250313e-16, 1.3962634015945001, 0.0, -2.220446049250313e-16, 1.3962634015945001, 0.0, 1.1102230246251565e-16, 0,0]


  o.joint_velocity = [0]*o.num_joints
  o.joint_effort = [0]*o.num_joints


  o.pose.translation.x =0;
  o.pose.translation.y =0;
  o.pose.translation.z =2.3;
  o.pose.rotation.w = 1;
  o.pose.rotation.x = 0
  o.pose.rotation.y = 0
  o.pose.rotation.z = 0

  lc.publish("EST_ROBOT_STATE",o.encode())  

####################################################################
lc = lcm.LCM()
print "started"

send_state()
