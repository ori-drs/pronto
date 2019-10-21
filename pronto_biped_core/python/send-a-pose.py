#!/usr/bin/python
import os,sys
import sys
import time

import rospy

home_dir =os.getenv("HOME")
#print home_dir
#sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
#sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

# this imports botpy currently...
from high_level_actions import *

import lcm
from bot_core.pose_t import pose_t

msg = pose_t()
msg.utime = 0
msg.pos = (0, 0, 0)
msg.orientation = euler_to_quat([0,0,0])

lc = lcm.LCM()
lc.publish("POSE_BODY", msg.encode())
print "send pose"