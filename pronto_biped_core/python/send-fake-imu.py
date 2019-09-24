#!/usr/bin/python
import os,sys
import sys
import time

import rospy
import numpy as np

home_dir =os.getenv("HOME")
#print home_dir
#sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
#sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

# this imports botpy currently...
from lcm_ros_conversions import *

import lcm
from bot_core.ins_t import ins_t
from bot_core.images_t import images_t

def on_camera(channel, data):
  m = images_t.decode(data) # its stupid to decode an image to get a timestamp
  msg = ins_t()
  pitch_degrees = 0
  msg.quat = euler_to_quat([0,pitch_degrees*np.pi/180.0,0])

  lc.publish("IMU_MICROSTRAIN", msg.encode())
  print "sent ", m.utime ," | pitch: ", pitch_degrees

lc = lcm.LCM()
print "start"

sub2 = lc.subscribe("MULTISENSE_CAMERA", on_camera)

while True:
  lc.handle()

