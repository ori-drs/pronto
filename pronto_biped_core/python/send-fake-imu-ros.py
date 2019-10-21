#!/usr/bin/python
import os,sys
import sys
import time

import rospy
import numpy as np

# ros msgs
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan

# this imports botpy currently...
from lcm_ros_conversions import *


def on_camera(data):
  send_imu(data.header.stamp)

def on_lidar(data):
  send_imu(data.header.stamp)

def send_imu(stamp):
  msg = Imu()
  msg.header.stamp = stamp
  pitch_degrees = 0
  q = euler_to_quat([0,pitch_degrees*np.pi/180.0,0])
  msg.orientation.w = q[0]
  msg.orientation.x = q[1]
  msg.orientation.y = q[2]
  msg.orientation.z = q[3]

  imu_pub.publish(msg)
  print "sent | pitch: ", pitch_degrees


####################################################################
rospy.init_node('send_fake_imu', anonymous=True)
imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
print "send_fake_imu started"

#rospy.Subscriber("/multisense/left/image_color", Image, on_camera)

# use lidar as fake image as it will have less latency
rospy.Subscriber("/multisense/lidar_scan", LaserScan, on_lidar)

rospy.spin()