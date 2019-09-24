#!/usr/bin/python
import os,sys
import rospy
from joy_manager_msgs.msg import AnyJoy
from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import JointState

#import os,sys
#import lcm
#import time
#from lcm import LCM
#from math import *


#from bot_core.pose_t import pose_t
#from bot_core.robot_state_t import robot_state_t
#from bot_core.joint_state_t import joint_state_t
#from bot_core.vector_3d_t import vector_3d_t
#from bot_core.position_3d_t import position_3d_t
#from bot_core.twist_t import twist_t
#from bot_core.quaternion_t import quaternion_t
#from bot_core.twist_t import twist_t
#from bot_core.force_torque_t import force_torque_t

########################################################################################
#def timestamp_now (): return int (time.time () * 1000000)


#rospy.init_node('talker', anonymous=True)

pub = rospy.Publisher('/commands/pose', PoseStamped, queue_size=10)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard")
    
    msg = PoseStamped()
    t = rospy.get_rostime() #get time as rospy.Time instance
    #print t

    # 0 : lateral in range -1:1
    # 1 : for/bk in range -1:1
    # 2 : unused part of right joiy
    # 3 : turning -1:1
    msg.header.stamp = data.header.stamp
    msg.header.frame_id = 'base'
    msg.pose.position.x = 2
    msg.pose.position.y = 0
    msg.pose.position.z = 1

    msg.pose.orientation.w = 1
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = 0

    pub.publish(msg)



def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensors/joint_states", JointState, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()






if __name__ == '__main__':
    listener()


# ####################################################################
# lc = lcm.LCM()
# print "started"

# sub2 = lc.subscribe("PATH_FOLLOWER_CMD", on_follower)


# while True:
#   lc.handle()