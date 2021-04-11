#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import numpy as np
import csv
import time
import sys

global fixed_frame
#fixed_frame = "odom"
fixed_frame = "map"

def readPoses(filename):
    results =[]
    with open(filename, 'r') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in spamreader:
            # skip lines beginning with #
            if row[0][0] == "#":
                continue

            if row[0][0] == "V":
                continue
            #print row
            result = [float(i) for i in row]
            #print result
            results.append(result)
    #print results
    return results

def talker(results, output_topic):
    pub = rospy.Publisher(output_topic, Path, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    path_msg = Path()
    for result in results:
        pose = PoseStamped()
        pose.header.frame_id = fixed_frame
        # todo - do proper conversion to timestamp
        #counter = result[0]
        secs = int(result[0])
        nsecs = int(result[1])

        pose.header.stamp.secs = secs
        pose.header.stamp.nsecs = nsecs

        pose.pose.position.x=result[2]
        pose.pose.position.y=result[3]
        pose.pose.position.z=result[4]
        pose.pose.orientation.x=result[5]
        pose.pose.orientation.y=result[6]
        pose.pose.orientation.z=result[7]
        pose.pose.orientation.w=result[8]
        path_msg.poses.append(pose)

    path_msg.header.frame_id = fixed_frame

    #while not rospy.is_shutdown():
    hello_str = "publish path %s" % rospy.get_time() 
    hello_str+= " - " + str(len(path_msg.poses)) + " poses"
    rospy.loginfo(hello_str)
    pub.publish(path_msg)

    time.sleep(1)
    pub.publish(path_msg)

    time.sleep(1)
    pub.publish(path_msg)

    #rate.sleep()

if __name__ == '__main__':
    try:
        output_topic = '/trajectory'

        if (len(sys.argv) > 1):
            filename = sys.argv[1]
        if (len(sys.argv) > 2):
            output_topic = sys.argv[2]

        results=readPoses(filename)
        talker(results,output_topic)

    except rospy.ROSInterruptException:
        pass
