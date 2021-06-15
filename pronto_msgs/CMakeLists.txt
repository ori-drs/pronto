cmake_minimum_required(VERSION 3.0.2)
project(pronto_msgs)

find_package(catkin REQUIRED roscpp rospy std_msgs geometry_msgs message_generation)

add_message_files(FILES  VisualOdometryUpdate.msg
                         FilterState.msg
                         GPSData.msg
                         IndexedMeasurement.msg
                         QuadrupedStance.msg
                         JointStateWithAcceleration.msg
                         LidarOdometryUpdate.msg
                         ControllerFootContact.msg
                         BipedForceTorqueSensors.msg
                         QuadrupedForceTorqueSensors.msg
                         VelocityWithSigmaBounds.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
