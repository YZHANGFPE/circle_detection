#!/usr/bin/env python
import rospy
import tf
import numpy
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion

if __name__ == '__main__':
    rospy.init_node('pick_up_lime')

    tf_listener = tf.TransformListener()

    # group = moveit_commander.MoveGroupCommander("left_arm")
    # group.set_planner_id('RRTConnectkConfigDefault')

    # # initial pose
    # initial_pose = Pose()
    # initial_pose.orientation.y = 1.0
    # initial_pose.position.x = 0.6
    # initial_pose.position.y = 0.48
    # initial_pose.position.z = 0.144

    # group.set_pose_target(initial_pose)
    # group.go()

    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.y = 1.0
    # pose_target.position.x = 0.7
    # pose_target.position.y = -0.05
    # pose_target.position.z = 1.1
    # group.set_pose_target(pose_target)
    rospy.loginfo("Looking for the detected pose")

    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform('/base', '/circle', rospy.Time(0), rospy.Duration(4.0))
            break
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            continue
    trans, rot = tf_listener.lookupTransform('/base', '/circle', rospy.Time(0))

    rospy.loginfo(trans)

    above_pose = Pose()
    above_pose.orientation.y = 1.0
    above_pose.position.x = trans[0]
    above_pose.position.y = trans[1]
    above_pose.position.z = 0.144
    group.set_pose_target(above_pose)
    group.go()

    pick_pose = Pose()
    pick_pose.orientation.y = 1.0
    pick_pose.position.x = trans[0]
    pick_pose.position.y = trans[1]
    pick_pose.position.z = -0.03
    group.set_pose_target(pick_pose)
    group.go()

    rospy.sleep(2)