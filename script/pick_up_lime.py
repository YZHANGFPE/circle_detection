#!/usr/bin/env python
import rospy
import tf
import numpy
import moveit_commander
import moveit_msgs.msg

from geometry_msgs.msg import Pose, Point, Quaternion

if __name__ == '__main__':
    rospy.init_node('pick_up_lime')

    tf_listener = tf.TransformListener()

    group = moveit_commander.MoveGroupCommander("left_arm")
    group.set_planner_id('RRTConnectkConfigDefault')

    # initial pose
    initial_pose = Pose()
    initial_pose.orientation.y = 1.0
    initial_pose.position.x = 0.6
    initial_pose.position.y = 0.48
    initial_pose.position.z = 0.144

    group.set_pose_target(initial_pose)
    plan1 = group.plan()
    group.execute(plan1)

    rospy.loginfo("Looking for the detected pose")

    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform('/base', '/circle', rospy.Time(0), rospy.Duration(4.0))
            break
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            continue
    trans, rot = tf_listener.lookupTransform('/base', '/circle', rospy.Time(0))

    rospy.loginfo(trans)

    # move to the position that is above the lime

    above_pose = Pose()
    above_pose.orientation.y = 1.0
    above_pose.position.x = trans[0]
    above_pose.position.y = trans[1]
    above_pose.position.z = 0.144
    group.set_pose_target(above_pose)
    plan2 = group.plan()
    group.execute(plan2)

    # set the start state as the end of the last plan
    # state = moveit_msgs.msg.RobotState()
    # state.joint_state.header = above_plan.joint_trajectory.header
    # state.joint_state.name = above_plan.joint_trajectory.joint_names
    # state.joint_state.position = above_plan.joint_trajectory.points[-1].positions
    # state.joint_state.velocity = above_plan.joint_trajectory.points[-1].velocities
    # state.joint_state.effort = above_plan.joint_trajectory.points[-1].effort

    # state.multi_dof_joint_state.header = above_plan.multi_dof_joint_trajectory.header
    # state.multi_dof_joint_state.joint_names = above_plan.multi_dof_joint_trajectory.joint_names

    # group.set_start_state(state)

    # set the orientation constraint during the approach
    constraints = moveit_commander.Constraints()
    constraints.name = "hold rotations"
    ocm = moveit_msgs.msg.OrientationConstraint()
    ocm.link_name = "left_hand"
    ocm.header.frame_id = "base"
    ocm.orientation = above_pose.orientation
    ocm.absolute_x_axis_tolerance = 100
    ocm.absolute_y_axis_tolerance = 100
    ocm.absolute_z_axis_tolerance = 100
    ocm.weight = 1.0

    constraints.orientation_constraints.append(ocm)

    group.set_path_constraints(constraints)

    pick_pose = Pose()
    pick_pose.orientation.y = 1.0
    pick_pose.position.x = trans[0]
    pick_pose.position.y = trans[1]
    pick_pose.position.z = -0.08
    group.set_pose_target(pick_pose)
    group.plan()
