#!/usr/bin/env python
import rospy
import tf
import numpy
import moveit_commander
import moveit_msgs.msg
import sys
from geometry_msgs.msg import Pose, Point, Quaternion
from baxter_interface import Gripper

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_up_lime')

    tf_listener = tf.TransformListener()

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    gripper = Gripper("left")
    gripper.calibrate()
    rospy.sleep(3)

    group = moveit_commander.MoveGroupCommander("left_arm")
    group.set_planner_id('RRTConnectkConfigDefault')

    # parameters
    no_detection = True
    drop_lemon = False
    press_faucet = False
    gripper_position_threshold = 4.3

    # clean the scene
    scene.remove_world_object()
    

    gripper.close()
    rospy.sleep(1)
    # pose1 = group.get_current_pose()
    # pose1.pose.position.z += 0.30
    # group.set_pose_target(pose1)
    # group.plan()

    # pose1 = group.get_current_pose()
    # pose1.pose.position.z -= 0.30
    # group.set_pose_target(pose1)
    # group.plan()

    pose1 = group.get_current_pose()
    pose1.pose.position.x += 0.10
    group.set_pose_target(pose1)
    group.go()

    gripper.open()
    rospy.sleep(1)

    pose2 = group.get_current_pose()
    pose2.pose.position.x -= 0.10
    group.set_pose_target(pose2)
    group.go()


    while not no_detection:

        # initial pose
        initial_pose = Pose()
        initial_pose.orientation.y = 1.0
        initial_pose.position.x = 0.6
        initial_pose.position.y = 0.48
        initial_pose.position.z = 0.144

        group.set_pose_target(initial_pose)
        group.go()

        rospy.loginfo("Time to kill the process")

        start_time = rospy.get_time()
        while (rospy.get_time() - start_time < 4.0):
            if rospy.is_shutdown():
                no_detection = True
                press_faucet = False
                break

        rospy.loginfo("Looking for the detected pose")


        start_time = rospy.get_time()

        while not rospy.is_shutdown():

            if(rospy.get_time() - start_time > 4.0):
                no_detection = True
                break
            try:
                tf_listener.waitForTransform('/base', '/circle', rospy.Time(0), rospy.Duration(1.0))
                break
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                continue

        if not no_detection:
            time_delay = rospy.get_time() - tf_listener.getLatestCommonTime('/circle' , '/base').to_sec()
            rospy.loginfo("The time delay for the detection is: %f sec" , time_delay)
            if(time_delay > 3.0):
                rospy.loginfo("Time delay of the detection exceeds 2.0 second. Stop the process")
                break

            trans, rot = tf_listener.lookupTransform('/base', '/circle', rospy.Time(0))

            rospy.loginfo(trans)

            # move to the position that is above the lime
            above_pose = Pose()
            above_pose.orientation.y = 1.0
            above_pose.position.x = trans[0]
            above_pose.position.y = trans[1]
            above_pose.position.z = 0.144
            group.set_pose_target(above_pose)
            group.go()

            # set the orientation constraint during the approach

            above_pose = group.get_current_pose()


            pick_pose = Pose()
            pick_pose.orientation = above_pose.pose.orientation
            pick_pose.position.x = trans[0]
            pick_pose.position.y = trans[1]
            pick_pose.position.z = -0.08

            group.set_pose_target(pick_pose)

            constraints = moveit_commander.Constraints()
            constraints.name = "hold rotations"
            ocm = moveit_msgs.msg.OrientationConstraint()
            ocm.link_name = "left_hand"
            ocm.header.frame_id = "base"
            ocm.orientation = pick_pose.orientation
            ocm.absolute_x_axis_tolerance = 0.2
            ocm.absolute_y_axis_tolerance = 0.2
            ocm.absolute_z_axis_tolerance = 0.2
            ocm.weight = 1.0
            
            constraints.orientation_constraints.append(ocm)
            group.clear_path_constraints()

            group.set_path_constraints(constraints)

            group.go()
            group.clear_path_constraints()
            gripper.close()
            rospy.sleep(1)

            # fake motion
            print gripper.position()

            if gripper.position() > gripper_position_threshold:
                
                if drop_lemon:
                    initial_pose = group.get_current_pose()

                    lift_pose = group.get_current_pose()
                    lift_pose.pose.position.z += 0.40
                    group.set_pose_target(lift_pose)
                    group.go()

                    drop_pose = group.get_current_pose()
                    drop_pose.pose.position.x = 0.76
                    drop_pose.pose.position.y = -0.05
                    group.set_pose_target(drop_pose)
                    group.go()
                    gripper.open()
                    rospy.sleep(1)

                    group.set_pose_target(lift_pose)
                    group.go()
                else:
                    initial_pose = group.get_current_pose()

                    lift_pose = group.get_current_pose()
                    lift_pose.pose.position.z += 0.06
                    group.set_pose_target(lift_pose)
                    group.go()

                    group.set_pose_target(initial_pose)
                    group.go()
                    gripper.open()
                    rospy.sleep(1)


            else:
                gripper.open()
                rospy.sleep(1)

    rospy.loginfo("No circle detection")

    if press_faucet:
        # press faucet

        joint_states = {
            'observe_r':
                [-0.7255729118408204,
                 -0.4893398707763672,
                 0.022626216595458985,
                 1.9788352141113283,
                 -1.549704089190674,
                 -1.5301458341674805,
                 -0.016106798254394532],
            'observe_l':
                [0.7255729118408204,
                 -0.4893398707763672,
                 0.022626216595458985,
                 1.9788352141113283,
                 1.549704089190674,
                 -1.5301458341674805,
                 -0.016106798254394532]
                
        }
        group.set_joint_value_target(joint_states['observe_l'])
        group.go()

        # move to the pose above the faucet
        faucet_pose = Pose()
        faucet_pose.orientation.x = -0.489
        faucet_pose.orientation.y = 0.517
        faucet_pose.orientation.z = -0.517
        faucet_pose.orientation.w = 0.476
        faucet_pose.position.x = 0.69
        faucet_pose.position.y = 0.10
        faucet_pose.position.z = 0.144
        group.set_pose_target(faucet_pose)
        group.go()


        # press down the faucet
        press_pose = group.get_current_pose()
        press_pose.pose.position.z -= 0.06
        group.set_pose_target(press_pose)
        group.go()
        rospy.sleep(5)
        release_pose = group.get_current_pose()
        release_pose.pose.position.z += 0.1
        group.set_pose_target(release_pose)
        group.go()

        group.set_joint_value_target(joint_states['observe_l'])
        group.go()


