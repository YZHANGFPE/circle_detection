#!/usr/bin/env python
import rospy
import tf
import numpy
import moveit_commander
import moveit_msgs.msg
import sys
from geometry_msgs.msg import Pose, Point, Quaternion
from baxter_interface import Gripper
from itertools import izip

def get_pose_with_sleep(group, delay = 0.1):
    rospy.sleep(delay)
    return group.get_current_pose()

def plan_validate(plan, threshold = 0.2):        
    points = plan.joint_trajectory.points
    if len(points) == 0: return False        

    score = max(sum((x1 - x0)**2 for x0, x1 in izip(point0.positions, point1.positions)) 
                                 for point0, point1 in izip(points[:-1], points[1:]))

    rospy.loginfo('Plan scored %s for discontinuity' % score)
    return False if score > threshold else True

def plan_with_orientation_lock(group, target_pose, side = "left", threshold = 0.1):

    current_pose = get_pose_with_sleep(group)
    constraints = moveit_commander.Constraints()
    constraints.name = "hold rotations"
    ocm = moveit_msgs.msg.OrientationConstraint()
    if side == "left":
        ocm.link_name = "left_hand"
    else:
        ocm.link_name = "right_hand"

    ocm.header.frame_id = "base"
    ocm.orientation = current_pose.pose.orientation
    ocm.absolute_x_axis_tolerance = 0.2
    ocm.absolute_y_axis_tolerance = 0.2
    ocm.absolute_z_axis_tolerance = 0.2
    ocm.weight = 1.0
    
    constraints.orientation_constraints.append(ocm)
    group.clear_path_constraints()

    group.set_path_constraints(constraints)

    group.set_pose_target(target_pose)
    plan_valid = False
    while not rospy.is_shutdown() and not plan_valid:
        plan = group.plan()
        plan_valid = plan_validate(plan, threshold)
        print plan_valid
    group.clear_path_constraints()
    return plan

def execute_valid_plan(group, target_pose):
    plan_valid = False
    group.set_pose_target(target_pose)
    while not rospy.is_shutdown() and not plan_valid:
        plan = group.plan()
        plan_valid = plan_validate(plan)
        print plan_valid
    group.execute(plan)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_up_lime')

    tf_listener = tf.TransformListener()

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    gripper_left = Gripper("left")
    gripper_left.calibrate()
    gripper_right = Gripper("right")
    gripper_right.calibrate()
    rospy.sleep(2)

    group_left = moveit_commander.MoveGroupCommander("left_arm")
    group_right = moveit_commander.MoveGroupCommander("right_arm")
    group_left.set_planner_id('RRTConnectkConfigDefault')
    group_right.set_planner_id('RRTConnectkConfigDefault')

    # parameters
    no_detection = False
    drop_lemon = True
    press_faucet = False
    place_cup = False
    gripper_position_threshold = 4.3

    # clean the scene
    scene.remove_world_object()
    

    while not no_detection:

        # initial pose
        initial_pose = Pose()
        initial_pose.orientation.y = 1.0
        initial_pose.position.x = 0.5
        initial_pose.position.y = -0.68
        initial_pose.position.z = 0.144

        execute_valid_plan(group_right, initial_pose)

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
            execute_valid_plan(group_right, above_pose)

            # set the orientation constraint during the approach

            above_pose = get_pose_with_sleep(group_right)


            pick_pose = Pose()
            pick_pose.orientation = above_pose.pose.orientation
            pick_pose.position.x = trans[0]
            pick_pose.position.y = trans[1]
            pick_pose.position.z = -0.08

            execute_valid_plan(group_right, pick_pose)
            gripper_right.close()
            rospy.sleep(1)

            # fake motion
            print gripper_right.position()

            if gripper_right.position() > gripper_position_threshold:
                
                if drop_lemon:
                    initial_pose = get_pose_with_sleep(group_right)

                    lift_pose = get_pose_with_sleep(group_right)
                    lift_pose.pose.position.z += 0.40
                    execute_valid_plan(group_right, lift_pose)

                    drop_pose = get_pose_with_sleep(group_right)
                    drop_pose.pose.position.x = 0.76
                    drop_pose.pose.position.y = -0.05
                    execute_valid_plan(group_right, drop_pose)
                    gripper_right.open()
                    rospy.sleep(1)

                    execute_valid_plan(group_right, lift_pose)
                else:
                    initial_pose = get_pose_with_sleep(group_right)

                    lift_pose = get_pose_with_sleep(group_right)
                    lift_pose.pose.position.z += 0.06
                    execute_valid_plan(group_right, lift_pose)

                    execute_valid_plan(group_right, initial_pose)
                    gripper_right.open()
                    rospy.sleep(1)


            else:
                gripper_right.open()
                rospy.sleep(1)

    rospy.loginfo("No circle detection")

    while not rospy.is_shutdown():

        rospy.loginfo("Time to kill the process")

        start_time = rospy.get_time()
        while (rospy.get_time() - start_time < 4.0):
            if rospy.is_shutdown():
                no_detection = True
                press_faucet = False
                break

        if place_cup:
            # step 1
            pose1 = Pose()
            pose1.position.x = 0.551
            pose1.position.y = 0.565
            pose1.position.z = -0.0728
            pose1.orientation.x = -0.0140
            pose1.orientation.y = 0.866
            pose1.orientation.z = -0.0272
            pose1.orientation.w = 0.499

            execute_valid_plan(group_left, pose1)

            # step 2


            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.x+= 0.15
            plan = plan_with_orientation_lock(group_left, pose2)
            group_left.execute(plan)
            gripper_left.command_position(40)

            # step 3
            rospy.sleep(1)

            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.x-= 0.10
            plan = plan_with_orientation_lock(group_left, pose2)
            group_left.execute(plan)


            # step 4

            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.z+= 0.3
            plan = plan_with_orientation_lock(group_left, pose2)
            group_left.execute(plan)

            # step 5

            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.y-= 0.5
            plan = plan_with_orientation_lock(group_left, pose2)
            group_left.execute(plan)

            # step 6

            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.z-= 0.3
            plan = plan_with_orientation_lock(group_left, pose2)
            group_left.execute(plan)

            # step 7

            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.x+= 0.08
            plan = plan_with_orientation_lock(group_left, pose2, threshold = 0.01)
            group_left.execute(plan)

            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.y-= 0.02
            plan = plan_with_orientation_lock(group_left, pose2, threshold = 0.01)
            group_left.execute(plan)

            gripper_left.open()

            # step 8

            rospy.sleep(1)

            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.y-= 0.01
            plan = plan_with_orientation_lock(group_left, pose2, threshold = 0.01)
            group_left.execute(plan)

            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.x-= 0.08
            plan = plan_with_orientation_lock(group_left, pose2, threshold = 0.01)
            group_left.execute(plan)

            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.y+= 0.03
            plan = plan_with_orientation_lock(group_left, pose2, threshold = 0.01)
            group_left.execute(plan)

            # step 9

            pose2 = get_pose_with_sleep(group_left)
            pose2.pose.position.z+= 0.3
            plan = plan_with_orientation_lock(group_left, pose2)
            group_left.execute(plan)

            # step 10


            # pose2 = group_left.get_current_pose()
            # pose2.pose.position.y+= 0.5
            # plan = plan_with_orientation_lock(group_left, pose2)
            # group_left.execute(plan)

            # # step 11

            # pose2 = group_left.get_current_pose()
            # pose2.pose.position.z-= 0.3
            # plan = plan_with_orientation_lock(group_left, pose2)
            # group_left.execute(plan)

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
            group_left.set_joint_value_target(joint_states['observe_l'])
            rospy.sleep(1)
            plan_valid = False
            while not rospy.is_shutdown() and not plan_valid:
                plan = group_left.plan()
                plan_valid = plan_validate(plan)
                print plan_valid
            group_left.execute(plan)

            # move to the pose above the faucet
            faucet_pose = Pose()
            faucet_pose.orientation.x = -0.489
            faucet_pose.orientation.y = 0.517
            faucet_pose.orientation.z = -0.517
            faucet_pose.orientation.w = 0.476
            faucet_pose.position.x = 0.68
            faucet_pose.position.y = 0.05
            faucet_pose.position.z = 0.144
            execute_valid_plan(group_left, faucet_pose)


            # press down the faucet
            press_pose = get_pose_with_sleep(group_left)
            press_pose.pose.position.z -= 0.05
            plan = plan_with_orientation_lock(group_left, press_pose)
            group_left.execute(plan)
            rospy.sleep(5)
            release_pose = get_pose_with_sleep(group_left)
            release_pose.pose.position.z += 0.1
            release_pose.pose.position.x += 0.05
            plan = plan_with_orientation_lock(group_left, release_pose)
            group_left.execute(plan)

            group_left.set_joint_value_target(joint_states['observe_l'])
            plan_valid = False
            while not rospy.is_shutdown() and not plan_valid:
                plan = group_left.plan()
                plan_valid = plan_validate(plan)
                print plan_valid
            group_left.execute(plan)


