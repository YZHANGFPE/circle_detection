#!/usr/bin/env python
import rospy
import tf
import numpy as np
import moveit_commander
import moveit_msgs.msg
import sys
from geometry_msgs.msg import Pose, Point, Quaternion
from baxter_interface import Gripper
from itertools import izip
from baxter_pykdl import baxter_kinematics

def get_pose_with_sleep(group, delay = 0.1):
    rospy.sleep(delay)
    pose = group.get_current_pose()
    return pose

def plan_validate(plan,  dx, dy , dz,  threshold = 0.2):        
    points = plan.joint_trajectory.points
    if len(points) == 0: return False        

    score = max(sum((x1 - x0)**2 for x0, x1 in izip(point0.positions, point1.positions)) 
                                 for point0, point1 in izip(points[:-1], points[1:]))
    kin = baxter_kinematics('left')

    rospy.loginfo('Plan scored %s for discontinuity' % score)

    # points range in x, y, z direction
    initial_pose = kin.forward_position_kinematics(points[0].positions)
    maxx = initial_pose[0]
    maxy = initial_pose[1]
    maxz = initial_pose[2]
    minx = maxx
    miny = maxy
    minz = maxz
    for point in points:
        pose = kin.forward_position_kinematics(point.positions)
        maxx = max(maxx,pose[0])
        maxy = max(maxy,pose[1])
        maxz = max(maxz,pose[2])
        minx = min(minx,pose[0])
        miny = min(miny,pose[1])
        minz = min(minz,pose[2])
    print (maxx-minx), (maxy-miny), (maxz-minz)
    if (maxx-minx) < dx and (maxy-miny) < dy and (maxz-minz) < dz:
        withInRange = True
    else:
        withInRange = False
    print dx, dy, dz
    print withInRange

    return False if score > threshold or not withInRange else True

def plan_with_orientation_lock(group, target_pose, side = "left", threshold = 0.1, tolerance = 0.2):

    current_pose = get_pose_with_sleep(group)
    constraints = moveit_commander.Constraints()
    constraints.name = "hold rotations"
    ocm = moveit_msgs.msg.OrientationConstraint()
    if side == "left":
        ocm.link_name = "left_gripper"
    else:
        ocm.link_name = "right_hand"

    ocm.header.frame_id = "base"
    ocm.orientation = current_pose.pose.orientation
    ocm.absolute_x_axis_tolerance = tolerance
    ocm.absolute_y_axis_tolerance = tolerance
    ocm.absolute_z_axis_tolerance = tolerance
    ocm.weight = 1.0
    
    constraints.orientation_constraints.append(ocm)
    group.clear_path_constraints()

    group.set_path_constraints(constraints)

    group.set_pose_target(target_pose)
    plan_valid = False
    while not rospy.is_shutdown() and not plan_valid:
        plan = group.plan()
        plan_valid = plan_validate(plan, 100, 100, 100, threshold)
        print plan_valid
    group.clear_path_constraints()
    return plan

def execute_valid_plan(group, target_pose, dx = 100, dy = 100, dz = 100):
    plan_valid = False
    group.set_pose_target(target_pose)
    while not rospy.is_shutdown() and not plan_valid:
        plan = group.plan()
        plan_valid = plan_validate(plan, dx, dy, dz)
        print plan_valid
    group.execute(plan)

def put(object, count):

    tf_listener = tf.TransformListener()

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    gripper_left = Gripper("left")
    gripper_right = Gripper("right")
    group_left = moveit_commander.MoveGroupCommander("left_arm")
    group_right = moveit_commander.MoveGroupCommander("right_arm")
    group_left.set_planner_id('RRTConnectkConfigDefault')
    group_right.set_planner_id('RRTConnectkConfigDefault')

    # parameters
    no_detection = False
    drop_lemon = True
    press_faucet = False
    place_cup = False
    gripper_position_threshold = 4.6
    # count = 3 # number of lemon slices

    # clean the scene
    # scene.remove_world_object()

    # forward kinematic object
    kin = baxter_kinematics('left')
    

    while True:
        
        
        rospy.sleep(1)
        # initial pose
        initial_pose = Pose()
        # initial_pose.orientation.x = 0.9495
        # initial_pose.orientation.y = 0.017127
        # initial_pose.orientation.z = -0.31322
        # initial_pose.orientation.w = 0.0071202
        # initial_pose.orientation.x = 0.37
        # initial_pose.orientation.y = 0.93
        # initial_pose.orientation.z = 0.0
        # initial_pose.orientation.w = 0.0
        initial_pose.orientation.y = 1.0
        initial_pose.position.x = 0.0
        initial_pose.position.y = 0.88
        initial_pose.position.z = 0.214 
        rospy.sleep(1)
        execute_valid_plan(group_left, initial_pose)

        if count == 0:
            rospy.loginfo("Finish the task")
            break

        if no_detection:
            rospy.loginfo("No detection")
            break

        rospy.loginfo("Time to kill the process")

        start_time = rospy.get_time()
        while (rospy.get_time() - start_time < 3.0):
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
            above_pose = get_pose_with_sleep(group_left)
            above_pose.pose.position.x = trans[0]
            above_pose.pose.position.y = trans[1]

            #plan = plan_with_orientation_lock(group_left, above_pose, side = "left", tolerance=1000)
            #group_left.execute(plan)
            execute_valid_plan(group_left, above_pose)

            # set the orientation constraint during the approach

            pick_pose = get_pose_with_sleep(group_left)
            # pick_pose.pose.position.x = trans[0]
            # pick_pose.pose.position.y = trans[1]
            pick_pose.pose.position.z = -0.025 # -0.08 for the table
            #plan = plan_with_orientation_lock(group_left, pick_pose, side = "left", tolerance=1000)
            #group_left.execute(plan)
            execute_valid_plan(group_left, pick_pose, dx = 0.02, dy = 0.02)
            gripper_left.close()
            rospy.sleep(1)

            
            print gripper_left.position()

            if gripper_left.position() > gripper_position_threshold:

                count-=1
                
                if drop_lemon:
                    initial_pose = get_pose_with_sleep(group_left)

                    lift_pose = get_pose_with_sleep(group_left)
                    lift_pose.pose.position.z += 0.35
                    # plan = plan_with_orientation_lock(group_left, lift_pose, side = "left")
                    # group_left.execute(plan)
                    execute_valid_plan(group_left, lift_pose, dx = 0.05, dy = 0.05)

                    pre_drop_pose = get_pose_with_sleep(group_left)
                    pre_drop_pose.pose.position.x = 0.75
                    pre_drop_pose.pose.position.y = 0.5
                    # execute_valid_plan(group_left, pre_drop_pose)


                    drop_pose = get_pose_with_sleep(group_left)
                    drop_pose.pose.position.x = 0.76
                    drop_pose.pose.position.y = 0.00
                    drop_pose.pose.position.z += 0.05  
                    # plan = plan_with_orientation_lock(group_left, drop_pose, side = "left", threshold= 1)
                    # group_left.execute(plan)

                    execute_valid_plan(group_left, drop_pose)
                    gripper_left.open()
                    rospy.sleep(1)

                    # execute_valid_plan(group_left, pre_drop_pose)

                    lift_pose2 = get_pose_with_sleep(group_left)
                    lift_pose2.pose.position.x = lift_pose.pose.position.x
                    lift_pose2.pose.position.y = lift_pose.pose.position.y
                    lift_pose2.pose.position.z = lift_pose.pose.position.z
                    # plan = plan_with_orientation_lock(group_left, lift_pose2, side = "left", threshold= 1)
                    # group_left.execute(plan)
                    execute_valid_plan(group_left, lift_pose)

                    # fake motion
                else:
                    initial_pose = get_pose_with_sleep(group_left)

                    lift_pose = get_pose_with_sleep(group_left)
                    lift_pose.pose.position.z += 0.1
                    execute_valid_plan(group_left, lift_pose,  dx = 0.01, dy = 0.01)
                    #plan = plan_with_orientation_lock(group_left, lift_pose, side = "left")
                    #group_left.execute(plan)

                    execute_valid_plan(group_left, initial_pose,dx = 0.01, dy = 0.01)
                    gripper_left.open()
                    rospy.sleep(1)


            else:
                gripper_left.open()
                rospy.sleep(1)

    # rospy.loginfo("No circle detection")

def serve():

    tf_listener = tf.TransformListener()

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    gripper_left = Gripper("left")
    gripper_right = Gripper("right")

    group_left = moveit_commander.MoveGroupCommander("left_arm")
    group_right = moveit_commander.MoveGroupCommander("right_arm")
    group_left.set_planner_id('RRTConnectkConfigDefault')
    group_right.set_planner_id('RRTConnectkConfigDefault')

    # parameters

    press_faucet = True
    place_cup = False
    gripper_position_threshold = 4.6

    # clean the scene
    scene.remove_world_object()

    # forward kinematic object
    kin = baxter_kinematics('left')

    rospy.loginfo("Time to kill the process")

    start_time = rospy.get_time()
    while (rospy.get_time() - start_time < 1.0):
        if rospy.is_shutdown():
            no_detection = True
            press_faucet = False
            break

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
    

    if place_cup:

        group_left.set_joint_value_target(joint_states['observe_l'])
        plan_valid = False
        while not rospy.is_shutdown() and not plan_valid:
            plan = group_left.plan()
            plan_valid = plan_validate(plan, 100,100,100)
            print plan_valid
        group_left.execute(plan)

        # step 1
        pose1 = Pose()
        pose1.position.x = 0.551
        pose1.position.y = 0.615
        pose1.position.z = -0.0728
        pose1.orientation.x = -0.0140
        pose1.orientation.y = 0.866
        pose1.orientation.z = -0.0272
        pose1.orientation.w = 0.499

        # execute_valid_plan(group_left, pose1)
        # print group_left.get_current_joint_values()

        group_left.set_joint_value_target([0.08283496245117188, -0.26192721923217777, 0.0609757362487793, 1.7851701398620607, 0.9625729432983399, -1.0622816943969726, -0.14841264105834961])
        rospy.sleep(1)
        plan_valid = False
        while not rospy.is_shutdown() and not plan_valid:
            plan = group_left.plan()
            plan_valid = plan_validate(plan, 100,100,100)
            print plan_valid
        group_left.execute(plan)

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
        plan = plan_with_orientation_lock(group_left, pose2, threshold = 0.2)
        group_left.execute(plan)

        # step 6

        pose2 = get_pose_with_sleep(group_left)
        pose2.pose.position.x-= 0.03
        plan = plan_with_orientation_lock(group_left, pose2)
        group_left.execute(plan)

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

        # pose2 = get_pose_with_sleep(group_left)
        # pose2.pose.position.y-= 0.01
        # plan = plan_with_orientation_lock(group_left, pose2, threshold = 0.01)
        # group_left.execute(plan)

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


    if press_faucet:
        # press faucet

        group_left.set_joint_value_target(joint_states['observe_l'])
        rospy.sleep(1)
        plan_valid = False
        while not rospy.is_shutdown() and not plan_valid:
            plan = group_left.plan()
            plan_valid = plan_validate(plan, 100,100,100)
            print plan_valid
        group_left.execute(plan)

        pre_pose = get_pose_with_sleep(group_left)
        pre_pose.pose.position.z += 0.1
        plan = plan_with_orientation_lock(group_left, pre_pose)
        group_left.execute(plan)

        # move to the pose above the faucet
        faucet_pose = Pose()
        faucet_pose.orientation.x = -0.489
        faucet_pose.orientation.y = 0.517
        faucet_pose.orientation.z = -0.517
        faucet_pose.orientation.w = 0.476
        faucet_pose.position.x = 0.69
        faucet_pose.position.y = 0.18
        faucet_pose.position.z = 0.244
        execute_valid_plan(group_left, faucet_pose)


        # press down the faucet
        press_pose = get_pose_with_sleep(group_left)
        press_pose.pose.position.z -= 0.13
        plan = plan_with_orientation_lock(group_left, press_pose)
        group_left.execute(plan)

        press_pose = get_pose_with_sleep(group_left)
        press_pose.pose.position.z -= 0.06
        plan = plan_with_orientation_lock(group_left, press_pose)
        group_left.execute(plan)
        rospy.sleep(5)
        release_pose = get_pose_with_sleep(group_left)
        release_pose.pose.position.z += 0.18
        release_pose.pose.position.y += 0.1
        release_pose.pose.position.x += 0.05
        plan = plan_with_orientation_lock(group_left, release_pose)
        group_left.execute(plan)

        group_left.set_joint_value_target(joint_states['observe_l'])
        plan_valid = False
        while not rospy.is_shutdown() and not plan_valid:
            plan = group_left.plan()
            plan_valid = plan_validate(plan, 100, 100, 100, threshold = 0.03)
            print plan_valid
        group_left.execute(plan)






if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_up_lime')

    recipe = "serve()"

    if "put"  in recipe or "serve" in recipe:
        try:
            compiled_recipe = compile(recipe, 'compiled_recipe', 'exec')
        except Exception as e:
            print e.message
            rospy.logerr("Failed to parse message!")

        print compiled_recipe

        exec(compiled_recipe)
    else:
        print "Execute pick and place"

    # put('lemon', 2)


        


