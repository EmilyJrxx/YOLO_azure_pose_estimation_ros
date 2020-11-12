#!/usr/bin/python

import sys, rospy, tf, moveit_commander, math, moveit_msgs.msg
import tf2_ros, tf2_geometry_msgs
import numpy as np
import copy

from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from moveit_python import PlanningSceneInterface
# from sensor_msgs.msg import Image
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from time import sleep

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

#from DrawManyTurtle.py import rob_x,rob_y

import turtle as tl
import math
rob_x =None
rob_y =None
def DrawManyTurtle():
    data = []
    points = 5000  

    N = 1000 + 1 
    x = [0] * N
    y = [0] * N
    global rob_x
    global rob_y
    rob_x = [0] * 5000
    rob_y = [0] * 5000
    rob_x_min = 0
    rob_x_max = 0
    rob_y_min = 0
    rob_y_max = 0
    # f = open("datas0" + ".txt", "r")
    f = open(r'/home/xxwang/ROS_Workspaces/ros_k4a_auboi5_ws/src/azure_kinect_test/scripts/xji/draw test/datas0.txt')
    tl.penup()
    tl.pensize(2)
    for line in f:
        line = eval(line)
        data.append(line)

        # tl.setup(960,720)

       
    for t in range(points):
        for i in range(len(data)):
            if i % 2 == 0:
                x[i] = data[i][0] * math.cos(i / points * 3.14 * t) - data[i][1] * math.sin(i / points * 3.14 * t)
                y[i] = data[i][0] * math.sin(i / points * 3.14 * t) + data[i][1] * math.cos(i / points * 3.14 * t)
            else:
                x[i] = data[i][0] * math.cos(-(i + 1) / points * 3.14 * t) - data[i][1] * math.sin(
                    -(i + 1) / points * 3.14 * t)
                y[i] = data[i][0] * math.sin(-(i + 1) / points * 3.14 * t) + data[i][1] * math.cos(
                    -(i + 1) / points * 3.14 * t)

        rob_x[t] = sum(x) / 3750
        rob_y[t] = sum(y) / 3750
        # print("rob_x[(%d)]= %f" % (t, rob_x[t]))
        # print("rob_y[(%d)]= %f" % (t, rob_y[t]))
        # if rob_x_max < rob_x[t]:
        #     rob_x_max = rob_x[t]
        # if rob_x_min > rob_x[t]:
        #     rob_x_min = rob_x[t]
        # if rob_y_max < rob_y[t]:
        #     rob_y_max = rob_y[t]
        # if rob_y_min > rob_y[t]:
        #     rob_y_min = rob_y[t]
        # tl.goto(int(sum(x)) / 2, -int(sum(y) / 2))  
        # tl.pendown()

def nothing(x):
    pass

def all_close(goal, actual, tolerance):
    """
    ============================================================
    Convenient method for testing if a list of values are within 
    a tolerance of their counterparts in another list.
    ============================================================
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal=True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True

class aubo_vision_pick(object):
    def __init__(self):
        super(aubo_vision_pick, self).__init__() # calling parent class
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('aubo_mp', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator_i5" 
        # Initialize the move group for the aubo
        group = moveit_commander.MoveGroupCommander(group_name)
        # Set global reference frame
        reference_frame = "world"
        # Set aubo_arm reference frame accordingly
        group.set_pose_reference_frame(reference_frame)
        # Allow replanning to increase odds of a solution
        group.allow_replanning(True)

        display_trajectory_publisher = rospy.Publisher('/move_group/planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame = group.get_planning_frame()
        print ("============ Reference frame: %s ============ " % planning_frame)
        # Get the name of the end-effector link
        eef_link = group.get_end_effector_link()
        print ("============ End effector: %s ============ " % eef_link)
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()
        print "============ Printing robot state"
        print robot.get_current_state()

        # Allow some leeway in position (meters) and orientation (radians)
        # 
        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.01)
        group.set_planning_time(0.1)
        # 
        group.set_max_acceleration_scaling_factor(.5)
        group.set_max_velocity_scaling_factor(.8)

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.track_flag = False
        self.default_pose_flag = True

        self.target_point = PointStamped()
        # Subscribe to recognition results
        rate = rospy.Rate(10)

    def go_to_ready_pose(self):

        group = self.group
        default_joint_states = group.get_current_joint_values()
        print(type(default_joint_states), default_joint_states)

	### Specify Ready Pose Configuration:
        ready_pose=geometry_msgs.msg.Pose()
        ready_pose.position.x = -0.4612
        ready_pose.position.y = -0.1438
        ### fixed value! DO NOT CHANGE! 
        ######################################
        ready_pose.position.z = 0.635 
        ready_pose.orientation.x = 0.7109
        ready_pose.orientation.y = -0.7031
        ready_pose.orientation.z = 0.0
        ready_pose.orientation.w = 0.0
        ######################################
	
	### Execution:
        group.set_pose_target(ready_pose)
        group.go(wait=True)
        # group.stop()
        rospy.sleep(1)

	### Status Check
        current_joints = group.get_current_joint_values()
        current_pose = self.group.get_current_pose().pose
        print("current pose:", current_pose)
        return all_close(default_joint_states, current_joints, 0.01)

    def lift_up(self):
        group = self.group
        current_pose = group.get_current_pose().pose
        pose_goal=geometry_msgs.msg.Pose()

        # define pose with specific parameters
        pose_goal = current_pose
        pose_goal.position.z += 0.1
        
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        # group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)
        return all_close(pose_goal, current_pose, 0.01)

        # Similar to <lift_up>
    def put_down(self):
        group = self.group
        current_pose = group.get_current_pose().pose
        pose_goal=geometry_msgs.msg.Pose()

        # define pose with specific parameters
        pose_goal = current_pose
        pose_goal.position.z -= 0.1
        
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        # group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)
        return all_close(pose_goal, current_pose, 0.01)
	######################
	# Your code here 
	######################

        # Similar to <lift_up>
    def goto_next(self, scale):
        group = self.group
        current_pose = group.get_current_pose().pose
        pose_goal=geometry_msgs.msg.Pose()

        # define pose with specific parameters
        pose_goal = current_pose
        pose_goal.position.y += scale*0.075
        
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        # group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)
        return all_close(pose_goal, current_pose, 0.01)
	######################
	# Your code here 
	######################

        #change of position.z should be fixed as 0.03

    def draw_fourier(self, scale):
        
        waypoints = []
        DrawManyTurtle()
        group = self.group
        start_pose = group.get_current_pose().pose
        print("Current pose: ", start_pose)

        self.put_down()
        # the first stroke
        wpose = group.get_current_pose().pose
        for t in range(5000):
            wpose.position.x += rob_x[t]
            wpose.position.y += rob_y[t]
            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )
        group.execute(plan, wait=True)

        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)

        group.set_pose_target(start_pose)
        group.go(wait=True)
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)

	######################
	# Your code here 
	######################


if __name__=="__main__":
    
    aubo_move = aubo_vision_pick()
    
    print "==== Press `Enter` to go to ready pose ===="
    raw_input()
    aubo_move.go_to_ready_pose()

    scale = 0.5

    print "==== Press `Enter` to write F ===="
    # raw_input() is one way you can test your patterns respectively
    aubo_move.draw_fourier(scale)

    print "==== Press `Enter` to write haha ===="
    # raw_input()
    aubo_move.goto_next(scale)

    # and D, A & N
