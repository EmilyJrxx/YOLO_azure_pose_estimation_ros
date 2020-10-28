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
        print ("============ Robot Groups:", robot.get_group_names())
        print ("============ Printing robot state")
        print (robot.get_current_state())

        # Allow some leeway in position (meters) and orientation (radians)
        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.01)
        group.set_planning_time(0.1)
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

	######################
	# Your code here 
	######################
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
        # Similar to <lift_up>
        
    def goto_next(self, scale):

	######################
	# Your code here 
        group = self.group
        current_pose = group.get_current_pose().pose
        pose_goal=geometry_msgs.msg.Pose()

        # define pose with specific parameters
        pose_goal = current_pose
        pose_goal.position.y += scale * 0.075
        
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        # group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)
        return all_close(pose_goal, current_pose, 0.01)
	######################
        
        #change of position.z should be fixed as 0.03

    def write_F(self, scale):
        waypoints = []

        group = self.group
        start_pose = group.get_current_pose().pose
        print("Current pose: ", start_pose)

        self.put_down()
        # the first stroke
        wpose = group.get_current_pose().pose
        wpose.position.y += scale * 0.05
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        
        # the second stroke
        wpose.position.z -= 0.03
        wpose.position.y -= scale * 0.05
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        
        # the third stroke
        wpose.position.z -= 0.03
        wpose.position.x -= scale * 0.05
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += scale * 0.05
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

    def write_U(self, scale):

	######################
	# Your code here 
        waypoints = []

        group = self.group
        start_pose = group.get_current_pose().pose
        print("Current pose: ", start_pose)
        self.put_down()
        # the first stroke
        wpose = group.get_current_pose().pose
        # wpose.position.z -= 0.03
        # waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        # the second stroke
        wpose.position.z -= 0.03
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += scale * 0.05
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        # the third stroke
        wpose.position.z -= 0.03
        wpose.position.x -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += scale * 0.1
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
	
    def write_D(self, scale):

	######################
	# Your code here 
        waypoints = []

        group = self.group
        start_pose = group.get_current_pose().pose
        print("Current pose: ", start_pose)
        self.put_down()
        # the first stroke
        wpose = group.get_current_pose().pose
        # wpose.position.z -= 0.03
        # waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        # the second stroke
        wpose.position.z -= 0.03
        wpose.position.x -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += scale * 0.05
        wpose.position.x += scale * 0.05
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        # the third stroke
        wpose.position.z -= 0.03
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= scale * 0.05
        wpose.position.x += scale * 0.05
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

    def write_A(self, scale):
	
	######################
	# Your code here 
        waypoints = []

        group = self.group
        start_pose = group.get_current_pose().pose
        print("Current pose: ", start_pose)

        #print("Current pose: ", start_pose)
        self.put_down()
        # the first stroke
        wpose = group.get_current_pose().pose
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += scale * 0.05
        waypoints.append(copy.deepcopy(wpose))
        # the first stroke
        wpose.position.z -= 0.03
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += scale * 0.1
        wpose.position.y -= scale * 0.025
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        # the second stroke
        wpose.position.z -= 0.03
        wpose.position.x -= scale * 0.05
        wpose.position.y += scale * 0.0125
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += scale * 0.025
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        # the third stroke
        wpose.position.z -= 0.03
        wpose.position.x -= scale * 0.05
        wpose.position.y -= scale * 0.0125
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += scale * 0.025
        wpose.position.x += scale * 0.1
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

    def write_N(self, scale):

	######################
	# Your code here 
        waypoints = []

        group = self.group
        start_pose = group.get_current_pose().pose
        print("Current pose: ", start_pose)

        self.put_down()
        # the first stroke
        wpose = group.get_current_pose().pose
        # wpose.position.z -= 0.03
        # waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        # the second stroke
        wpose.position.z -= 0.03
        wpose.position.x -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += scale * 0.05
        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))
        # the third stroke
        wpose.position.z -= 0.03
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x -= scale * 0.1
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


if __name__=="__main__":

    aubo_move = aubo_vision_pick()
    
    print("==== Press `Enter` to go to ready pose ====")
    raw_input()
    aubo_move.go_to_ready_pose()

    scale = 0.5
    print("==== Press `Enter` to write F ====")
    #raw_input() #is one way you can test your patterns respectively
    aubo_move.write_F(scale)
    print("==== Press `Enter` to write U ====")
    #raw_input()
    aubo_move.goto_next(scale)
    aubo_move.write_U(scale)
    print("==== Press `Enter` to write D ====")
    #raw_input()
    aubo_move.goto_next(scale)
    aubo_move.write_D(scale)
    print("==== Press `Enter` to write A ====")
    #raw_input()
    aubo_move.goto_next(scale)
    aubo_move.write_A(scale)
    print("==== Press `Enter` to write N ====")
    #raw_input()
    aubo_move.goto_next(scale)
    aubo_move.write_N(scale)

    # and D, A & N
    aubo_move.go_to_ready_pose()