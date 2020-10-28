#!/usr/bin/python

import sys, rospy, tf, moveit_commander, math, moveit_msgs.msg
import tf2_ros, tf2_geometry_msgs
import numpy as np

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
        print "============ Robot Groups:", robot.get_group_names()
        print "============ Printing robot state"
        print robot.get_current_state()

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

        # define ready pose with specific joint value
        default_joint_states[0] = 2.3938   / 180 * math.pi
        default_joint_states[1] = -17.7986 / 180 * math.pi
        default_joint_states[2] = -95.4710 / 180 * math.pi
        default_joint_states[3] = -5.4654  / 180 * math.pi
        default_joint_states[4] = -94.8020 / 180 * math.pi
        default_joint_states[5] = 1.7620   / 180 * math.pi
        
        group.go(default_joint_states, wait=True)
        # group.stop()
        rospy.sleep(1)
        current_joints = group.get_current_joint_values()
        current_pose = self.group.get_current_pose().pose
        print("current pose:", current_pose)
        return all_close(default_joint_states, current_joints, 0.01)

    def go_to_pre_pointA(self):
        group = self.group
        current_pose = group.get_current_pose().pose
        print("Current pose: ", current_pose)
        pose_goal=geometry_msgs.msg.Pose()

        # define pose with specific parameters
        pose_goal.position.x = -0.249004
        pose_goal.position.y = -0.292347
        pose_goal.position.z = 0.697158
        pose_goal.orientation.x = -0.84184
        pose_goal.orientation.y = -0.53862
        pose_goal.orientation.z = 0.002209
        pose_goal.orientation.w = 0.008829
        
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        # group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)
        return all_close(pose_goal, current_pose, 0.01)
        
	# Point A is 0.2m lower than Pre_Point A 
    def go_to_pointA(self):



    def init_2f_gripper(self):
        # rospy.init_node('Robotiq2FGripper')
        self.gripper_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output();
        # gripper reset
        self.gripper_command.rACT = 0
        # gripper activate
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output();
        self.gripper_command.rACT = 1
        self.gripper_command.rGTO = 1
        self.gripper_command.rSP  = 255
        self.gripper_command.rFR  = 150
        rospy.sleep(0.1)

    def gripper_open(self):
        self.gripper_command.rPR = 0
        self.gripper_pub.publish(self.gripper_command)
        rospy.sleep(0.1)
	
	# The corresponding highest value is 255
    def gripper_close(self):


if __name__=="__main__":

    aubo_move = aubo_vision_pick()
    
    print "==== Press `Enter` to go to ready pose ===="
    raw_input()
    aubo_move.go_to_ready_pose()
    aubo_move.init_2f_gripper()

    print "==== Press `Enter` to go to position A ===="
    raw_input()
    aubo_move.go_to_pre_pointA()
    aubo_move.go_to_pointA()
    aubo_move.gripper_close()
    aubo_move.go_to_pre_pointA()

	#wait for subsequent commands
    rospy.sleep(3)

	#move to the original position
    aubo_move.go_to_ready_pose()
