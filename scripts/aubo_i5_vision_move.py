#!/usr/bin/python

import sys, rospy, tf, moveit_commander, math, moveit_msgs.msg
import tf2_ros, tf2_geometry_msgs
import numpy as np
import copy

from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseStampedNamed
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


obj_pose_topic = "/object_pose"

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
        group.set_max_acceleration_scaling_factor(0.5)
        group.set_max_velocity_scaling_factor(0.5)

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.track_flag = False
        self.default_pose_flag = True
        self.grasp_ongoing = False
        
        self.target_point = PointStamped()
        # Subscribe to recognition results
        
        self.pose_sub = rospy.Subscriber(obj_pose_topic, PoseStampedNamed, self.obj_pose_callback, queue_size = 1)
        # queue_size = 1, no remaining task for the sake of safety
        self.tf_listener = tf.TransformListener()
        rate = rospy.Rate(10)
        rospy.spin()

        # self.go_to_ready_pose()

    def obj_pose_callback(self, msg):
        # geometry_msg pose from vision part
        obj_pose_stamped = copy.deepcopy(msg.stamped_pose)
        label = msg.label
        if (self.grasp_ongoing == False):
            self.obj_pose_camera = copy.deepcopy(obj_pose_stamped) # PoseStamped
        else:
            return
        print ("============ Pose Receiving ===========")
        print ("receiving pose: ", self.obj_pose_camera)
        self.grasp_ongoing = True
        
        # pose converting
        print ("============ Coordinate Converting ===========")
        # raw_input()
        rospy.sleep(0.5)
        # self.go_to_ready_pose()
        self.init_2f_gripper()
        self.coordinate_convert()
        # go to pre-grasp pose
        print ("============ Go to pre-grasp pose ===========")
        # raw_input()
        rospy.sleep(0.5)
        self.go_to_pre_grasp_pose()
        # self.go_to_ready_pose()
        # go to grasp pose
        print ("============== Go to grasp pose =============")
        raw_input()
        rospy.sleep(0.5)
        self.go_to_grasp_pose()
        self.gripper_close()
        self.go_to_exit_grasp_pose()
        # self.go_to_ready_pose()
        print ("============== Go to place pose =============")
        # raw_input()
        # rospy.sleep(0.5)
        if (label == "apple"):
            self.go_to_place_pose_A()
        if (label == "orange"):
            self.go_to_place_pose_B()
        self.gripper_open()
        self.go_to_ready_pose()

        rospy.sleep(3.3)
        self.grasp_ongoing = False
        # Place
        

    
    def coordinate_convert(self):
        # convert object pose from rgb_frame to robot_frame
        # if listener.frameExists("base_link") and listener.frameExists("camera_rgb_link"):
        #     # 'base_link' or 'world' depends on planning frame the system is using
        #     listener.waitForTransform("base_link", "camera_rgb_link",
        #                               rospy.Time(0), rospy.Duration(4.0))
        self.obj_pose_robot =self.tf_listener.transformPose("base_link", self.obj_pose_camera)
        # self.obj_pose_robot.pose.position.y += 0.003
        self.obj_pose_robot.pose.position.x += 0.007
        self.obj_pose_robot.pose.orientation.x = 0.707106781
        self.obj_pose_robot.pose.orientation.y = -0.707106781
        self.obj_pose_robot.pose.orientation.z = -4.32978028e-17
        self.obj_pose_robot.pose.orientation.w = 4.32978028e-17
        self.obj_pose_robot.pose.position.z += 0.68


        self.pose_refreshed = True
        print ("position in robot frame:")
        print (self.obj_pose_robot)

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
    
    def go_to_pre_grasp_pose(self):
        if (self.pose_refreshed == False):
            print ("pose not ready")
            return

        group = self.group
        # current_pose = group.get_current_pose().pose
        # print("Current pose: ", current_pose)
        # define pose with specific parameters

        pose_goal = copy.deepcopy(self.obj_pose_robot.pose)
        pose_goal.position.z += 0.1
        print("Pre-grasp pose: ", pose_goal)

        group.set_pose_target(pose_goal)
        plan = group.plan()
        group.go(wait=True)

        # group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)

        # self.pose_refreshed = False
        return all_close(pose_goal, current_pose, 0.01)
    
    def go_to_exit_grasp_pose(self):
        if (self.pose_refreshed == False):
            print ("pose not ready")
            return

        group = self.group
        # current_pose = group.get_current_pose().pose
        # print("Current pose: ", current_pose)
        # define pose with specific parameters

        pose_goal = copy.deepcopy(self.obj_pose_robot.pose) # ATTENTION: mind if obj_pose_robot changes during grasping
        pose_goal.position.z += 0.3

        group.set_pose_target(pose_goal)
        plan = group.plan()
        group.go(wait=True)

        # group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)

        self.pose_refreshed = False
        return all_close(pose_goal, current_pose, 0.01)

    def go_to_grasp_pose(self):
        if (self.pose_refreshed == False):
            print ("pose not ready")
            return

        group = self.group
        # current_pose = group.get_current_pose().pose
        # print("Current pose: ", current_pose)
        # define pose with specific parameters

        pose_goal = copy.deepcopy(self.obj_pose_robot.pose)
        print("Grasp pose: ", pose_goal)

        group.set_pose_target(pose_goal)
        plan = group.plan()
        group.go(wait=True)

        # group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)

        # self.pose_refreshed = False
        return all_close(pose_goal, current_pose, 0.01)
        
    def go_to_place_pose_A(self):
        # if (self.pose_refreshed == False):
        #     print ("pose not ready")
        #     return
        print("Goto Apple")
        group = self.group
        # current_pose = group.get_current_pose().pose
        # print("Current pose: ", current_pose)
        # define pose with specific parameters

        pose_goal=geometry_msgs.msg.Pose()
        # pose_goal.position.x = -0.156
        # pose_goal.position.y = 0.319
        # pose_goal.position.z = 0.6
        pose_goal.position.x = -0.48
        pose_goal.position.y = -0.531
        pose_goal.position.z = 0.46
        pose_goal.orientation.x = 0.707106781
        pose_goal.orientation.y = -0.707106781
        pose_goal.orientation.z = -4.32978028e-17
        pose_goal.orientation.w = 4.32978028e-17
        print("Place pose: ", pose_goal)

        group.set_pose_target(pose_goal)
        plan = group.plan()
        group.go(wait=True)

        # group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)

        # self.pose_refreshed = False
        return all_close(pose_goal, current_pose, 0.01)

        # self.go_to_ready_pose()
    
    def go_to_place_pose_B(self):
        # if (self.pose_refreshed == False):
        #     print ("pose not ready")
        #     return
        print("Goto Orange")
        group = self.group

        pose_goal=geometry_msgs.msg.Pose()
        pose_goal.position.x = -0.69
        pose_goal.position.y = -0.531
        pose_goal.position.z = 0.46
        pose_goal.orientation.x = 0.707106781
        pose_goal.orientation.y = -0.707106781
        pose_goal.orientation.z = -4.32978028e-17
        pose_goal.orientation.w = 4.32978028e-17
        print("Place pose: ", pose_goal)

        group.set_pose_target(pose_goal)
        plan = group.plan()
        group.go(wait=True)

        # group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)

        # self.pose_refreshed = False
        return all_close(pose_goal, current_pose, 0.01)

        # self.go_to_ready_pose()

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
        self.gripper_pub.publish(self.gripper_command)
        rospy.sleep(0.1)

    def gripper_open(self):
        self.gripper_command.rPR = 0
        self.gripper_pub.publish(self.gripper_command)
        rospy.sleep(0.1)

    def gripper_close(self):
        self.gripper_command.rPR = 255
        # self.gripper_command.rFR = 50
        self.gripper_pub.publish(self.gripper_command)
        rospy.sleep(0.1)
    
    def if_grasp_check(self):
        if (self.gripper.command.rPR == 255):
            print("Grasping failed")

if __name__=="__main__":

    aubo_move = aubo_vision_pick()
    aubo_move.go_to_ready_pose()
    # print "==== Press `Enter` to go to ready pose ===="
    # raw_input()
    # aubo_move.go_to_ready_pose()
    # aubo_move.init_2f_gripper()

    # print "==== Press `Enter` to go to position A ===="
    # aubo_move.go_to_grasp_pose()