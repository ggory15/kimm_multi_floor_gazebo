import threading
import cmd, sys, os
from csuite_controller.control_suite import bcolors, ControlSuite
import numpy as np
import copy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import time
import rclpy 
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.action import ActionClient
import subprocess, signal, os, inspect, time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

class GoalposePublisher(Node):
    def __init__(self):
        super().__init__('goalpose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)

class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        self.csuite = ControlSuite()
        self.ros2 = rclpy.init()
        self.initpose_publisher_ = InitialPosePublisher()
        self.goalpose_publisher_ = GoalposePublisher()
        
        self.root_path = os.path.dirname(os.path.abspath(__file__))
        self.stage2_map = self.root_path + '/../maps/stage2.yaml'

    def do_init(self, arg):
        initPose = PoseWithCovarianceStamped()
        initPose.header.stamp = Clock().now().to_msg()

        initPose.header.frame_id = "map"
        initPose.pose.pose.position.x = -2.
        initPose.pose.pose.position.y = -0.5
        initPose.pose.pose.position.z = 0.0

        initPose.pose.pose.orientation.x = 0.0
        initPose.pose.pose.orientation.y = 0.0
        initPose.pose.pose.orientation.z = 0.0
        initPose.pose.pose.orientation.w = 1.0

        self.initpose_publisher_.publisher_.publish(initPose)
        print("Initial Pose at State 1 is Published")

    def do_stage1(self, arg):
        print("Navigate to goal at Stage #1")
        goalPose = PoseStamped()
        goalPose.header.stamp = Clock().now().to_msg()

        goalPose.header.frame_id = "map"
        goalPose.pose.position.x = 2.
        goalPose.pose.position.y = 1.0
        goalPose.pose.position.z = 0.0

        goalPose.pose.orientation.x = 0.0
        goalPose.pose.orientation.y = 0.0
        goalPose.pose.orientation.z = 1.0
        goalPose.pose.orientation.w = 0.0

        self.goalpose_publisher_.publisher_.publish(goalPose)
        print("Goal is sent")

    def do_load_map(self, arg):
        devnull = open(os.devnull, 'wb')
        print ("Load Stage#2 map")
        subprocess.Popen("ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap '{map_url: " + self.stage2_map + "}'",
                            stdout=subprocess.PIPE,
                            shell=True,
                            preexec_fn=os.setsid,
                            stderr=devnull)
        print("Initial Pose at State 2 is Published")

    def do_stage2(self, arg):
        print("Navigate to goal at Stage #2")
        goalPose = PoseStamped()
        goalPose.header.stamp = Clock().now().to_msg()

        goalPose.header.frame_id = "map"
        goalPose.pose.position.x = -2.
        goalPose.pose.position.y = -1.0
        goalPose.pose.position.z = 0.0

        goalPose.pose.orientation.x = 0.0
        goalPose.pose.orientation.y = 0.0
        goalPose.pose.orientation.z = 0.0
        goalPose.pose.orientation.w = 1.0

        self.goalpose_publisher_.publisher_.publish(goalPose)

        print("Goal is sent")


    def do_quit(self, arg):
        'Quit the control suite and its console'
        # if self.csuite.enableGZ:
        #     self.csuite.gazebo.__del__()
        # if self.csuite.enableRviz:
        #     self.csuite.rviz.__del__()
        return True



if __name__ == '__main__':
    ControlSuiteShell().cmdloop()