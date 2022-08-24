#! /usr/bin/env python3

import time  # Time library
import os
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node
from rclpy.clock import Clock

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module
from launch_ros.substitutions import FindPackageShare
from gazebo_msgs.srv import DeleteEntity, SpawnEntity

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

class DeleteEntityAsync(Node):
    def __init__(self):
        super().__init__('delete_entity_async')
        self.cli = self.create_client(DeleteEntity, '/delete_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DeleteEntity.Request()

    def send_request(self, name):
        self.req.name = name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

# class SpawnEntityAsync(Node):
#     def __init__(self):
#         super().__init__('add_entity_async')
#         self.cli = self.create_client(SpawnEntity, '/spawn_entity')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.req = SpawnEntity.Request()

#     def send_request(self, name):
#         self.req.name = "turtlebot3_waffle"
#         self.req.xml = ""
#         self.req.robot_namespace = ""
#         self.req.initial_pose = ""
#         self.req.reference_frame = ""

#         self.future = self.cli.call_async(self.req)
#         rclpy.spin_until_future_complete(self, self.future)
#         return self.future.result()


rclpy.init()
package_name = 'kimm_multi_floor_gazebo'
pkg_share = FindPackageShare(package=package_name).find(package_name)
# Launch the ROS 2 Navigation Stack
navigator = BasicNavigator()
map_file_path_stage1 = 'maps/stage1.yaml'
map_file_path_stage2 = 'maps/stage2.yaml'

stage1_map = os.path.join(pkg_share, map_file_path_stage1)
stage2_map = os.path.join(pkg_share, map_file_path_stage2)

initpose_publisher_ = InitialPosePublisher() # Initial Pose Publisher for Nav2

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

initpose_publisher_.publisher_.publish(initPose)
print("Initial Pose at State 1 is Published")

time.sleep(3.0)

def moveTo(goal, name):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = float(goal[0])
    goal_pose.pose.position.y = goal[1]
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = goal[2]
    goal_pose.pose.orientation.y = goal[3]
    goal_pose.pose.orientation.z = goal[4]
    goal_pose.pose.orientation.w = goal[5]

    done = False
    print('[Robot]   Going to ' + name)
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():  
      feedback = navigator.getFeedback()
      if feedback:
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
          navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('[Robot]   Arrived to ' + name)
        done = True
    elif result == TaskResult.FAILED:
        print('[Robot]   Could not get to ' + name)
    else:
        print('Goal has an invalid return status!')
        
    return done

def main():
    goals = ["stage1_mid_point", "stage1_goal", "stage2_mid_point", "stage2_goal"]
    goals_coordinates = [[0.5, 0.5, 0., 0., 0., 1.], [2., 1.0, 0., 0., 1., 0.], [-0.5, 5.5, 0., 0., 0., -1.], [-2., 4., 0., 0., 0., 1., 0.]]
    # Every Coord is repreasented as x_pos, y_pos, x_quat, y_quat, z_quat, w_quat 

    i = 0
    have_task = 0
    package1_picked = 0
    package1_delivered = 0

    to_elevator = 0
    done = False 

    stage = 1
    current_goal = goals[0]
    print('[Robot]   Okay, I heard I have a task')

    while not done:
        if have_task == 0: # Do not have task, go to Office worker to get one
            stage = 1
            current_goal = goals[0]
            navigator.clearAllCostmaps()
            navigator.changeMap(stage1_map)
            isDone = moveTo(goals_coordinates[0], current_goal)

            if isDone:
                have_task = 1
                moveTo(goals_coordinates[1], current_goal)
                stage = 1
            else:
                print("[Robot]   Trying again")

        elif have_task == 1:
            delete_model_client = DeleteEntityAsync()
            response = delete_model_client.send_request("turtlebot3_waffle") 
            # delete model for next stage
            from subprocess import call
            call(["ros2", "launch", "kimm_multi_floor_gazebo", "gz_respawner.py"])
            print ("[Robot] Move to Next Stage")

            initPose = PoseWithCovarianceStamped()
            initPose.header.stamp = Clock().now().to_msg()

            initPose.header.frame_id = "map"
            initPose.pose.pose.position.x = 2.
            initPose.pose.pose.position.y = 6.
            initPose.pose.pose.position.z = 0.0

            initPose.pose.pose.orientation.x = 0.0
            initPose.pose.pose.orientation.y = 0.0
            initPose.pose.pose.orientation.z = 1.0
            initPose.pose.pose.orientation.w = 0.0

            initpose_publisher_.publisher_.publish(initPose)

            time.sleep(3.0)

            navigator.clearAllCostmaps()
            navigator.changeMap(stage2_map)
            isDone = moveTo(goals_coordinates[2], current_goal)
            stage = 2
            if isDone:
                moveTo(goals_coordinates[3], current_goal)
                print('[Robot]   I have a package, need to go to deliver it...')
                have_task = 2
        elif have_task == 2:
            print('[Robot]   Everythins is Okay for me')
            done = True

    # Shut down the ROS 2 Navigation Stack
    navigator.lifecycleShutdown()
    exit(0) 

if __name__ == '__main__':
  main()
