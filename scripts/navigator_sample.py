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

stage1_map = os.path.join(pkg_share, map_file_path_stage1)

## if you want to set initial pose, you can use topic publisher,
# initpose_publisher_ = InitialPosePublisher() # Initial Pose Publisher for Nav2
# initPose = PoseWithCovarianceStamped()
# initPose.header.stamp = Clock().now().to_msg()

# initPose.header.frame_id = "map"
# initPose.pose.pose.position.x = -2.
# initPose.pose.pose.position.y = -0.5
# initPose.pose.pose.position.z = 0.0

# initPose.pose.pose.orientation.x = 0.0
# initPose.pose.pose.orientation.y = 0.0
# initPose.pose.pose.orientation.z = 0.0
# initPose.pose.pose.orientation.w = 1.0
# initpose_publisher_.publisher_.publish(initPose)
# print("Initial Pose at State 1 is Published")
# time.sleep(3.0)

#Otherwise, you can use navigator function
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = navigator.get_clock().now().to_msg()
initial_pose.pose.position.x = -2.0
initial_pose.pose.position.y = -0.5
initial_pose.pose.position.z = 0.
initial_pose.pose.orientation.x = 0.0
initial_pose.pose.orientation.y = 0.0
initial_pose.pose.orientation.z = 0.0
initial_pose.pose.orientation.w = 1.0
navigator.setInitialPose(initial_pose)

# Wait for navigation to fully activate. Use this line if autostart is set to true.
navigator.waitUntilNav2Active()

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

def waypoints(goals):
    goals_pose = []
    
    for i in range(len(goals)):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()

        goal_pose.pose.position.x = float(goals[i][0])
        goal_pose.pose.position.y = goals[i][1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = goals[i][2]
        goal_pose.pose.orientation.y = goals[i][3]
        goal_pose.pose.orientation.z = goals[i][4]
        goal_pose.pose.orientation.w = goals[i][5]

        goals_pose.append(goal_pose)
    
    print(goals_pose)
    return goals_pose


def main():
    stage1_goals = ["stage1_mid_point", "stage1_goal"]
    stage1_goals_coordinates = [[0.5, 0.5, 0., 0., 0., 1.], [2., 1.0, 0., 0., 1., 0.]]
    # Every Coord is repreasented as x_pos, y_pos, x_quat, y_quat, z_quat, w_quat 

    i = 0
    have_task = 0

    done = False 

    stage = 1
    current_goal = stage1_goals[0]
    print('[Robot]   Okay, I heard I have a task')

    ## There are two way to generate trajectory, one is a hard coding 
    initialize_flag = False
    while not done:
        if have_task == 0: 
            stage = 1
            current_goal = stage1_goals[0]
            navigator.clearAllCostmaps()
            navigator.changeMap(stage1_map)
            isDone = moveTo(stage1_goals_coordinates[0], current_goal)

            if isDone:
                have_task = 1
                current_goal = stage1_goals[1]
                isDone2= moveTo(stage1_goals_coordinates[1], current_goal)

            if isDone2:
                print ("[Robot]   Evrything is Done")
                done = isDone2

    ## The other is follow waypoint method.
    delete_model_client = DeleteEntityAsync()
    response = delete_model_client.send_request("turtlebot3_waffle") 
    # delete model for next stage
    from subprocess import call
    call(["ros2", "launch", "kimm_multi_floor_gazebo", "gz_respawner_stage1.py"])
    print ("[Robot] Return to Previous Stage")
    
    navigator.clearAllCostmaps()
    navigator.changeMap(stage1_map)

    time.sleep(3)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -2.0
    initial_pose.pose.position.y = -0.5
    initial_pose.pose.position.z = 0.
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)


    nav_start = navigator.get_clock().now()
    stage1_waypoints = waypoints(stage1_goals_coordinates)
    navigator.followWaypoints(stage1_waypoints)

    ## Stage1
    i = 0
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        i = i + 1
        if feedback and i%5 == 0:
            print('Executing current waypoint: ' + stage1_goals[feedback.current_waypoint])

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection is complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection was canceled. Returning to start...')
        exit(1)
    elif result == TaskResult.FAILED:
        print('Inspection failed! Returning to start...') 

    # Shut down the ROS 2 Navigation Stack
    navigator.lifecycleShutdown()
    exit(0) 

if __name__ == '__main__':
  main()
