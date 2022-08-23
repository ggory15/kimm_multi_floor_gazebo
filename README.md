# KIMM_MULTI_FLOOR_GAZEBO

## Prerequisite
1) Ubuntu 22.04
2) ROS2 Humble
3) sudo apt install ros-humble-nav2* ros-humble-turtlebot3*

## How to Use This
1) First Terminal (Gazebo)

``` ros2 launch kimm_multi_floor_gazebo tb3_gazebo_stage1.py headless:=False ```

2) Second Terminal (Rviz)

``` ros2 launch kimm_multi_floor_gazebo multi_floor_navigation_launch.py ```

3) Third Terminal (State Machine)

In scripts folder,

``` python3 navigator_sample.py ```
