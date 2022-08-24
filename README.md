# KIMM_MULTI_FLOOR_GAZEBO

## Prerequisite
1) Ubuntu 22.04
2) ROS2 Humble
3) sudo apt install ros-humble-nav2* ros-humble-turtlebot3*
4) Open the terminal, and then (if you use colcon workspace as $HOME/colcon_ws)

``` echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc ```

``` echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models:$HOME/colcon_ws/src/kimm_multi_floor_gazebo/worlds" >> ~/.bashrc ```

## How to Use This
1) First Terminal (Gazebo)

``` ros2 launch kimm_multi_floor_gazebo tb3_gazebo_stage1.py headless:=False ```

2) Second Terminal (Rviz)

``` ros2 launch kimm_multi_floor_gazebo multi_floor_navigation_launch.py ```

3) Third Terminal (State Machine)

In scripts folder,

``` python3 navigator_sample.py ```
