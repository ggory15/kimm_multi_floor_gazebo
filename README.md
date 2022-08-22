#KIMM_MULTI_FLOOR_GAZEBO

1) First Terminal (Gazebo)

``` ros2 launch kimm_multi_floor_gazebo tb3_gazebo_stage1.py headless:=False ```

2) Second Terminal (Rviz)

``` ros2 launch kimm_multi_floor_gazebo multi_floor_navigation_launch.py ```

3) Third Terminal (State Machine)

``` ros2 launch kimm_multi_floor_gazebo navigation_sample.py ```
