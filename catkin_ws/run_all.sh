#!/bin/bash
# 一键启动Gazebo仿真、无人机起点降落、RRT路径规划、路径跟随

WORKSPACE=~/catkin_ws
source $WORKSPACE/devel/setup.bash

# 启动Gazebo仿真（第一个tab）
gnome-terminal --tab -- bash -c "source $WORKSPACE/devel/setup.bash; roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic; exec bash"

sleep 5  # 等待仿真环境加载

# 启动无人机初始点到RRT起点的控制节点（第二个tab）
gnome-terminal --tab -- bash -c "source $WORKSPACE/devel/setup.bash; rosrun rotors_evaluation go_to_start_publisher.py; exec bash"

sleep 2

# 启动RRT路径规划节点（第三个tab）
gnome-terminal --tab -- bash -c "source $WORKSPACE/devel/setup.bash; rosrun rotors_evaluation rrt_planner.py; exec bash"

sleep 2

# 启动路径跟随节点（第四个tab）
gnome-terminal --tab -- bash -c "source $WORKSPACE/devel/setup.bash; rosrun rotors_evaluation waypoint_follower.py; exec bash"

# 启动RViz（第五个tab）
gnome-terminal --tab -- bash -c "source $WORKSPACE/devel/setup.bash; rviz -d $WORKSPACE/src/rotors_simulator/rotors_description/rviz/techpod_model_view.rviz; exec bash" 