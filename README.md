The robot Simulation in Gazebo of ros.


# Building
1. `catkin_make`
2. `source devel/setup.bash`


# Running
1. 打开 master 节点
    `roscore &`
2. 打开 gazebo    
    `roslaunch robot_gazebo gazebo.launch gui:=true`
3. 打开 moveit    
    `roslaunch robot_moveit_config robot_moveit_planning_execution.launch`
4. 打开 rviz      
    `roslaunch robot_moveit_config moveit_rviz.launch`
5. 测试机械臂移动   
    `rosrun robot_control robot_test`





