
Instructions to operate the UR5
=================================

Turn on the robot
------------------
1. Press the Power button (green one)
2. Appear window saying "Cannot proceed". Press button "Initialization screen"
3. Press button "On"
4. After a little while, press "Start" (press hard)
5. The robot will groan a little bit. Then press "OK" (lower right corner)
6. A screen appear with options, which you don't need to select ( Run program. program robot setup robot )

Just for fun: Ping the arm: 192.168.1.108

Bringup the robot with some limit [-PI, PI]
--------------------------------------------

roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.108 WORKED AND WAS ABLE TO SEE /joint_states

roslaunch ur_bringup ur5_bringup_joint_limited.launch robot_ip:=192.168.1.108

Moveit stuff (no need IP)
-------------------------

roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true

Moveit rviz
-------------
roslaunch ur5_moveit_config moveit_rviz.launch config:=true

PS.- Hand (I COULDN'T INSTALL SOEM FOR ROBOTIQ, so...)


Simulation
------------
roscore
roslaunch ur_gazebo ur5.launch

