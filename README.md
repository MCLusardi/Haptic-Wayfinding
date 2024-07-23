# Haptic-Wayfinding
Commands to test the robot's navigation path-finding: 
Path for Map: /home/hcal-group/stretch_user/maps
Map Name: navigation_test_map1.yaml corresponds to southeast corner of pavilion
there are other maps in stretch_user/maps, can try those too
Launch map in rviz: roslaunch stretch_navigation navigation.launch map_yaml:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml, 
Note the command above launches the stretch_driver, make sure robot is homed first by running stretch_robot_home.py in terminal
remember after launch, set the 2D Pose Estimate to robot's current pose before teloperation or running of navigation script
roslaunch stretch_navigation tag_location.launch to publish the current pose and save or delete poses, 
rostopic echo /tag_location to get stream of current poses being published to topic, 
python3 navigation.py in stretch_tutorials/src to navigate to a 2D Nav Goal which is set to the landmark want to go(pose is saved as quaternion) 
and 1. rosservice call /switch_to_navigation_mode
2. rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=stretch/cmd_vel to teleoperate robot by sending twist messages from keyboard
Note: For all ROS commands to work properly, must run source devel/setup.bash from catkin_ws inside Haptic_WayFinding