# Haptic-Wayfinding
## Setup
For all ROS commands to work properly, must run source devel/setup.bash from catkin_ws inside Haptic_WayFinding in each new terminal

Also, make sure robot is homed first by running `stretch_robot_home.py` in terminal.

For any other issues, run `stretch_system_check.py` and see if any errors come up.

## Teleoperation
to teleoperate robot by sending twist messages from keyboard:
1. `rosservice call /switch_to_navigation_mode`
2. `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=stretch/cmd_vel`

## Building a map
Save all maps to `/home/hcal-group/stretch_user/maps`

### Current Maps
`navigation_test_map1.yaml` corresponds to the mobile robot arena in the northeast corner of the robot pavilion.

`demo_8_6.yaml` is a map of the east side of the robot pavilion, including the mobile robot arena of the robot pavilion.

## Autonomous Navigation
To launch a previously saved map in RViz: ```roslaunch stretch_navigation navigation.launch map_yaml:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml```

   **NOTE** the command above will also launch the stretch_driver.
   
After launch, before running any other scripts, set the 2D Pose Estimate to robot's current pose in RViz

To publish the current pose and save or delete poses: `roslaunch stretch_navigation tag_location.launch`

To get the stream of poses being published to a topic: `rostopic echo /tag_location` 

to set a saved pose as a 2D Nav Goal: `python3 navigation.py` in `stretch_tutorials/src` 

**NOTE** Pose is saved as a quaternion
