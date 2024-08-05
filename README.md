# Haptic-Wayfinding
## Setup
For all ROS commands to work properly, must run source devel/setup.bash from catkin_ws inside Haptic_WayFinding in each new terminal

Also, make sure robot is homed first by running `stretch_robot_home.py` in terminal.

For any other issues, run `stretch_system_check.py` and see if any errors come up.

## Untethered Operation
1. Power on robot while attached to monitor and keyboard
2. run `ip addr` to find the robots current ip address on the wifi network (it's called wlp0 something something)
3. Open TigerVNC Viewer (or the VNC of your choice) and input that ip addres you found. Log on to the hcal-group account.
4. When the remote desktop comes up, unplug the monitor hdmi cable and plug in the HDMI dummy.

Now you're set to run from remote desktop without any cables attached to the robot.

## Teleoperation
to teleoperate robot by sending twist messages from keyboard:
1. `rosservice call /switch_to_navigation_mode`
2. `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=stretch/cmd_vel`

## Building a map
**NOTE** This requires operating the robot untethered.

1. Run `roslaunch stretch_navigation mapping.launch`
2. Teleoperate the robot around the space you want to map. Make sure to revisit areas several times and avoid sharp turns
3. Run `rosrun map_server map_saver -f ${HELLO_FLEET_PATH}/maps/<map_name>` with the `<map_name>` of your choice.
   `$HELLO_FLEET_PATH` resolves to `/home/hcal-group/stretch_user`

### Current Maps
All maps can be found in `/home/hcal-group/stretch_user/maps`

`navigation_test_map1.yaml` corresponds to the mobile robot arena in the northeast corner of the robot pavilion.

`demo_8_6.yaml` is a map of the east side of the robot pavilion, including the mobile robot arena of the robot pavilion.

## Autonomous Navigation
To launch a previously saved map in RViz: 

1. Run `roslaunch stretch_navigation navigation.launch map_yaml:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml`

   **NOTE** the command above will also launch the stretch_driver.
   
2. Set the 2D Pose Estimate to robot's current pose in RViz

To save a pose: 
1. Run `roslaunch stretch_navigation tag_location.launch`
2. Teleoperate the robot to a pose you want to save
3. Hit 1 and then enter a label for that pose
   **NOTE** Pose is saved as a quaternion
   
To get the stream of poses being published to a topic: 

  `rostopic echo /tag_location` 

To set a saved pose as a 2D Nav Goal: 
1. Run `python3 navigation.py` in `stretch_tutorials/src`
2. Enter the lable of the saved pose you want to navigate to
