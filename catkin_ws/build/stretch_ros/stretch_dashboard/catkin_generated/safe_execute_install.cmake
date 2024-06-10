execute_process(COMMAND "/home/wayfinders/Haptic-Wayfinding/catkin_ws/build/stretch_ros/stretch_dashboard/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/wayfinders/Haptic-Wayfinding/catkin_ws/build/stretch_ros/stretch_dashboard/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
