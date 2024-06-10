# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "stretch_demos: 14 messages, 0 services")

set(MSG_I_FLAGS "-Istretch_demos:/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(stretch_demos_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg" "std_msgs/Header:stretch_demos/ArucoHeadScanGoal:stretch_demos/ArucoHeadScanResult:stretch_demos/ArucoHeadScanActionFeedback:stretch_demos/ArucoHeadScanActionGoal:actionlib_msgs/GoalStatus:stretch_demos/ArucoHeadScanFeedback:actionlib_msgs/GoalID:stretch_demos/ArucoHeadScanActionResult"
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg" "actionlib_msgs/GoalID:stretch_demos/ArucoHeadScanGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:stretch_demos/ArucoHeadScanResult:std_msgs/Header"
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:stretch_demos/ArucoHeadScanFeedback:std_msgs/Header"
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg" ""
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg" ""
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg" ""
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg" "stretch_demos/VisualServoActionFeedback:std_msgs/Header:stretch_demos/VisualServoActionGoal:stretch_demos/VisualServoGoal:actionlib_msgs/GoalStatus:stretch_demos/VisualServoActionResult:stretch_demos/VisualServoResult:actionlib_msgs/GoalID:stretch_demos/VisualServoFeedback"
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg" "stretch_demos/VisualServoGoal:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:stretch_demos/VisualServoResult:std_msgs/Header"
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:stretch_demos/VisualServoFeedback:std_msgs/Header"
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg" ""
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg" ""
)

get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg" NAME_WE)
add_custom_target(_stretch_demos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stretch_demos" "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg"
  "${MSG_I_FLAGS}"
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)
_generate_msg_cpp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
)

### Generating Services

### Generating Module File
_generate_module_cpp(stretch_demos
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(stretch_demos_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(stretch_demos_generate_messages stretch_demos_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_cpp _stretch_demos_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stretch_demos_gencpp)
add_dependencies(stretch_demos_gencpp stretch_demos_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stretch_demos_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg"
  "${MSG_I_FLAGS}"
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)
_generate_msg_eus(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
)

### Generating Services

### Generating Module File
_generate_module_eus(stretch_demos
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(stretch_demos_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(stretch_demos_generate_messages stretch_demos_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_eus _stretch_demos_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stretch_demos_geneus)
add_dependencies(stretch_demos_geneus stretch_demos_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stretch_demos_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg"
  "${MSG_I_FLAGS}"
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)
_generate_msg_lisp(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
)

### Generating Services

### Generating Module File
_generate_module_lisp(stretch_demos
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(stretch_demos_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(stretch_demos_generate_messages stretch_demos_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_lisp _stretch_demos_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stretch_demos_genlisp)
add_dependencies(stretch_demos_genlisp stretch_demos_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stretch_demos_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg"
  "${MSG_I_FLAGS}"
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)
_generate_msg_nodejs(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
)

### Generating Services

### Generating Module File
_generate_module_nodejs(stretch_demos
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(stretch_demos_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(stretch_demos_generate_messages stretch_demos_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_nodejs _stretch_demos_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stretch_demos_gennodejs)
add_dependencies(stretch_demos_gennodejs stretch_demos_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stretch_demos_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg"
  "${MSG_I_FLAGS}"
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)
_generate_msg_py(stretch_demos
  "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
)

### Generating Services

### Generating Module File
_generate_module_py(stretch_demos
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(stretch_demos_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(stretch_demos_generate_messages stretch_demos_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg" NAME_WE)
add_dependencies(stretch_demos_generate_messages_py _stretch_demos_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stretch_demos_genpy)
add_dependencies(stretch_demos_genpy stretch_demos_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stretch_demos_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stretch_demos
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(stretch_demos_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stretch_demos
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(stretch_demos_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stretch_demos
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(stretch_demos_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stretch_demos
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(stretch_demos_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stretch_demos
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(stretch_demos_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
