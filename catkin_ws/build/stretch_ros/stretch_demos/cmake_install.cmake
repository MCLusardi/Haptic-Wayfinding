# Install script for directory: /home/wayfinders/Haptic-Wayfinding/catkin_ws/src/stretch_ros/stretch_demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/wayfinders/Haptic-Wayfinding/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/wayfinders/Haptic-Wayfinding/catkin_ws/build/stretch_ros/stretch_demos/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stretch_demos/action" TYPE FILE FILES
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/src/stretch_ros/stretch_demos/action/ArucoHeadScan.action"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/src/stretch_ros/stretch_demos/action/VisualServo.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stretch_demos/msg" TYPE FILE FILES
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanAction.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionGoal.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionResult.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanActionFeedback.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanGoal.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanResult.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/ArucoHeadScanFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stretch_demos/msg" TYPE FILE FILES
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoAction.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionGoal.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionResult.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoActionFeedback.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoGoal.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoResult.msg"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/stretch_demos/msg/VisualServoFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stretch_demos/cmake" TYPE FILE FILES "/home/wayfinders/Haptic-Wayfinding/catkin_ws/build/stretch_ros/stretch_demos/catkin_generated/installspace/stretch_demos-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/include/stretch_demos")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/roseus/ros/stretch_demos")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/common-lisp/ros/stretch_demos")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/share/gennodejs/ros/stretch_demos")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/lib/python3/dist-packages/stretch_demos")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/lib/python3/dist-packages/stretch_demos" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/lib/python3/dist-packages/stretch_demos" FILES_MATCHING REGEX "/home/wayfinders/Haptic-Wayfinding/catkin_ws/devel/lib/python3/dist-packages/stretch_demos/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wayfinders/Haptic-Wayfinding/catkin_ws/build/stretch_ros/stretch_demos/catkin_generated/installspace/stretch_demos.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stretch_demos/cmake" TYPE FILE FILES "/home/wayfinders/Haptic-Wayfinding/catkin_ws/build/stretch_ros/stretch_demos/catkin_generated/installspace/stretch_demos-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stretch_demos/cmake" TYPE FILE FILES
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/build/stretch_ros/stretch_demos/catkin_generated/installspace/stretch_demosConfig.cmake"
    "/home/wayfinders/Haptic-Wayfinding/catkin_ws/build/stretch_ros/stretch_demos/catkin_generated/installspace/stretch_demosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stretch_demos" TYPE FILE FILES "/home/wayfinders/Haptic-Wayfinding/catkin_ws/src/stretch_ros/stretch_demos/package.xml")
endif()

