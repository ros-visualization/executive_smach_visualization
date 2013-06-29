cmake_minimum_required(VERSION 2.8.3)
project(smach_viewer)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()

else()

  find_package(catkin REQUIRED)

  catkin_package(
    CATKIN_DEPENDS smach_ros xdot smach_msgs
    )

  # Install targets
  install(PROGRAMS scripts/smach_viewer.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()
