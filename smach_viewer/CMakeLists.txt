cmake_minimum_required(VERSION 2.8.3)

project(smach_viewer)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS smach_ros smach_msgs
  )

# Install targets
catkin_install_python(
  PROGRAMS scripts/smach_viewer.py
           scripts/smach_image_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
