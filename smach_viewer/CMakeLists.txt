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
catkin_install_python(
  PROGRAMS test/test_smach.py
           test/test_rosout_error.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_smach_py.test)
  add_rostest(test/test_smach_image.test)
  find_package(roseus QUIET)
  if(roseus_FOUND)
    add_rostest(test/test_smach_eus.test)
  endif()
endif()
