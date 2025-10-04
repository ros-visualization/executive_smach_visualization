cmake_minimum_required(VERSION 3.0.2)

project(smach_viewer)

find_package(catkin REQUIRED COMPONENTS)

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
           test/test_smach_event.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_smach_py.test)
  add_rostest(test/test_smach_image.test)
  add_rostest(test/test_smach_event.test)
  find_package(roseus QUIET)
  if(roseus_FOUND)
    add_rostest(test/test_smach_eus.test)
  endif()
  find_package(roseus_smach QUIET)
  if(roseus_smach_FOUND)
    add_rostest(test/test_roseus_smach_state_machine_ros_sample.test)
    add_rostest(test/test_smach_event.test ARGS target_python:=false target_roseus:=true)
  endif()
endif()
