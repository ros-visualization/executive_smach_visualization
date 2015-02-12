cmake_minimum_required(VERSION 2.8.3)
project(rqt_smach)

find_package(catkin REQUIRED COMPONENTS
  qt_dotgraph
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_package(
  CATKIN_DEPENDS qt_dotgraph rospy rqt_gui rqt_gui_py
)

catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS bin/rqt_smach
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
