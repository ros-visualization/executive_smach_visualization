^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package smach_viewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2014-04-17)
------------------
* Removing rosbuild support and old useless test
* Fix lost information in package.xml
  That was dropped while catkinizing
* Contributors: Felix Kolbe, Jonathan Bohren

1.1.0 (2013-06-29)
------------------
* Putting cmake required version call inside the rosbuild/catkin switch
* Catkinizng (hybrid)
* Removing old, unused dependency
* Adding documentation, cleaning up some parts of the wx smach viewer
* Fix for `#5122 <https://github.com/ros-visualization/executive_smach_visualization/issues/5122>`_: Multi-line name states aren't in activate color by smach_viewer
* doc review for smach_msgs and smach_viewer
* smach viewer is doc reviewed
* add description for smach viewer
* remove reference to executive python
* smach viewer runs again
* smach viewer needs rospy
* use smach messages instead of executive python messages
* import from https://code.ros.org/svn/wg-ros-pkg/branches/jbohren/executive_smach, which is the restructured code from the executive_python stack
* Contributors: Jonathan Bohren, Wim Meeussen, wim
