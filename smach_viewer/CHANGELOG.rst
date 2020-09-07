^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package smach_viewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2020-08-25)
------------------
* Merge pull request `#30 <https://github.com/ros-visualization/executive_smach_visualization//issues/30>`_ from k-okada/add_noetic
  add noetic, remove indigo/lunar
* fix syntax for python3
* Contributors: Kei Okada

3.0.0 (2019-12-11)
------------------
* use xlabel to show better layout
* fix https://github.com/ros-visualization/executive_smach_visualization/commit/8e7dd857049695098ed2562b82811db338d0421d#diff-0594a813c7145f4e9e802a6224262e35 and https://github.com/ros-visualization/executive_smach_visualization/commit/c158e3093500c4cdd15654746cf007dba7a094e4
* add url to edges and jump, add unescape
* check if item is Url, sometimes it is Jump
* add necessary packages
* xdot 0.7 does not have url in edges
* add subgraph_shapes, which is introduced in jbohren's version of xdot
* udpate to gtk3.0
  see https://github.com/jrfonseca/xdot.py/commit/dbc0e556cffb164d65b4e56bab8bb9af7f023778#diff-044dd4123ac853930ad0086da07ef7b6L123
  the xdot.py file was copied from version 0.7 (https://raw.githubusercontent.com/jrfonseca/xdot.py/0.7/xdot.py)
* Contributors: Kei Okada

2.0.2 (2017-10-26)
------------------
* Allow launching from a launch file, use rospy.myargv() to remove ROS  remapping arguments (`#16 <https://github.com/ros-visualization/executive_smach_visualization/issues/16>`_)
  * Use rospy.myargv() to remove ROS remapping arguments  Required to allow launching from a launch file, otherwise get errors of the type:
  ```
    usage: smach_viewer.py [-h] [-f]
    smach_viewer.py: error: unrecognized arguments: __name:=smach_viewer
  ```
    * Solved 'Cannot start smach_viewer.py in launch file' problem `#17 <https://github.com/ros-visualization/executive_smach_visualization/issues/17>`_

* Contributors: Kartik Mohta

2.0.1 (2017-06-20)
------------------
* add ROS Orphaned Package Maintainers to maintainer tag (`#15 <https://github.com/ros-visualization/executive_smach_visualization/issues/15>`_)
* copy xdot from https://github.com/jbohren/xdot, (`#14 <https://github.com/ros-visualization/executive_smach_visualization/issues/14>`_)

  * support for Qt5 (Kinetic)
  * update CMakeLists.txt, package.xml, setup.py, smach_viewer.py for new xdot structure
  * add necessary lines in xdot/__init_\_.py https://github.com/jbohren/xdot/pull/14
  * copy xdot from https://github.com/jbohren/xdot, since system xdot is released in rosdep key https://github.com/ros/rosdistro/pull/4976

* add auto focus to subgraph mode button (`#11 <https://github.com/ros-visualization/executive_smach_visualization/issues/11>`_)

  * add launch option for 'auto focus to subgraph' mode as default
  * add auto focus to subgraph mode button

* feature: Add ability to save the dot graph for further processing (`#8 <https://github.com/ros-visualization/executive_smach_visualization/issues/8>`_)

  * forgot two imports
  * Add option to save dot graph to file
    Add a icon which enables the user to save the currently displayed
    graph as a .dot file in the currently hardcoded ros_home/dotfiles,
    which should normaly be $HOME/.ros/dotfiles
    From there it can be converted with the dot commandline tool into
    png, pdf or others without the problem of quality loss.

* wx viewer: checking to make sure item urls are strings to prevent crash (`#1 <https://github.com/jbohren/executive_smach_visualization/pull/1>`_)
* Contributors: Yuki Furuta, Jonathan Bohren, Kei Okada, Markus Bajones

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
