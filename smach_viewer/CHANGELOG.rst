^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package smach_viewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.1.0 (2023-02-24)
------------------
* add timestamp in published image (`#50 <https://github.com/ros-visualization/executive_smach_visualization/issues/50>`_)

  * some tools, for example jsk_rosbag_tools requries all message have time stamp

* [smach_viewer] remove smach image in /tmp when the node is killed (`#48 <https://github.com/ros-visualization/executive_smach_visualization/issues/48>`_)

  * remove smach image in /tmp when the node is killed

* [smach_viewer] add smach_image_publisher.py for headless environment (`#46 <https://github.com/ros-visualization/executive_smach_visualization/issues/46>`_)

  * add time stamp in image topics
  * add compressed image publisher
  * change file name and set constant image size
  * check if file is proper
  * change color in active states
  * import not local file
  * add smach_image_publisher.py
  * split and make SmachViewerBase, utils and text_wrapper

* Contributors: Kei Okada, Shingo Kitagawa

4.0.1 (2022-08-02)
------------------
* Fix 4.0.0, which does not work on both Melodic/Noetic (`#43 <https://github.com/ros-visualization/executive_smach_visualization/issues/43>`_)

  * use LooseVersion(wx.__version_\_) to support old Melodic (wxPython 3.0)
  * typos
  * more deprecated functions to test on melodic
  * why wouldn't this work with python3 ?
  * add missing dependency on cv_bridge
  * update use of deprecated functions
  * update pckage.xml based on https://github.com/ros-visualization/executive_smach_visualization/pull/39
  * use catkin_install_python to automatically set /usr/bin/env python3 for noetic, but need to remove smach_viewer/lib/smach_viewer from sys.path
  * xdot/wxxdot.py: intentionally uses from xdot, instead of from .xdot, because we want to use local xdot for Pytohn2 and sytem xdot for Python3
  * Revert "apply 2to3 -w -f import *" This reverts commit 4cbd2ab0d16d4123e9227bf6b9e627bd4bd8ea7f.

* Contributors: Kei Okada, Mikael Arguedas

4.0.0 (2022-07-23)
------------------
* publish smach viewer image (`#37 <https://github.com/ros-visualization/executive_smach_visualization/issues/37>`_)
* use custom TextWrapper to wrap a sentence of multibyte languages into several lines (`#40 <https://github.com/ros-visualization/executive_smach_visualization/issues/40>`_)

  - use TextWrapper for multibyte launguages
  - the TextWrapper is copied from sphinx

* Melodic/Noetic support (`#42 <https://github.com/ros-visualization/executive_smach_visualization/issues/42>`_)

  - use MyXDotParser in python3
  - check subgraph_shapes hasattr
  - Merge branch 'melodic-devel' into image-publish
  - apply 2to3 -w -f import *
  - Use GetPosition istead of GetPositionTuple
  - noetic support, based on InigoMoreno's work. see `#39 <https://github.com/ros-visualization/executive_smach_visualization/issues/39>`_

* fix unpacking the user data (`#38 <https://github.com/ros-visualization/executive_smach_visualization/issues/38>`_)
* Fix CI `#41 <https://github.com/ros-visualization/executive_smach_visualization/issues/41>`_ from k-okada/fix_ci

  - update .travis.yml
  - apply 2to3 -w -f import *
  - apply 2to3 -w -f dict *
  - apply 2to3 -w -f zip *
  - add try except and trying line width one by one

    * publish smach viewer image
* [FIX] 'Jump' object has no attribute 'url' (`#31 <https://github.com/ros-visualization/executive_smach_visualization/issues/31>`_)
  Fix for AttributeError: 'Jump' object has no attribute 'url'
  To reproduce:
  - roscore
  - rosrun smach_viewer smach_viewer.py
  - Press "+" button to increase depth and hover over/click text "Path not available"
  - Error prints in terminal
* Contributors: Gintaras, Kei Okada, Kousuke Takeuchi, Shingo Kitagawa

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
