#!/usr/bin/env python
import sys
import unittest
import rospy

import smach
import smach_ros
import time

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        if self.counter < 3:
            time.sleep(1)
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        return 'outcome2'


## A sample python unit test
class TestSmachViewer(unittest.TestCase):

    def test_imports(self):

        import rospy
        import rospkg
        from smach_msgs.msg import SmachContainerStatus,SmachContainerInitialStatusCmd,SmachContainerStructure
        import sys
        import os
        import threading
        import pickle
        import pprint
        import copy
        try:
            from StringIO import StringIO ## for Python 2
        except ImportError:
            from io import StringIO ## for Python 3
        import colorsys
        import time
        import wx
        import wx.richtext
        import textwrap
        import base64
        # import xdot
        import smach
        import smach_ros
        import os
        import sys
        import subprocess
        import math
        import colorsys
        import time
        import re

        try:
            import PyQt4
        except:
            import PyQt5


        import wx
        import wx.lib.wxcairo as wxcairo

    # def test_smach_introspection(self):

    #     # Create a SMACH state machine
    #     sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    #     # Open the container
    #     with sm:
    #         # Add states to the container
    #         smach.StateMachine.add('FOO', Foo(),
    #                             transitions={'outcome1':'BAR',
    #                                             'outcome2':'outcome4'})
    #         smach.StateMachine.add('BAR', Bar(),
    #                             transitions={'outcome2':'FOO'})

    #     # Create and start the introspection server
    #     sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    #     sis.start()

    #     # Execute SMACH plan
    #     outcome = sm.execute()

    #     time.sleep(5)

    #     # Check that the viewer has subscribed
    #     for proxy in sis._proxies:
    #         self.assertGreater(proxy._structure_pub.get_num_connections(),0)
    #         self.assertGreater(proxy._status_pub.get_num_connections(),0)

    #     sis.stop()

if __name__ == '__main__':
    import rostest
    rospy.init_node("test_smach_viewer")
    rostest.rosrun('smach_viewer', 'test_smach_viewer', TestSmachViewer)
