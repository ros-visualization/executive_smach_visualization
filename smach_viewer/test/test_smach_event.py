#!/usr/bin/env python

import sys
import os

parent_dir_name = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

try:
    import importlib.util
    file_path = parent_dir_name + "/scripts/smach_viewer.py"
    module_name = 'smach_viewer_frame'
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    smach_viewer_frame = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = smach_viewer_frame
    spec.loader.exec_module(smach_viewer_frame)

except:
    sys.path.insert(0, parent_dir_name + "/scripts")
    import smach_viewer as smach_viewer_frame

import wx

import rospy
import rostest
import unittest

import time

class TestSmachViewerEvent(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_smach_event', anonymous=True)
        app = wx.App()

        self.frame = smach_viewer_frame.SmachViewerFrame()
        self.frame.set_filter('dot')
        #/server_namsmach/container_statsup
        self.frame.Show()
        self.frame.is_button = type('DummyButton', (object, ), {'Disable': lambda self: None, 'Enable': lambda self: None})();
        #app.MainLoop()
        self.evtloop = wx.GUIEventLoop()
        #old = wx.EventLoop.GetActive()
        wx.EventLoop.SetActive(self.evtloop)

    def test_samch_event(self):
        try:
            for i in range(10):
                self.frame.path_input.SetValue('/SM_ROOT/FOO')
                self.frame.selection_changed(None)
                time.sleep(1)
                self.evtloop.ProcessIdle()
            rospy.loginfo("SUCCEEDED: _local_data {}".format(self.frame._containers))
            rospy.loginfo("SUCCEEDED: _local_data {}".format([v._local_data._data for v in self.frame._containers.values()]))
            self.frame.kill()
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr("FAILED: _local_data {}".format(self.frame._containers))
            rospy.logerr("FAILED: _local_data {}".format([v._local_data._data for v in self.frame._containers.values()]))
            self.assertFalse(e)

if __name__ == '__main__':
    rostest.run('smach_viewer', 'test_smach_event', TestSmachViewerEvent, sys.argv)
