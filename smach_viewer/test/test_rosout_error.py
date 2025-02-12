#!/usr/bin/env python

import rospy
import rostest
import sys
import unittest
from rosgraph_msgs.msg import Log

class TestRosoutOutput(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_rosout_output', anonymous=True)
        self.received_error_msg = None
        rospy.Subscriber('/rosout', Log, self.callback)

    def callback(self, msg):
        print("received rosout message with level = {}".format(msg.level))
        if msg.level == Log.ERROR or msg.level == Log.FATAL:
            self.received_error_msg = msg.msg

    def test_rosout_output(self):
        rospy.wait_for_message('/rosout', Log, timeout=5)  # Wait for a message
        wait_time = float(rospy.get_param('~wait_time', 10.))
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < wait_time:
            rospy.sleep(1.0)
            self.assertIsNone(self.received_error_msg, "Error message received!") # if not equal, then error

if __name__ == '__main__':
    rostest.run('smach_viewer', 'test_rosout_output', TestRosoutOutput, sys.argv)
