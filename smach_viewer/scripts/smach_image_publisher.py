#!/usr/bin/env python

import copy
import cv2
import cv_bridge
import os
import rospy
import subprocess
import sys

from sensor_msgs.msg import Image

from smach_viewer.smach_viewer_base import SmachViewerBase

if sys.version_info[0] >= 3:
    unicode = str


class SmachImagePublisher(SmachViewerBase):

    def __init__(self):
        super(SmachImagePublisher, self).__init__()
        self.bridge = cv_bridge.CvBridge()
        duration = rospy.get_param('~duration', 0.1)
        self._timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        self._pub = rospy.Publisher('~image', Image, queue_size=1)

    def _timer_cb(self, event):
        with self._update_cond:
            dotcode = copy.copy(self.dotstr)
        if sys.version_info[0] < 3 and isinstance(dotcode, unicode):
            dotcode = dotcode.encode('utf8')
        filepath = '/tmp/smach_image_publisher.png'
        p = subprocess.Popen(
            ['dot', '-Tpng', '-o{}'.format(filepath)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=False,
            universal_newlines=True
        )
        _, error = p.communicate(dotcode)
        if p.returncode != 0:
            rospy.logerr("ERROR PARSING DOT CODE {}".format(error))
            return False

        if not os.path.exists(filepath):
            return
        img = cv2.imread(filepath)
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self._pub.publish(img_msg)


if __name__ == '__main__':
    rospy.init_node('smach_image_publisher')
    app = SmachImagePublisher()
    def signal_handler():
        rospy.logwarn('Killing threads...')
        app.kill()
    rospy.on_shutdown(signal_handler)
    rospy.spin()

