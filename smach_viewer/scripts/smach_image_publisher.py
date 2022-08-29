#!/usr/bin/env python

import copy
import cv2
import cv_bridge
import imp
import os
import rospy
import subprocess
import sys

from sensor_msgs.msg import Image

try:
    # this import system (or ros-released) xdot
    # import xdot
    # need to import currnt package, but not to load this file
    # http://stackoverflow.com/questions/6031584/importing-from-builtin-library-when-module-with-same-name-exists
    def import_non_local(name, custom_name=None):
        custom_name = custom_name or name

        path = filter(lambda x: x != os.path.dirname(os.path.abspath(__file__)), sys.path)
        f, pathname, desc = imp.find_module(name, path)

        module = imp.load_module(custom_name, f, pathname, desc)
        if f:
            f.close()

        return module

    smach_viewer = import_non_local('smach_viewer')
    from smach_viewer.smach_viewer_base import SmachViewerBase
except Exception:
    # Guard against self import
    this_dir = os.path.dirname(__file__)
    # Use os.getcwd() to aovid weird symbolic link problems
    cur_dir = os.getcwd()
    os.chdir(this_dir)
    this_dir_cwd = os.getcwd()
    os.chdir(cur_dir)
    # Remove this dir from path
    sys.path = [a for a in sys.path if a not in [this_dir, this_dir_cwd]]
    # Ignore path ending with smach_viewer/lib/smach_viewer
    sys.path = [a for a in sys.path if not a.endswith('smach_viewer/lib/smach_viewer')]
    #
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
