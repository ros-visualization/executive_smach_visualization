#!/usr/bin/env python

import copy
import cv2
import cv_bridge
import imp
import numpy as np
import os
import rospy
import subprocess
import sys
import time

from sensor_msgs.msg import CompressedImage
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
        self.image_width = rospy.get_param('~image_width', 1000)
        self.image_height = rospy.get_param('~image_height', 1500)
        self.image_dpi = rospy.get_param('~image_dpi', 500)

        self.filepath = '/tmp/smach_image_publisher_{}.png'.format(
            os.getpid())
        self._timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        self._pub = rospy.Publisher('~image', Image, queue_size=1)
        self._pub_compressed = rospy.Publisher(
            '~image/compressed', CompressedImage, queue_size=1)

    def _timer_cb(self, event):
        with self._update_cond:
            dotcode = copy.copy(self.dotstr)
        if sys.version_info[0] < 3 and isinstance(dotcode, unicode):
            dotcode = dotcode.encode('utf8')

        w_scale = self.image_width // self.image_dpi
        h_scale = self.image_height // self.image_dpi
        p = subprocess.Popen(
            [
                'dot',
                '-Tpng',
                '-Gsize={},{}\\!'.format(w_scale, h_scale),
                '-Gdpi={}'.format(self.image_dpi),
                '-o{}'.format(self.filepath)
            ],
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

        if not os.path.exists(self.filepath):
            return
        if os.path.getsize(self.filepath) == 0:
            return

        img = cv2.imread(self.filepath)
        H, W, _ = img.shape
        if self.image_width > W or self.image_height > H:
            top_pad = (self.image_height - H) // 2
            bottom_pad = self.image_height - H - top_pad
            left_pad = (self.image_width - W) // 2
            right_pad = self.image_width - W - left_pad
            img = cv2.copyMakeBorder(
                img, top_pad, bottom_pad, left_pad, right_pad,
                cv2.BORDER_CONSTANT, value=(255, 255, 255))
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = rospy.Time.now()
        self._pub.publish(img_msg)
        compressed_img_msg = CompressedImage()
        compressed_img_msg.header = img_msg.header
        compressed_img_msg.format = img_msg.encoding
        compressed_img_msg.format += '; jpeg compressed bgr8'
        compressed_img_msg.data = np.array(
            cv2.imencode('.jpg', img)[1]).tostring()
        self._pub_compressed.publish(compressed_img_msg)

    def remove_file(self):
        # wait and loop until file is removed
        while os.path.exists(self.filepath):
            rospy.logwarn(
                'Removing file: {}'.format(self.filepath))
            os.remove(self.filepath)
            time.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('smach_image_publisher')
    app = SmachImagePublisher()

    def signal_handler():
        rospy.logwarn('Killing threads...')
        app.kill()
        app.remove_file()

    rospy.on_shutdown(signal_handler)
    rospy.spin()
