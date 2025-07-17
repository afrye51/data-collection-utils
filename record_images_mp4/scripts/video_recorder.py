#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import datetime
import os
import time

class VideoRecorder:
    def __init__(self):
        self.camera_topic = rospy.get_param('~camera_topic', None)
        self.camera_name = rospy.get_param('~camera_name', None)
        self.scenario_name = rospy.get_param('~scenario_filename', None)
        self.timeout_sec = rospy.get_param('~timeout_sec', 5.0)
        if not self.camera_name:
            rospy.logwarn('missing camera_name')
        if not self.camera_topic:
            rospy.logwarn('missing camera_topic')
        if not self.scenario_name:
            rospy.logwarn('missing scenario_name')

        if not all([self.camera_topic, self.camera_name, self.scenario_name]):
            rospy.logerr("Missing one or more required parameters: camera_topic, camera_name, scenario_filename")
            rospy.signal_shutdown("Missing parameters")
            return

        self.bridge = CvBridge()

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        output_dir = os.path.join('/gwmp_data', self.scenario_name)
        # os.makedirs(output_dir, exist_ok=True)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        self.video_writer = cv2.VideoWriter(
            os.path.join(output_dir, self.camera_name + '_video.avi'),
            fourcc,
            3.0,
            (1024, 768)
        )
        self.timing_file = open(os.path.join(output_dir, self.camera_name + '_frame-timing.csv'), 'w')

        self.i = 0
        self.last_image_time = time.time()
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_timeout)
        self.subscriber = rospy.Subscriber(self.camera_topic, Image, self.listener_callback, queue_size=10)

        rospy.on_shutdown(self.on_shutdown)

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: " + str(e))
            return

        # self.timing_file.write(str(self.i) + str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S_%f")) + '\n')
        self.timing_file.write('%05d,%s\n' % (self.i, datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S_%f")))
        if self.i == 0:
            rospy.loginfo(self.camera_name + ' recording started')

        self.i += 1
        self.last_image_time = time.time()
        self.video_writer.write(cv_image)

    def check_timeout(self, event):
        elapsed = time.time() - self.last_image_time
        if elapsed > self.timeout_sec:
            rospy.logwarn("No image received in the last %.1f seconds!" % (elapsed))

    def on_shutdown(self):
        if self.video_writer is not None and self.video_writer.isOpened():
            self.video_writer.release()
            rospy.loginfo("Recording stopped and video saved.")
        if self.timing_file:
            self.timing_file.close()

def main():
    rospy.init_node('video_recorder', anonymous=False)
    recorder = VideoRecorder()
    rospy.spin()

if __name__ == '__main__':
    main()
