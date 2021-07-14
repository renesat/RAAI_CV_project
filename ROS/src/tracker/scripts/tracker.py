#!/usr/bin/python3

import rospy
import cv_bridge
import cv2
import random
from sensor_msgs.msg import Image


class TrackerNode:

    def __init__(self):

        rospy.init_node('tracker')

        self.sub = rospy.Subscriber('show_video', Image, self.on_image, queue_size=10)
        self.pub = rospy.Publisher('with_rectangle', Image, queue_size=10)

        self.br = cv_bridge.CvBridge()
    

    def on_image(self, image_msg : Image):
        image = self.br.imgmsg_to_cv2(image_msg)
        left = (random.randint(5, 100), random.randint(5, 100))
        right = (random.randint(100, 200), random.randint(100, 200))
        image_with_rec = cv2.rectangle(image, left, right, (0, 255, 0), 4)
        image_msg = self.br.cv2_to_imgmsg(image_with_rec)
        self.pub.publish(image_msg)


    def spin(self):
        rospy.spin()


def main():
    node = TrackerNode()
    node.spin()


if __name__ == '__main__':
    main()
