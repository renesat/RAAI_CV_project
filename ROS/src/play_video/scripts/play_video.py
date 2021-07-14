#!/usr/bin/python3

import rospy
import cv_bridge
import cv2
import uuid

from sensor_msgs.msg import Image
from video_msgs.msg import OptionalBox, InitFrame, VideoFrame



class PlayVideoNode:

    def __init__(self):
        rospy.init_node('play_video')

        self._video_path = rospy.get_param('~video_path')
        self._optional_box = [int(x) for x in (rospy.get_param('~optional_box')).split(' ')]
        
        rospy.loginfo('video_path = %s', self._video_path)

        self._pub = rospy.Publisher('/show_video/state', VideoFrame, queue_size=10)
        self._image_pub = rospy.Publisher('/show_video/Image', Image, queue_size=10)
        self._br = cv_bridge.CvBridge()
        
        self._init_frame = None
        self._init_state = None
        self._capture = None
        self._id = None
        
    
    def init_video(self):
        self._capture = cv2.VideoCapture(self._video_path)
        if self._capture.isOpened():
            ret, frame = self._capture.read()
            if ret:
                self._init_frame = self._br.cv2_to_imgmsg(frame)
                self._id = str(uuid.uuid4())
        else:
            rospy.loginfo("Video not open")
            exit(1)
        self._init_frame_msg = self.create_init_state()

    def create_init_state(self):
        state = InitFrame()
        state.init_frame = self._init_frame
        state.box.x = self._optional_box[0]
        state.box.y = self._optional_box[1]
        state.box.w = self._optional_box[2]
        state.box.h = self._optional_box[3]
        return state        
    
    def create_video_frame(self, image_msg):
        videoFrame = VideoFrame()
        videoFrame.id = self._id
        videoFrame.frame = image_msg
        videoFrame.init_frame = self._init_frame_msg
        return videoFrame
        
    def spin(self):
        
        self.init_video()
        fps = self._capture.get(cv2.CAP_PROP_FPS)
        rospy.loginfo(fps)
        rate = rospy.Rate(fps) # 10hz
        while not rospy.is_shutdown() and self._capture.isOpened():
            ret, frame = self._capture.read()         
            if ret:
              image_msg = self._br.cv2_to_imgmsg(frame)
              
              videoFrame = self.create_video_frame(image_msg)
              
              self._pub.publish(videoFrame)
              self._image_pub.publish(image_msg)
            else:
              break
            rate.sleep()
           

def main():
    node = PlayVideoNode()
    node.spin()


if __name__ == '__main__':
    main()


