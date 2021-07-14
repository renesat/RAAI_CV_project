#!/usr/bin/env python3

import rospy
import cv_bridge
import cv2
import random
from sensor_msgs.msg import Image

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../pytracking")
from pytracking.evaluation import Tracker
from pytracking.evaluation.multi_object_wrapper import MultiObjectWrapper
from video_msgs.msg import OptionalBox, InitFrame, VideoFrame
from collections import OrderedDict

class TrackerNode:

    def __init__(self):

        rospy.init_node('tracker')

        self._tracker_name = rospy.get_param('~tracker_name')
        self._tracker_param = rospy.get_param('~tracker_param')
	
        self._sub = rospy.Subscriber('/show_video/state', VideoFrame, self.on_image, queue_size=10)
        self._pub = rospy.Publisher('/tracker/img_with_rectangle', Image, queue_size=10)

        self._br = cv_bridge.CvBridge()
        
        self._cur_id = None
        self._init_state = None
        self._tracker = Tracker(self._tracker_name, self._tracker_param)
        self._tracker_fun = self.init_tracker()
        rospy.loginfo("INIT")
       
        
    def init_tracker(self):
        params = self._tracker.get_parameters()
        params.debug = 0
        params.output_not_found_box = True
        params.tracker_name = self._tracker.name
        params.param_name = self._tracker.parameter_name
        self._tracker._init_visdom(None, 0)

        multiobj_mode = getattr(params, 'multiobj_mode', getattr(self._tracker.tracker_class, 'multiobj_mode', 'default'))

        if multiobj_mode == 'default':
            tracker = self._tracker.create_tracker(params)
            if hasattr(tracker, 'initialize_features'):
                tracker.initialize_features() 
        elif multiobj_mode == 'parallel':
            tracker = MultiObjectWrapper(self._tracker.tracker_class, params, self._tracker.visdom, fast_load=True)
        else:
            raise ValueError('Unknown multi object mode {}'.format(multiobj_mode))
        return tracker

    def __build_init_info(self, box):
        return {
            'init_bbox': OrderedDict({1: box}),
            'init_object_ids': [1, ], 
            'object_ids': [1, ],
            'sequence_object_ids': [1, ]
        }
        
    def on_image(self, state : VideoFrame):
    
        if state.id != self._cur_id:
           self._cur_id = state.id
           self._init_state = state.init_frame
           self._tracker_fun.initialize(
               self._br.imgmsg_to_cv2(
                   self._init_state.init_frame
               ),
               self.__build_init_info([
                   self._init_state.box.x,
                   self._init_state.box.y,
                   self._init_state.box.w,
                   self._init_state.box.h
               ])
           )
    	   
        image = self._br.imgmsg_to_cv2(state.frame)
        
        # track
        image_disp = image.copy()
        out = self._tracker_fun.track(image)
        state = [int(s) for s in out['target_bbox'][1]]
        
        
        
        # Draw box
        if state != [-1, -1, -1, -1]:
            cv2.rectangle(image_disp, (state[0], state[1]), (state[2] + state[0], state[3] + state[1]), (0, 255, 0), 5)
                         
        # Publish box
        image_msg = self._br.cv2_to_imgmsg(image_disp)
        self._pub.publish(image_msg)


    def spin(self):
        rospy.spin()


def main():
    node = TrackerNode()
    node.spin()


if __name__ == '__main__':
    main()
