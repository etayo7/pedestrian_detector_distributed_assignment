#!/usr/bin/env python3

#   Imports

import cv2
from cv_bridge import CvBridge
import rospy
import rosbag
import os
import genpy

#   Global

bridge = CvBridge()

#   Functions

def img_extractor(src : str):
    inBag = rosbag.Bag(src, 'r')
    outPath = 'exported'

    rospy.loginfo("Image extractor initialized!")
    
    i = 0
    topic : str
    t : genpy.Time
    for topic, msg, t in inBag.read_messages():
        if 'compressed' in topic:
            img = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        else:
            img = bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(f'{outPath}img_{i}.jpg', img)
        i += 1
        

    
    rospy.loginfo("Extraction done! Closing.")
    pass


#   Guard

if __name__ == '__main__':
    rospy.init_node("extractor", anonymous=False, log_level=rospy.DEBUG)
    rospy.loginfo("Image extractor initializing...")
    src = rospy.get_param('extractor/src')
    # Launch in execution
    try:
        img_extractor(src)
    except rospy.ROSInterruptException:
        rospy.logerr("Image extractor initialization error!")
        pass
