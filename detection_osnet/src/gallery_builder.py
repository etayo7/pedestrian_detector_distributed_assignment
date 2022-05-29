#!/usr/bin/env python3

#   Imports

import os
import cv2
from cv_bridge import CvBridge
import rospy
import rosbag
from  yolo import YOLO
from reid import ReID
import scipy
import numpy
from tkinter import filedialog


#   Defines

os.environ['ROS_NAMESPACE'] = 'r_1'

#   Globals

#   Classes

class MatObj:
    def __init__(self):
        pass

#   Functions

def gallery_builder():
    try:
        target_no = rospy.get_param("/master/target_no")

        files = YOLO.YOLOFiles(
            rospy.get_param("cfg/yolo/path"), 
            rospy.get_param("cfg/yolo/weight"), 
            rospy.get_param("cfg/yolo/cfg"), 
            rospy.get_param("cfg/yolo/names")
            )
        params = YOLO.YOLOParameters(
            rospy.get_param("cfg/yolo/min_score"), 
            rospy.get_param("cfg/yolo/max_overlap"), 
            rospy.get_param("cfg/yolo/min_box_confidence"), 
            rospy.get_param("cfg/yolo/max_count")
            )
        yolo_obj = YOLO(params = params, files = files)

        files = ReID.ReIDFiles(
            rospy.get_param("cfg/reid/path"), 
            rospy.get_param("cfg/reid/model")
            )
        params = ReID.ReIDParameters(
            rospy.get_param("cfg/reid/descriptor_size"), 
            rospy.get_param("cfg/reid/window_w"), 
            rospy.get_param("cfg/reid/window_ratio"), 
            rospy.get_param("cfg/reid/max_distance"), 
            rospy.get_param("cfg/reid/dynamic_gallery"), 
            rospy.get_param("/master/target_no"), 
            rospy.get_param("cfg/reid/redo_yolo_windows")
            )
        reid_obj = ReID(files, params)

    except Exception:
        rospy.logerr("Configuration info missing! Load configuration file / Run configuration script, then try again!")
        rospy.signal_shutdown("Node cannot run without configuration info!")
        return

    filetypes = (('JPEG Image', '*.jpg'), ('PNG Image', '*.png'))
    rospy.loginfo("Load images to create gallery...")

    filenames = filedialog.askopenfilename(
        title='Select images...',
        initialdir='.',
        filetypes=filetypes)

    for fn in filenames:
        img = cv2.imread(fn)
        
    pass

#   Guard

if __name__ == '__main__':    
    
    # Initialize node
    rospy.init_node("gallery_builder", anonymous=False, log_level=rospy.DEBUG)
    rospy.loginfo("Gallery Builder Node initializing...")

    # Launch in execution
    try:
        gallery_builder()
    except rospy.ROSInterruptException:
        rospy.logerr("Gallery Builder Node initialization error!")
        pass
