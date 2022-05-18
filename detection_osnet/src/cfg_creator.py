#!/usr/bin/env python3

# Run this script (not using roslaunch!) to set parameters. Then use `rosparam dump filename.yaml /cfg` to create a param file for loading later

import os
import rospy

def cfg_creator():
    rospy.set_param('cfg/yolo_path', os.path.join(os.path.dirname(__file__), '..', '..', 'detection', 'darknet'))
    rospy.set_param('cfg/yolo_weight', 'yolov3-tiny.weights')
    rospy.set_param('cfg/yolo_cfg', 'yolov3-tiny.cfg')
    rospy.set_param('cfg/yolo_names', 'coco.names')

    rospy.set_param('cfg/yolo_min_score', 0.3)
    rospy.set_param('cfg/yolo_max_overlap', 0.5)
    rospy.set_param('cfg/yolo_max_count', 10)

    rospy.set_param('cfg/osnet_path', os.path.join(os.path.dirname(__file__), '..', 'osnet_data'))
    rospy.set_param('cfg/osnet_model', 'osnet_ain_x0_25')

    rospy.set_param('cfg/osnet_descriptor_size', 1)
    rospy.set_param('cfg/osnet_window_w', 128)
    rospy.set_param('cfg/osnet_window_ratio', 2)
    rospy.set_param('cfg/osnet_max_distance', 0.6)

    rospy.set_param('master/robot_no', 1)
    rospy.set_param('master/target_no', 3)

#   Guard
if __name__ == '__main__':    

    # Launch in execution
    cfg_creator()