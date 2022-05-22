#!/usr/bin/env python3

# Monitor (master) node. Can read raw images and data from robot. Compiles statistics and presents data for visualisation

# TFM - Mircea-Alexandru Popovici
# Directors - Eduardo Montijano, Danilo Tardioli, Adrian Burlacu



#   Imports

from typing import List  # Typed datatypes

import rospy  # ROS-Python Interface
from cv_bridge import CvBridge  # Interface between OpenCV and ROS imaging

from monitor import Monitor  # Monitor class

#   Defines

#   Globals

bridge = CvBridge()

class Master:

    def __init__(self):
        self.robot_no = rospy.get_param('/master/robot_no')
        self.monitors : List["Monitor"] = []
        for i in range(self.robot_no):
            self.monitors.append(Monitor(bridge, f'r_{i+1}', target_no=rospy.get_param('/master/target_no')))

    def stats(self):
        id = 0
        for m in self.monitors:
            id += 1
            pt, acc, lf = m.summary()
            txt = f'Robot {id} summary: \n\tAverage processing time: {pt.to_sec()}s\n\tAverage accuracy: {acc}%\n\tAverage frame loss: {lf}%'
            rospy.loginfo(txt)

    def service(self):
        for m in self.monitors:
            if m.reader.hasPending():
                # rospy.loginfo("Monitor processing data!")
                m.service()

#   Functions

def master():

    # Initialization
    master = Master()

    rospy.loginfo("Master node active. ")

    # Normal operation
    while not rospy.is_shutdown():
        master.service()
    
    # End messages
    master.stats()

#   Guard
if __name__ == '__main__':    
    
    # Initialize node
    rospy.init_node("master", anonymous=False)
    rospy.loginfo("Master node initializing!")

    # Launch in execution
    try:
        master()
    except rospy.ROSInterruptException:
        rospy.logerr("Master node initialization error!")
        pass
