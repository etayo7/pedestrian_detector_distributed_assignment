#!/usr/bin/env python3

# Monitor (master) node. Can read raw images and data from robot. Compiles statistics and presents data for visualisation

# TFM - Mircea-Alexandru Popovici
# Directors - Eduardo Montijano, Danilo Tardioli, Adrian Burlacu



#   Imports

from typing import List

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
        colors = [  [68, 65, 249],
                    [142, 144, 77],
                    [79, 199, 249],
                    [44, 114, 243],
                    [144, 117, 87],
                    [109, 190, 144],
                    [30, 150, 248],
                    [161, 125, 39],
                    [139, 170, 67],
                    [74, 132, 249]
                ]
        # colors = numpy.array(colors, ndmin=2)

        try:
            for i in range(self.robot_no):
                self.monitors.append(Monitor(bridge = bridge, colors = colors, ns = i + 1, target_no=rospy.get_param('/master/target_no'), source = rospy.get_param(f'/r_{i+1}/cfg/monitor/level')))
        except Exception:
            rospy.logerr("Configuration info missing! Load configuration file / Run configuration script, then try again!")
            rospy.signal_shutdown("Node cannot run without configuration info!")
            return
            
    def stats(self):
        id = 0
        for m in self.monitors:
            id += 1
            pt, acc, lf = m.summary()
            txt = f'Robot {id} summary: \n\tAverage processing time: {pt.to_sec()}s\n\tAverage accuracy: {acc}%\n\tAverage frame loss: {lf}%'
            rospy.loginfo(txt)

    def service(self):
        for m in self.monitors:
            if m.pending:
                # rospy.loginfo("Monitor processing data!")
                m.service()

#   Functions

def master():

    # Initialization
    master = Master()

    rospy.on_shutdown(master.stats)

    if (not rospy.is_shutdown()):
        rospy.loginfo("Master node active. ")

    # Normal operation
    while not rospy.is_shutdown():
        master.service()
    
    # End messages
    # master.stats()

#   Guard
if __name__ == '__main__':    
    
    # Initialize node
    rospy.init_node("master", anonymous=False, log_level = rospy.DEBUG)
    rospy.loginfo("Master node initializing!")

    # Launch in execution
    try:
        master()
    except rospy.ROSInterruptException:
        rospy.logerr("Master node initialization error!")
        pass
