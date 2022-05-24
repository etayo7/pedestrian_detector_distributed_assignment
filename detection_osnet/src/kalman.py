#!/usr/bin/env python3

#   Imports

import os
from typing import List
os.environ["ROS_NAMESPACE"] = "/r_1"
from dataclasses import dataclass

import cv2
from cv2 import KalmanFilter
import numpy
import rospy
from cv2 import dnn_Net
from detection_osnet.msg import ProcessWindow, Window, WindowPack
from sensor_msgs.msg import Image


#   Globals



#   Classes

class Kalman:

    def __init__(self, source : str):
        self.dataS = rospy.Subscriber(f'processing/{source.lower()}', WindowPack, self.callback, queue_size=1)
        self.dataP = rospy.Publisher('processing/kalman', WindowPack, queue_size=1)

        self.pending = False
        self.rcv : WindowPack = None

        self.kf : List['KalmanFilter'] = []

        self.next : List['Window'] = []

    def process(self):

        skip_kalman = []
        window : ProcessWindow
        for  window in self.rcv.data:
            while window.assignment > len(self.kf):
                self.kf.append(KalmanFilter(4,2))
                self.kf[len(self.kf) - 1].measurementMatrix = numpy.array([[1, 0, 0, 0], 
                                                                           [0, 1, 0, 0]], numpy.float32)
                self.kf[len(self.kf) - 1].transitionMatrix = numpy.array([[1, 0, 1, 0], 
                                                                          [0, 1, 0, 1], 
                                                                          [0, 0, 1, 0], 
                                                                          [0, 0, 0, 1]], numpy.float32)
                self.kf[len(self.kf) - 1].processNoiseCov = numpy.eye(4, dtype = numpy.float32) * 0.03 
            while window.assignment > len(self.next):
                self.next.append(Window())
            self.next[window.assignment - 1] = window.window
            skip_kalman.append(window.assignment - 1)

            aux = numpy.array((self.next[window.assignment - 1].x, self.next[window.assignment - 1].y), numpy.float32)
            self.kf[window.assignment - 1].correct(aux)

        msg = WindowPack(img = self.rcv.img)

        for i in range(len(self.kf)):
            if (i in skip_kalman) == False:
                measurement = self.kf[i].predict()
                self.next[i].x = measurement[0]
                self.next[i].y = measurement[1]
            msg.data.append(ProcessWindow(window = self.next[i], assignment = i + 1))

        msg.header.stamp = self.rcv.header.stamp
        msg.timestamp = rospy.Time.now()
        self.dataP.publish(msg)
        self.pending = False

    def callback(self, msg : WindowPack):
        if self.pending == True:
            return
        self.rcv = msg
        self.pending = True
        

#   Functions

def kalman():

       
    obj = Kalman(source = rospy.get_param("cfg/kalman/source"))

    rospy.on_shutdown(world_end)

    rospy.loginfo("Robot Kalman node running!")

    while not rospy.is_shutdown():
        if obj.pending:
            obj.process()

def world_end():
    rospy.loginfo("Robot Kalman node shutting down.")


#   Guard

if __name__ == '__main__':    
    
    # Initialize node
    rospy.init_node("kalman", anonymous=False, log_level=rospy.DEBUG)
    rospy.loginfo("Robot Kalman Node initializing...")

    # Launch in execution
    try:
        kalman()
    except rospy.ROSInterruptException:
        rospy.logerr("Robot Kalman Node initialization error!")
        pass
