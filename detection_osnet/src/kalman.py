#!/usr/bin/env python3

#   Imports

import os
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

        self.kf : list = [KalmanFilter()]

    def process(self):

        window : ProcessWindow
        for  window in self.rcv.data:
            while window.assignment > len(self.kf):
                self.kf.append(KalmanFilter())

        next = self.rcv

        for window in next.data:
            aux = numpy.array([window.window.x, window.window.y], numpy.float32)
            self.kf[window.assignment].correct(aux)
            window.x, window.y = self.kf[window.assignment].predict()

        next.timestamp = rospy.Time.now()
        self.dataP.publish(next)
        self.pending = False
        
            
    def filter_windows(self, boxes: list):
        # Picked indexes
        pick = []

        # Array of box coordintates
        x_start = []
        y_start = []
        x_end = []
        y_end = []
        a = []

        for box in boxes:
            dh = int(box.h/2)
            dw = int(box.w/2)
            x_start.append(box.x - dw)
            y_start.append(box.y - dh)
            x_end.append(box.x + dw)
            y_end.append(box.y + dh)
            a.append(box.h*box.w)

        x_start = numpy.array(x_start)
        y_start = numpy.array(y_start)
        x_end = numpy.array(x_end)
        y_end = numpy.array(y_end)
        a = numpy.array(a)

        # Sort boxes based on closeness to the camera
        indexes = numpy.argsort(y_end)
        # indexes = indexes.tolist()
        while len(indexes):
            end = len(indexes) - 1
            curr_index = indexes[end]
            # Pick the box
            pick.append(curr_index)

            # Find the largest overlapping box
            overlap_x_start = numpy.maximum(
                x_start[curr_index], x_start[indexes[:end]])
            overlap_y_start = numpy.maximum(
                y_start[curr_index], y_start[indexes[:end]])
            overlap_x_end = numpy.minimum(
                x_end[curr_index], x_end[indexes[:end]])
            overlap_y_end = numpy.minimum(
                y_end[curr_index], y_end[indexes[:end]])

            # Compute width and height of the overlapping box
            w = numpy.maximum(0, overlap_x_end - overlap_x_start + 1)
            h = numpy.maximum(0, overlap_y_end - overlap_y_start + 1)

            # Compute ratio of overlap
            overlap = (w * h) / a[indexes[:end]]

            # Delete indexes that go past the overlap threshold
            indexes = numpy.delete(indexes, numpy.concatenate(
                ([end], numpy.where(overlap > self.params.max_overlap)[0])))

        # Pick the windows
        return pick


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
