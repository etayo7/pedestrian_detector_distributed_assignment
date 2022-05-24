#!/usr/bin/env python3

#   Imports

from typing import Deque, NamedTuple

import cv2
import numpy
import rospy
from cv_bridge import CvBridge
from detection_osnet.msg import (  # Custom message types between robot and monitor
    Accuracy, MonitorUpdate, ProcessData, ProcessWindow, WindowPack)
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Time, Duration
import message_filters

#   Defines

DETECT_LVL_RAW = 0
DETECT_LVL_YOLO = 1
DETECT_LVL_OSNET = 2
DETECT_LVL_KALMAN = 3

#   Classes

class Monitor:


    class Results(NamedTuple):
        avg_process_time : rospy.Duration
        avg_accuracy_percent : numpy.float64
        avg_frame_loss_percent : numpy.float64

    class ReceivePack(NamedTuple):
        img: Image
        wp: WindowPack

    class Writer:

        def __init__(self, ns: str = None, post_queue_len: int = 1):
            self.imgP = rospy.Publisher(f'/r_{ns}/results/image', Image, queue_size=post_queue_len)
            self.dataP = rospy.Publisher(f'r_{ns}/results', MonitorUpdate, queue_size=post_queue_len)
        
        def write_frame(self, img : Image, data : MonitorUpdate):
            self.imgP.publish(img)
            self.dataP.publish(data)

    def __init__(self, bridge : CvBridge, colors : numpy.array = None, ns : str = None, data_queue_len: int = 1, raw_img_queue_len: int = 1, post_queue_len: int = 1, target_no : int = 0, source : str = None):
        

        self.rcv : self.ReceivePack
        imgS = message_filters.Subscriber(f'/r_{ns}/camera/color/image_raw', Image)
        dataS = message_filters.Subscriber(f'/r_{ns}/processing/{source}', WindowPack)
        self.reader = message_filters.ApproximateTimeSynchronizer([imgS, dataS], 10000, slop = 10000, reset=True)
        self.reader.registerCallback(self.callback)

        self.writer = self.Writer(ns, post_queue_len)

        self.bridge = bridge
        self.target_no = target_no
        self.colors = colors

        if source == 'yolo':
            self.level = DETECT_LVL_YOLO
        elif source == 'osnet':
            self.level = DETECT_LVL_OSNET
        elif source == 'kalman':
            self.level = DETECT_LVL_KALMAN

        self.process_time = rospy.Duration()
        self.total_process_time = rospy.Duration()

        self.accuracy = Accuracy()

        self.process_frame_count = numpy.uint64(0)

        self.pending = False

        self.first_frame_id = 0
        self.last_frame_id = 0

        

    def callback(self, rcvImg, rcvData):
        if self.pending:
            return
        rospy.loginfo("Monitor RX!")
        self.rcv = self.ReceivePack(img = rcvImg, wp = rcvData)
        self.pending = True

    def service(self):

        now = rospy.Time.now()

        # Processing time
        self.process_time = self.rcv.wp.timestamp - self.rcv.img.header.stamp
        self.total_process_time += self.process_time

        # Frame Loss
        self.process_frame_count += 1
        
        if self.first_frame_id == 0:
            self.first_frame_id = self.rcv.img.header.seq
            lost_frames = 0
        else:
            lost_frames = self.rcv.img.header.seq - self.last_frame_id
        self.last_frame_id = self.rcv.img.header.seq

        accuracy = Accuracy(0, 0, 0, 0)
        # Accuracy
        if self.level == DETECT_LVL_YOLO:
            accuracy.correct = min(len(self.rcv.wp.data), self.target_no)
            if len(self.rcv.wp.data) > self.target_no:
                accuracy.extra = len(self.rcv.wp.data) - self.target_no
            else:
                accuracy.missing = self.target_no - len(self.rcv.wp.data)
        elif self.level > DETECT_LVL_YOLO: # For tests where each person keeps its lane
            pw = sorted(self.rcv.wp.data, key = sortKeyProcessWindow) # Sort windows from leftmost to the right
            accuracy.missing = self.target_no
            for i in range(len(pw)):
                accuracy.correct += (i - accuracy.extra == pw[i].assignment)
                if (pw[i].assignment != i - accuracy.extra):
                    if pw[i].assignment <= self.target_no:
                        accuracy.missing -= 1
                        accuracy.wrong += 1
                    else:
                        accuracy.extra += 1                

        self.accuracy.correct   += accuracy.correct
        self.accuracy.wrong     += accuracy.wrong
        self.accuracy.missing   += accuracy.missing
        self.accuracy.extra     += accuracy.extra

        # Process Image
        img_cv2 = self.bridge.imgmsg_to_cv2(self.rcv.img,"bgr8")

        if self.level > DETECT_LVL_RAW:
            height, width, channels = img_cv2.shape
            color = [255, 255, 255] # White, default
            txtbox_h = int(height/30)
            txtbox_w = int(width/10)

            pw : ProcessWindow
            for pw in self.rcv.wp.data:
                xLeft = int(max(0, pw.window.x - pw.window.w/2))
                yUp = int(max(0, pw.window.y - pw.window.h/2))
                xRight = int(min(width, pw.window.x + pw.window.w/2 - 1))
                yDown = int(min(height, pw.window.y + pw.window.h/2 - 1))

                txt = str(pw.window.w) + 'x' + str(pw.window.h)

                if self.level > DETECT_LVL_YOLO:
                    if (pw.assignment <= self.target_no):
                        color = self.colors[pw.assignment - 1]
                        txt = f'Person {pw.assignment}'
                    else:
                        color = [255, 255, 255] # White, default
                        txt = f'Person ?'

                
                # Main YOLO Window
                cv2.rectangle(img_cv2, (xLeft, yUp), (xRight, yDown), color, 2)

                # YOLO Textbox
                cv2.rectangle(img_cv2, (xLeft, yUp), (xLeft + txtbox_w, yUp + txtbox_h), color, -1)

                # YOLO Text
                # txtcolor = numpy.subtract([255, 255, 255], color).tolist()
                txtcolor = 0
                cv2.putText(img_cv2, txt, (xLeft + 1, yUp + int(txtbox_h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, txtcolor, 1)

                self.writer.write_frame(img = self.bridge.cv2_to_imgmsg(img_cv2, "bgr8"), data = MonitorUpdate(accuracy = accuracy, frame_loss = lost_frames))
                self.pending = False

    def summary(self):
        try:
            processing_time : rospy.Duration = self.total_process_time.__div__(self.process_frame_count)
        except ValueError:
            processing_time = rospy.Time.from_sec(0)
        
        try:
            accuracy = numpy.true_divide(self.accuracy[0] + self.accuracy[1], numpy.sum(self.accuracy))*100
        except Exception:
            accuracy = 0

        try:
            frame_loss = numpy.true_divide(self.process_frame_count , self.last_frame_id - self.first_frame_id + 1)
        except ValueError:
            frame_loss = 0

        return self.Results(processing_time, accuracy, frame_loss)


#   Functions

def sortKeyProcessWindow(pw : ProcessWindow):
    return pw.window.x