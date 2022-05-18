#!/usr/bin/env python3

#   Imports

from typing import Deque, NamedTuple

import cv2
import numpy
import rospy
from cv_bridge import CvBridge
from detection_osnet.msg import (  # Custom message types between robot and monitor
    Accuracy, MonitorUpdate, ProcessData, ProcessWindow)
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Time, Duration

#   Defines

DETECT_LVL_RAW = 0
DETECT_LVL_YOLO = 1
DETECT_LVL_OSNET = 2

#   Classes

class Monitor:


    class Results(NamedTuple):
        avg_process_time : rospy.Duration
        avg_accuracy_percent : numpy.float64
        avg_frame_loss_percent : numpy.float64

    class Reader:

        class ReceivePack(NamedTuple):
            img: Image
            data: ProcessData


        def __init__(self, ns: str = 'r_1', data_queue_len: int = 1, raw_img_queue_len: int = 1):
            self._msg_data = Deque['ProcessData']()
            self._pending = 0
            self._msg_img = Deque['Image']()
            self.dataS = rospy.Subscriber(
                ns+'/data', ProcessData, self.callback_data, queue_size=data_queue_len)
            self.imgS = rospy.Subscriber(
                ns+'/camera/color/image_raw', Image, self.callback_img, queue_size=raw_img_queue_len)

        def callback_data(self, msg : ProcessData):
            self._msg_data.append(msg)
            self._pending += 1
        
        def callback_img(self, msg : Image):
            self._msg_img.append(msg)
        
        def hasPending(self):
            return self._pending > 0
    
        def read(self):
            if self._pending <= 0:
                raise ValueError("No pending data to be read!")
            self._pending -=1
            data = self._msg_data.popleft()
            img : Image = None
            while self._msg_img:
                img = self._msg_img.popleft()
                if img.header.seq >= data.frame_seq:
                    break
            return self.ReceivePack(img, data)

    class Writer:

        def __init__(self, ns: str = 'r_1', post_queue_len: int = 1):
            self.imgP = rospy.Publisher(
                ns+'/results/image', Image, queue_size=post_queue_len)
            self.ptimeP = rospy.Publisher(ns+'/results/processing_time', Duration, queue_size=post_queue_len)
            self.dataP = rospy.Publisher(ns+'/metrics', MonitorUpdate, queue_size=1)
        
        def write_frame(self, post_image : Image, process_time : Duration):
            self.imgP.publish(post_image)
            self.ptimeP.publish(process_time)
            
        def update(self, data : MonitorUpdate = None):
            self.dataP.publish(data)

    def __init__(self, bridge : CvBridge, ns : str = None, data_queue_len: int = 1, raw_img_queue_len: int = 1, post_queue_len: int = 1, target_no : int = 0):
        self.ns = ns
        self.reader = self.Reader(ns, data_queue_len, raw_img_queue_len)
        self.writer = self.Writer(ns, post_queue_len)

        self.bridge = bridge
        self.history_duration = rospy.Duration.from_sec(10) # TODO: Make this variable
        self.target_no = target_no
        self.colors = numpy.random.uniform(0, 255, size=(target_no, 3))
        self.analyze_lvl = DETECT_LVL_OSNET # TODO: Make this variable

        self.process_time = Deque['rospy.Duration']()
        self.total_process_time = rospy.Duration()

        self.accuracy = numpy.zeros(4, dtype=numpy.uint64)   # tp, tn, fp, fn
        self.acc_hist = numpy.zeros((1,4), dtype=numpy.uint16)

        self.process_frame_count = numpy.uint64(0)
        self.lost_frames_hist = numpy.zeros(1,numpy.uint16)

        self.hist_timestamps = Deque["Time"]()
        self.hist_timestamps.append(rospy.Time.now())

        self.first_frame_id = 0
        self.last_frame_id = 0


    def update(self):

        now = rospy.Time.now()

        # Read data
        img, data = self.reader.read()
        
        # Start-time error mitigation
        if img is None:
            return

        # Marking point in history
        self.hist_timestamps.append(now)

        # Processing time
        self.total_process_time += data.delta
        self.process_time.append(data.delta)

        # Accuracy & Frame Loss
        self.process_frame_count += 1

        window_count = len(data.windows)
        if window_count == self.target_no:
            self.accuracy[0] += 1
            self.acc_hist = numpy.append(self.acc_hist, [[1, 0, 0, 0]], axis=0)
        elif window_count < self.target_no:
            self.accuracy[3] += 1
            self.acc_hist = numpy.append(self.acc_hist, [[0, 0, 0, 1]], axis=0)
        else:
            self.accuracy[2] += 1
            self.acc_hist = numpy.append(self.acc_hist, [[0, 0, 1, 0]], axis=0)

        if self.first_frame_id == 0:
            self.first_frame_id = img.header.seq
        else:
            self.lost_frames_hist = numpy.append(self.lost_frames_hist, img.header.seq - self.last_frame_id)
        self.last_frame_id = img.header.seq

        # Process Image
        img_cv2 = self.bridge.imgmsg_to_cv2(img,"bgr8")

        if self.analyze_lvl > DETECT_LVL_RAW:
            height, width, channels = img_cv2.shape
            color = [255, 255, 255] # White, default
            txtbox_h = int(height/30)
            txtbox_w = int(width/10)

            pw : ProcessWindow
            for pw in data.windows:
                xLeft = int(max(0, pw.yolo.x - pw.yolo.w/2))
                yUp = int(max(0, pw.yolo.y - pw.yolo.h/2))
                xRight = int(min(width, pw.yolo.x + pw.yolo.w/2 - 1))
                yDown = int(min(height, pw.yolo.y + pw.yolo.h/2 - 1))

                txt = str(pw.yolo.w) + 'x' + str(pw.yolo.h)

                if self.analyze_lvl > DETECT_LVL_YOLO:
                    if (pw.assignment <= self.target_no):
                        color = self.colors[pw.assignment - 1]
                        txt = "Person " + str(pw.assignment)
                    else:
                        color = [255, 255, 255] # White, default
                        txt = "!Person " + str(pw.assignment) + "!"

                
                # Main YOLO Window
                cv2.rectangle(img_cv2, (xLeft, yUp), (xRight, yDown), color, 2)

                # YOLO Textbox
                cv2.rectangle(img_cv2, (xLeft, yUp), (xLeft + txtbox_w, yUp + txtbox_h), color, -1)

                # YOLO Text
                # txtcolor = numpy.subtract([255, 255, 255], color).tolist()
                txtcolor = 0
                cv2.putText(img_cv2, txt, (xLeft + 1, yUp + int(txtbox_h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, txtcolor, 1)

                if self.analyze_lvl > DETECT_LVL_YOLO:
                    xLeft = int(max(0, pw.osnet.x - pw.osnet.w/2))
                    yUp = int(max(0, pw.osnet.y - pw.osnet.h/2))
                    xRight = int(min(width, pw.osnet.x + pw.osnet.w/2 - 1))
                    yDown = int(min(height, pw.osnet.y + pw.osnet.h/2 - 1))
                    cv2.rectangle(img_cv2, (xLeft, yUp), (xRight, yDown), color, 1)

        self.writer.write_frame(self.bridge.cv2_to_imgmsg(img_cv2, "bgr8"), data.delta)



    def getHistory(self):
        # Eliminate old data
        if self.hist_timestamps:
            while self.hist_timestamps[0] - rospy.Time.now() > self.history_duration:
                self.hist_timestamps.popleft()
                self.process_time.popleft()
                self.acc_hist = numpy.delete(self.acc_hist, 0, 0)
                self.lost_frames_hist = numpy.delete(self.lost_frames_hist, 0, 0)
        
        process_score = 0
        acc_percents = numpy.zeros(4)
        frame_loss_percent = Float64(0)

        # Process history window
        if self.hist_timestamps:
            count = len(self.hist_timestamps)
            # Processing time - not used
            process_score = numpy.true_divide(numpy.sum(self.process_time), count)

            # Accuracy
            acc_scores = self.acc_hist.sum(axis=0)
            acc_percents = numpy.true_divide(acc_scores, count)

            # Frame loss
            frame_loss_score = numpy.sum(self.lost_frames_hist)
            frame_loss_percent = numpy.true_divide(frame_loss_score, frame_loss_score + count)

        # Result interpretation
        acc_msg = Accuracy(true_positive = acc_percents[0], true_negative = acc_percents[1], false_positive = acc_percents[2], false_negative = acc_percents[3])
        return acc_msg, frame_loss_percent
            

    def service(self):
        acc, frame_loss = self.getHistory()
        self.writer.update(MonitorUpdate(accuracy = acc, frame_loss = frame_loss))

        if self.reader.hasPending() == False:
            return
        self.update()

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
