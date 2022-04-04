#!/usr/bin/env python

# TARGETS CAMERA DETECTOR NODE - target_boxplot_track (no Args)

# TFM - Mircea-Alexandru Popovici
# Directors - Eduardo Montijano, Danilo Tardioli, Adrian Burlacu

from __future__ import print_function
from __future__ import division
import rospy
import sys
import cv2
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import dlib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import String, Int32, Int32MultiArray

# import the necessary packages
from imutils.object_detection import non_max_suppression
from imutils import paths
from imutils import face_utils
from simplex.msg import IntList
from simplex.msg import simplex
import argparse
import imutils

# Instantiate CvBridge
bridge = CvBridge()

# Load YOLO
weight_path = "/home/mircea98ro/catkin_ws/src/pedestrian_detector_distributed_assignment/detection/darknet/yolov3-tiny.weights"
tiny_path = "/home/mircea98ro/catkin_ws/src/pedestrian_detector_distributed_assignment/detection/darknet/yolov3-tiny.cfg"
names_path = "/home/mircea98ro/catkin_ws/src/pedestrian_detector_distributed_assignment/detection/darknet/coco.names"
net = cv2.dnn.readNet(weight_path, tiny_path)
classes = []
with open(names_path, "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))
font = cv2.FONT_HERSHEY_PLAIN
starting_time = time.time()
frame_id = 0

stimer = rospy.Time(0.0)


class image_frame:
    global img

    def __init__(self):
        # initialize ros publisher and subscriber
        image_str = 'image'
        image_topic = 'camera/color/image_raw'
        self.frame_received = 0
        self.imgS = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        self.img = self.imgS
        self.img_post = self.img
        self.pub_image = rospy.Publisher('Tracker', Image, queue_size=1)
        self.pub_log = rospy.Publisher('Log', String, queue_size=1)
        self.min_confidence = 0.5
        self.max_overlap = 0.5

    def callback(self, msg):
        self.imgS = msg
        self.frame_received = 1
        return self

    def process(self):
        orig = bridge.imgmsg_to_cv2(self.img, "bgr8")
        gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        drawImg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        self.img_post = bridge.cv2_to_imgmsg(drawImg, "bgr8")
        return self

    def darknet_detection(self):
        img = bridge.imgmsg_to_cv2(self.img, "bgr8")
        height, width, channels = img.shape

        # Detecting objects
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Showing informations on the screen
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if (confidence > self.min_confidence) and (class_id == 0):
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
        people = non_max_suppression(boxes, probs=None, overlapThresh=self.max_overlap)

        return [self, people]

    def color_detection(self, PD_loc, HSV_max, HSV_min, H_rec, S_rec, V_rec):  # color detector [HSV]
        imageFrame_orig = bridge.imgmsg_to_cv2(self.img_post, "bgr8")

        # initialize area for each target, 3 colours
        area_PD = [0] * np.size(PD_loc, 0)
        id = [0] * np.size(PD_loc, 0)
        i = 0

        # Morphological Transform, Dilation
        # for each color and bitwise_and operator
        # between imageFrame and mask determines
        # to detect only that particular color
        kernal = np.ones((5, 5), "uint8")

        for (xA, yA, xB, yB) in PD_loc:
            x_inc = int((xB - xA) * 0.35)
            y_inc = int((yB - yA) * 0.30)
            y_inc2 = int((yB - yA) * 0.50)
            xi = xA + x_inc
            xf = xB - x_inc
            yi = yA + y_inc
            yf = yB - y_inc2
            imageFrame = imageFrame_orig[yi:yf, xi:xf]

            # Convert the imageFrame in
            # BGR(RGB color space) to
            # HSV(hue-saturation-value)
            # color space
            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

            hue, sat, val = hsvFrame[:, :, 0], hsvFrame[:, :, 1], hsvFrame[:, :, 2]

            HSV_mean = [np.mean(hue), np.mean(sat), np.mean(val)]
            H_rec = np.append(H_rec, np.mean(hue))
            S_rec = np.append(S_rec, np.mean(sat))
            V_rec = np.append(V_rec, np.mean(val))

            HSV_max = np.maximum(HSV_mean, HSV_max)
            HSV_min = np.minimum(HSV_mean, HSV_min)

        return [HSV_max, HSV_min, H_rec, S_rec, V_rec]


def target_boxplot_track():

    # initialize image frame
    image = image_frame()
    
    # arguments
    image.min_confidence = float(sys.argv[1])
    image.max_overlap = float(sys.argv[2])

    #intialize node
    rospy.init_node('target_boxplot_track', anonymous=True)
    now = rospy.get_rostime()

    while not rospy.is_shutdown():
        time.sleep(5)
        print('Python version -> ', str(sys.version))
        print()
        print()
        try:
            input("Press Enter to start target tracking")
        except SyntaxError:
            pass
        print()
        t = rospy.Duration(0.1)
        now = rospy.get_rostime()
        secs = rospy.get_rostime() - now

        PD_location = []
        while PD_location.count == 0:
            if (image.frame_received == 1):
                image.img = image.imgS
                image.img_post = image.img
                [image, PD_location] = image.darknet_detection()  # locate targets
                image.frame_received = 0
                try:
                    image.pub_image.publish(bridge.cv2_to_imgmsg(image.img_post))
                except:
                    image.pub_image.publish(image.img_post)
                image.pub_log.publish("Image published")
            secs = rospy.get_rostime() - now
        print('Target detected in ', str(secs))
        print('Beginning tracking')
        print()
        print()

        
        now = rospy.get_rostime()
        secs = rospy.get_rostime() - now
        while not rospy.is_shutdown():
            if (image.frame_received == 1) and (secs > t):
                image.img = image.imgS
                image.img_post = image.img
                [image, PD_location] = image.darknet_detection()  # locate targets
                image.frame_received = 0

                imageFrame_orig = bridge.imgmsg_to_cv2(image.img_post, "bgr8")
                height, width = imageFrame_orig.shape[:2]
                imageFrame = imageFrame_orig
                for (xA, yA, xB, yB) in PD_location:
                    cv2.rectangle(imageFrame, (xA, yA), (xB, yB), [255, 255, 255], 2)
                image.pub_image.publish(bridge.cv2_to_imgmsg(imageFrame))
            secs = rospy.get_rostime() - now
        break
            
        


if __name__ == '__main__':
    try:
        target_boxplot_track()
    except rospy.ROSInterruptException:
        pass
