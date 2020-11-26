#!/usr/bin/env python

# TARGETS CAMERA DETECTOR NODE (post-processing) - post_process (Arg 1 = N Robots, Arg 2 = robot i)
# (Args 3 = Colours of the targets)

# Creator - Inigo Etayo
# Directors - Eduardo Montijano, Danilo Tardioli

# This node with three argument receives the frames obtained through a
# RGBD camera. At each frame tries to detect pedestrians and knowing the colours of
# each target, is able to establish the id of each pedestrian detected. Then outputs a video
# file with the frames processed.

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
import colorsys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import String, Int32, Int32MultiArray
import os

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
weight_path = "/home/divcore/odas-ws/src/detection/darknet/yolov3-tiny.weights"
tiny_path = "/home/divcore/odas-ws/src/detection/darknet/yolov3-tiny.cfg"
names_path = "/home/divcore/odas-ws/src/detection/darknet/coco.names"
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

r = int(sys.argv[2]) - 1
timer = rospy.Time(0.0)


# arguments
n_robots = int(sys.argv[1])
robot = int(sys.argv[2]) - 1
r_colours = sys.argv[3:-2]  # ['blue','yellow','green','red']
colours_v = np.ones(np.size(r_colours, 0))
for ii in range(np.size(r_colours, 0)):
    colours_v[ii] = float(r_colours[ii])


class image_frame:
    global img

    def __init__(self):
        # initialize ros publisher and subscriber
        image_str = 'image'
        self.pub_image = rospy.Publisher(image_str, Image, queue_size=1)
        image_topic = 'camera/color/image_raw'
        self.frame_received = 0
        self.imgS = rospy.Subscriber(image_topic, Image, self.callback, queue_size=10000)
        self.img = self.imgS
        self.img_post = self.img
        self.assign = 0
        self.assign = rospy.Subscriber('/topData', simplex, self.callback_assign, queue_size=1)

    def callback(self, msg):
        self.imgS = msg
        self.frame_received = 1

        return self

    def callback_assign(self, msg):
        id = msg.id - 1
        # self.assign = 0
        if id == r:
            self.assign = msg.assign
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
                if (confidence > 0.5) and (class_id == 0):
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
        people = non_max_suppression(boxes, probs=None, overlapThresh=0.65)

        return [self, people]

    def color_detection_darknet(self, PD_loc, N, colours_value):  # color detector [RGB]
        imageFrame_orig = bridge.imgmsg_to_cv2(self.img_post, "bgr8")

        # initialize area for each target, 3 colours
        id = [0] * np.size(PD_loc, 0)
        i = 0
        ids = []

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

            HSV_dist = np.ones((N, 1))

            for color in range(N):
                HSV_dist[color] = math.sqrt((1.2 * (HSV_mean[0] - colours_value[color * 3])) ** 2 + (
                            HSV_mean[1] - colours_value[color * 3 + 1]) ** 2 + (
                                                        HSV_mean[2] - colours_value[color * 3 + 2]) ** 2)

            HSV_dist[ids] = 3000.0
            id[i] = np.argmin(HSV_dist)
            ids.append(id[i])
            i = i + 1
        return [self, id]

    def final_detection(self, PD_loc, id, colours):
        image = bridge.imgmsg_to_cv2(self.img, "bgr8")
        height, width = image.shape[:2]
        colours_value = colours
        i = 0
        # draw the assignment
        xA = int(1 * width / 6)
        xA2 = xA + 7
        xB = width - 10
        yA = 10
        yB = int(height / 10)
        yB2 = yB - 7
        cv2.rectangle(image, (xA, yA), (xB, yB), [255, 255, 255], -1)
        if self.assign != 0:
            if (float(colours_value[(self.assign - 1) * 3 + 1])) > 100:
                rgb = colorsys.hsv_to_rgb(float(colours_value[(self.assign - 1) * 3]) / 180.0,
                                          1.0,
                                          1.0)
            else:
                rgb = colorsys.hsv_to_rgb(float(colours_value[(self.assign - 1) * 3]) / 180.0,
                                          float(colours_value[(self.assign - 1) * 3 + 1]) / 255.0,
                                          float(colours_value[(self.assign - 1) * 3 + 2]) / 255.0)
            rgb = np.multiply(rgb, 255.0)
            rgb = [rgb[2], rgb[1], rgb[0]]
            cv2.rectangle(image, (xA, yA), (xB, yB), rgb, 3)
        cv2.putText(image, 'Robot N: {}'.format(r + 1) + '  -  Assignment: Target {}'.format(self.assign), (xA2, yB2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, [0, 0, 0], 2)
        # draw the final bounding boxes
        for (xA, yA, xB, yB) in PD_loc:
            yA2 = yA - 25
            cv2.rectangle(image, (xA, yA2), (xB, yA), [255, 255, 255], -1)
            if (float(colours_value[(id[i]) * 3 + 1])) > 100:
                rgb = colorsys.hsv_to_rgb(float(colours_value[(id[i]) * 3]) / 180.0,
                                          1.0,
                                          1.0)
            else:
                rgb = colorsys.hsv_to_rgb(float(colours_value[(id[i]) * 3]) / 180.0,
                                          float(colours_value[(id[i]) * 3 + 1]) / 255.0,
                                          float(colours_value[(id[i]) * 3 + 2]) / 255.0)
            rgb = np.multiply(rgb, 255.0)
            rgb = [rgb[2], rgb[1], rgb[0]]
            cv2.rectangle(image, (xA, yA2), (xB, yA), rgb, 3)
            cv2.rectangle(image, (xA, yA), (xB, yB), rgb, 3)
            xA2 = xA + 5
            yA2 = yA - 5
            if (id[i] + 1) == self.assign:
                cv2.putText(image, 'Target:{}'.format(id[i] + 1), (xA2, yA2),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, [0, 0, 0], 2)
            else:
                cv2.putText(image, 'Target:{}'.format(id[i] + 1), (xA2, yA2),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, [0, 0, 0], 1)
            i = i + 1
        image = imutils.resize(image, width=640)
        self.img_post = bridge.cv2_to_imgmsg(image, "bgr8")
        return self

def dk_find():

    # initialize image frame
    image = image_frame()
    rospy.init_node('dk_find', anonymous=True)
    now = rospy.get_rostime()
    t = rospy.Duration(0.1)
    f = 1
    directory = r'/home/divcore/odas-ws/src/detection/images'
    os.chdir(directory)

    while not rospy.is_shutdown():
        secs = rospy.get_rostime() - now
        if secs > t:
            now = rospy.get_rostime()
            image.img = image.imgS
            image.img_post = image.img
            # targets detection darknet YOLO
            [image, PD_location] = image.darknet_detection()  # locate targets
            [image, id] = image.color_detection_darknet(PD_location, n_robots,
                                                      colours_v)  # assign each person to a target
            image = image.final_detection(PD_location, id, colours_v)  # assign each person to a bounding box
            # show targets and colours detected
            image.pub_image.publish(image.img_post)
            img = bridge.imgmsg_to_cv2(image.img_post, "bgr8")
            cv2.imwrite("frame{}.png".format(f), img)
            f = f+1


if __name__ == '__main__':
    try:
        dk_find()
    except rospy.ROSInterruptException:
        pass
