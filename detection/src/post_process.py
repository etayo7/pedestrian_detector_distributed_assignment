#!/usr/bin/env python

# TARGETS CAMERA DETECTOR NODE - dk_find (Arg 1 = N Robots, Arg 2 = robot i, Arg 3 = costf =f(blob=0/dist=1))
# (Args 4 = Colours of the targets)

# Creator - Inigo Etayo
# Directors - Eduardo Montijano, Danilo Tardioli

# This node with four argument receives the frames obtained through a
# RGBD camera. At each frame tries to detect pedestrians and knowing the colours of
# each target, is able to establish the id of each pedestrian detected. Then using
# the depth of the camera, this node send to the assignment node the computed costs,
# and the desired positions towards of those faces.

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


class image_frame:
    global img

    def __init__(self, N):
        # initialize ros publisher and subscriber
        image_str = 'image'
        image_topic = 'camera/color/image_raw'
        self.frame_received = 0
        self.imgS = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
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
            if ((id[i] + 1) == self.assign):
                cv2.putText(image, 'Target: {}'.format(id[i] + 1), (xA2, yA2),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, [0, 0, 0], 2)
            else:
                cv2.putText(image, 'Target: {}'.format(id[i] + 1), (xA2, yA2),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, [0, 0, 0], 1)
            i = i + 1
        image = imutils.resize(image, width=640)
        self.img_post = bridge.cv2_to_imgmsg(image, "bgr8")
        return self

    def pixel2center(self, PD_loc, id, cost_function):
        global timer
        image = bridge.imgmsg_to_cv2(self.img_post, "bgr8")
        height, width = image.shape[:2]
        angle = self.angle
        angle = [0] * (np.size(angle, 0))
        cost = [x ** 0.5 for x in self.cost]  # mitigate detection gaps
        i = 0
        j = 0
        for h in self.cost:
            angle[j * 4] = (height / 2)
            angle[(j * 4) + 1] = (width / 2)
            angle[(j * 4) + 2] = 0
            angle[(j * 4) + 3] = 0
            j = j + 1

        for (xA, yA, xB, yB) in PD_loc:
            x = xA + ((xB - xA) / 2)
            y = yA + (1 * (yB - yA) / 5)
            [a, b, c, d] = self.convert_depth_image(y, x)
            cost[id[i]] = 1.0
            # add argument, cost = f(blob size or depth central pixel)
            if c > 0 and cost_function == 1:
                cost[id[i]] = (1 - (500 / c)) ** 3
            if c > 0 and cost_function == 0:
                size_img = height * width
                size_blob = (xB - xA) * (yB - yA)
                relative_size = float(size_blob / size_img)
                cost[id[i]] = (1 - relative_size) ** 5
            angle[id[i] * 4] = a
            angle[(id[i] * 4) + 1] = b
            angle[(id[i] * 4) + 2] = c
            angle[(id[i] * 4) + 3] = d
            if x > (3 * width / 8) and x < (5 * width / 8) and y > (height / 3) and y < (2 * height / 3):
                angle[id[i] * 4] = 0
                angle[(id[i] * 4) + 1] = 0
                angle[(id[i] * 4) + 2] = 0
                angle[(id[i] * 4) + 3] = 0
            i = i + 1
        if len(PD_loc) > 0:
            timer = rospy.get_rostime()
        else:
            if (rospy.get_rostime() - timer) > rospy.Duration(4):
                angle[(self.assign - 1) * 4] = 0.0
                angle[((self.assign - 1) * 4) + 1] = 0.0
                angle[((self.assign - 1) * 4) + 2] = 0.0
                angle[((self.assign - 1) * 4) + 3] = 0.1
        cost[self.assign - 1] = cost[self.assign - 1] / 1.05  # to increase the chances to assign the same target twice
        self.angle = angle
        self.cost = cost
        return self

    def convert_depth_image(self, fil, col):  # from Victor Fuertes TFM

        global u, v, a, b
        ros_image = self.img_depth
        depth_image = bridge.imgmsg_to_cv2(ros_image, "passthrough")

        # ask for the pixel that the user want to center
        u = fil
        v = col
        b = 0
        a = 0
        c = 0
        d = 0

        # Convert the depth image to a Numpy array
        depth_array = np.array(depth_image, dtype=np.float32)

        # obtain the depth from the pixel (we make the average of the surroundings pixels)
        arrows = 5
        colums = arrows
        arrow = int(u - (arrows - 1) / 2)
        colum = int(v - (colums - 1) / 2)
        depth_media = np.zeros([arrows, colums], dtype=float)

        for c in range(0, arrows):
            for d in range(0, colums):
                depth_media[c, d] = depth_array[arrow][colum]
                colum = colum + 1
            arrow = arrow + 1
            colum = int(v - (colums - 1) / 2)

        camera_intrinsics_fx = 613.0291137695312
        camera_intrinsics_fy = 613.4072265625

        camera_intrinsics_ppy = 250.34303283691406
        camera_intrinsics_ppx = 322.4624938964844

        # obtain the coordinate Z as the mean of the given pixel
        # and those around it

        divisor = 0
        sum_depth = 0.0
        z = 0.0

        for c in range(0, arrows):
            for d in range(0, colums):
                if depth_media[c][d] != "NaN" and depth_media[c][d] != 0.0:
                    sum_depth = sum_depth + depth_media[c][d]
                    divisor = divisor + 1
        if divisor > 0:
            z = sum_depth / float(divisor)

        x = (v - camera_intrinsics_ppx) * z / camera_intrinsics_fx
        y = (u - camera_intrinsics_ppy) * z / camera_intrinsics_fy

        return [x, y, z, z]


def dk_find():
    # arguments
    n_robots = int(sys.argv[1])
    robot = int(sys.argv[2]) - 1
    cost_function = int(sys.argv[3])
    r_colours = sys.argv[4:-2]  # ['blue','yellow','green','red']
    colours_v = np.ones(np.size(r_colours, 0))
    for ii in range(np.size(r_colours, 0)):
        colours_v[ii] = float(r_colours[ii])

    # initialize image frame
    image = image_frame(n_robots)
    rospy.init_node('dk_find', anonymous=True)
    now = rospy.get_rostime()
    t = rospy.Duration(0.1)

    while not rospy.is_shutdown():
        secs = rospy.get_rostime() - now
        if (image.frame_received == 1) and (secs > t):
            now = rospy.get_rostime()
            image.img = image.imgS
            image.img_depth = image.img_depthS
            image.img_post = image.img
            # targets detection darknet YOLO
            [image, PD_location] = image.darknet_detection()  # locate targets
            [image, id] = image.color_detection_darknet(PD_location, n_robots,
                                                        colours_v)  # assign each person to a target
            image = image.final_detection(PD_location, id, colours_v)  # assign each person to a boundaing box

            # show targets and colours detected
            image.pub_image.publish(image.img_post)

            # computing assignment depth-cost and
            # send (x,y,z) pixels to center with PTZ
            image = image.pixel2center(PD_location, id, cost_function)

            # publish image.cost and image.angle
            a = IntList()
            b = IntList()
            a.data = image.cost
            b.data = image.angle
            image.pub_cost.publish(a)
            image.pub_angle.publish(b)

            image.frame_received = 0


if __name__ == '__main__':
    try:
        dk_find()
    except rospy.ROSInterruptException:
        pass
