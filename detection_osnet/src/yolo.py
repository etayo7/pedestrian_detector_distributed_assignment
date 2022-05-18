#!/usr/bin/env python3

#   Imports

import os
from dataclasses import dataclass

import cv2
import numpy
import rospy
from cv2 import dnn_Net

#   Classes

class YOLO:
    #   Nested classes
    class YOLOFiles:
        def __init__(self, path: str, weight: str, cfg: str, names: str):
            self.path = path
            self.weight = weight
            self.cfg = cfg
            self.names = names

        def getPath(self, filetype: str):
            if filetype == "weight":
                aux = self.weight
            elif filetype == "cfg":
                aux = self.cfg
            elif filetype == "names":
                aux = self.names
            return os.path.realpath(os.path.join(self.path, aux))

    @dataclass
    class YOLONetwork:
        net: dnn_Net
        classes: list
        output_layers: list

    @dataclass
    class YOLOParameters:
        min_score: float
        max_overlap: float
        max_count: int

    #   Functions
    # def __init__(self, *args, **kwargs):
    #     self.params = None
    #     self.files = None
    #     self.network = None
    #     if ('min_score' in kwargs) and ('max_overlap' in kwargs) and ('max_count' in kwargs):
    #         min_score = kwargs.get('min_score')
    #         max_overlap = kwargs.get('max_score')
    #         max_count = kwargs.get('max_count')
    #         if (type(min_score) is not float) or (type(max_overlap) is not float) or (type(max_count) is not int):
    #             raise TypeError("Wrong argument type")
    #         self.params = self.YOLOParameters(min_score, max_overlap, max_count)
    #     elif ('Parameters' in kwargs):
    #         self.params = kwargs.get('Paramaters')
    #         if (type(self.params) is not self.YOLOParameters):
    #             print(type(self.params))
    #             print(self.params)
    #             raise TypeError("Wrong argument type")

    #     if 'Network' in kwargs:
    #         self.network = kwargs.get('Network')
    #         if (type(self.network) is not self.YOLONetwork):
    #             raise TypeError("Wrong argument type")
    #         return
    #     elif ('path' in kwargs) and ('weight' in kwargs) and ('cfg' in kwargs) and ('names' in kwargs):
    #         path = kwargs.get('path')
    #         weight = kwargs.get('weight')
    #         cfg = kwargs.get('cfg')
    #         names = kwargs.get('names')
    #         if (type(path) is not str) or (type(weight) is not str) or (type(cfg) is not str) or (type(names) is not str):
    #             raise TypeError("Wrong argument type")
    #         self.files = self.YOLOFiles(path, weight, cfg, names)
    #     elif 'Files' in kwargs:
    #         self.files = kwargs.get('Files')
    #         if (type(self.files) is not self.YOLOFiles):
    #             raise TypeError("Wrong argument type")
    #     net = cv2.dnn.readNet(self.files.getPath('weight'), self.files.getPath('cfg'))
    #     classes = []
    #     with open(self.files.names, "r") as f:
    #         classes = [line.strip() for line in f.readlines()]
    #     layer_names = net.getLayerNames()
    #     output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    #     self.network = self.YOLONetwork(net, classes, output_layers)

    def __init__(self, params: YOLOParameters, files: YOLOFiles):
        rospy.loginfo("YOLO network initializing!")
        self.params = params
        self.files = files
        self.network = self.YOLONetwork(None, None, None)
        self.network.net = cv2.dnn.readNet(
            self.files.getPath('weight'), self.files.getPath('cfg'))
        self.network.classes = []
        with open(self.files.getPath('names'), "r") as f:
            self.network.classes = [line.strip() for line in f.readlines()]
        layer_names = self.network.net.getLayerNames()
        self.network.output_layers = [layer_names[i - 1]
                                      for i in self.network.net.getUnconnectedOutLayers()]
        rospy.loginfo("YOLO network initialized succesfully.")

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
