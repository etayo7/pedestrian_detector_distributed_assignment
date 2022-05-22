#!/usr/bin/env python3

#   Imports

import os
from dataclasses import dataclass

import rospy
from torchreid.utils import FeatureExtractor

#   Classes

class ReID:
    #   Nested classes
    class ReIDFiles:
        def __init__(self, path: str, model: str):
            self.path = path
            self.model = model

        def getPath(self):
            return os.path.realpath(os.path.join(self.path, self.model + "_imagenet.pyth"))

    @dataclass
    class ReIDParameters:
        descriptor_size: int
        window_w: int
        window_ratio: float
        max_distance: float

    @dataclass
    class ReIDGallery:
        descriptors: list

    #   Functions

    def __init__(self, files: ReIDFiles, params: ReIDParameters, galleries: list = None, device: str = 'cpu'):
        rospy.loginfo("OSNET network initializing!")
        self.galleries = galleries
        self.dynamic_gallery = (galleries is None)
        self.params = params
        self.files = files
        self.extractor = FeatureExtractor(
            model_name=self.files.model, model_path=self.files.getPath(), device=device)
        rospy.loginfo("OSNET network initialized succesfully.")
