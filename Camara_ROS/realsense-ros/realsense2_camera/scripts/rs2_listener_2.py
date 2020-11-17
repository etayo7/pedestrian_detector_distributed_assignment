import rospy
import cv2
import imageio
import numpy as np
import pyrealsense2 as rs
import os
import shutil
import sys
import time
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Imu as msg_Imu
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import inspect
import ctypes
import struct
import tf


def make_clean_folder(path_folder):
    if not os.path.exists(path_folder):
        os.makedirs(path_folder)
    else:
        user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
        if user_input.lower() == "y":
            shutil.rmtree(path_folder)
            os.makedirs(path_folder)
        else:
            exit()


def record_rgbd():
    make_clean_folder("../data/realsense-ros/")

    pipeline = rs.pipeline()

    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_sensor.set_option(
        rs.option.visual_preset, 3
    )  # Set high accuracy for depth sensor
    depth_scale = depth_sensor.get_depth_scale()

    clipping_distance_in_meters = 1
    clipping_distance = clipping_distance_in_meters / depth_scale

    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            raise RuntimeError("Could not acquire depth or color frames.")

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        grey_color = 153
        depth_image_3d = np.dstack(
            (depth_image, depth_image, depth_image)
        )  # Depth image is 1 channel, color is 3 channels
        bg_removed = np.where(
            (depth_image_3d > clipping_distance) | (depth_image_3d <= 0),
            grey_color,
            color_image,
        )

        color_image = color_image[..., ::-1]

        imageio.imwrite("../data/realsense-ros/depth.png", depth_image)
        imageio.imwrite("../data/realsense-ros/rgb.png", color_image)


    finally:
        pipeline.stop()

    return color_image, depth_image


if __name__ == "__main__":
    record_rgbd()

