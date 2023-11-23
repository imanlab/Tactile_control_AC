#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import sys
import rospy

import numpy as np
import pyrealsense2 as rs

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    pipeline = rs.pipeline()
    config = rs.config()

    serial_number = "048122070681"  # 048122070681, 752112070781, 801212071197
    topic_name = "tilted_camera"

    config.enable_device(serial_number)

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    rospy.sleep(2)

    rospy.init_node('realsense_camera', anonymous=False)
    img_pub = rospy.Publisher(topic_name + '/color/image_raw', Image, queue_size=1)
    depth_pub = rospy.Publisher(topic_name +'/color/depth_raw', Image, queue_size=1)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        ros_color_image = bridge.cv2_to_imgmsg(np.array(color_image), "rgb8")
        ros_depth_image = bridge.cv2_to_imgmsg(np.array(depth_image), "passthrough")

        ros_color_image.header.stamp = rospy.Time.now()
        ros_depth_image.header.stamp = rospy.Time.now()
        img_pub.publish(ros_color_image)
        depth_pub.publish(ros_depth_image)


if __name__ == '__main__':
    main()