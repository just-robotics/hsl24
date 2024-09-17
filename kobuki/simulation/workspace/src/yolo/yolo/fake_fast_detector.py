#!/usr/bin/env python3


import cv2 as cv
import numpy as np
from time import time

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from dynamic_nav_interfaces.msg import YoloData


class MotionDetector(Node):

    def __init__(self):
        super().__init__('yolo')
        self.declare_parameters(namespace='', parameters=[('input_topic', ''),
                                                          ('output_topic', ''),])
        self.input_topic_ = self.get_parameter('input_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value
        self.get_logger().info(f'input_topic: {self.input_topic_}')
        self.get_logger().info(f'output_topic: {self.output_topic_}')
        self.subscription_ = self.create_subscription(YoloData, self.input_topic_, self.callback, 1)
        self.subscription_  # prevent unused variable warning
        self.publisher_ = self.create_publisher(YoloData, self.output_topic_, 10)
        self.bridge_ = CvBridge()
        self.h, self.w = (480, 640)
        self.empty_mask_ = np.zeros((self.h, self.w, 1), dtype='uint8')
        self.get_logger().info('model initialized')

    def callback(self, msg):
        t = time()

        msg.mask = self.bridge_.cv2_to_imgmsg(self.empty_mask_, encoding='passthrough')
        msg.boxes = []

        self.publisher_.publish(msg)

        self.get_logger().info(f'dt = {int((time() - t) * 1000)}ms')


def main(args=None):
    rclpy.init(args=args)
    motion_detector = MotionDetector()
    rclpy.spin(motion_detector)
    motion_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
