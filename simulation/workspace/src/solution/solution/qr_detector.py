import cv2 as cv

from qreader import QReader

import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from robot_msgs.msg import StringMultiArray


class Detector(Node):

    def __init__(self):
        print('init')
        super().__init__('qr_detector')
        self.pub_ = self.create_publisher(StringMultiArray, '/result', 10)
        self.stop_pub_ = self.create_publisher(Bool, '/explore/resume', 10)
        self.sub_ = self.create_subscription(Image, '/camera/image_raw', self.callback, 1)
        self.bridge_ = CvBridge()
        self.qreader = QReader()
        self.msg = StringMultiArray()
        self.msg.header.frame_id = 'qr'
        for _ in range(1, 16):
            s = String()
            self.msg.data.append(s)
        self.cnt = 0
        print('node started')

    def callback(self, msg : Image):
        cv_frame = self.bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_frame = cv.cvtColor(cv_frame, cv.COLOR_BGR2RGB)
        text = self.qreader.detect_and_decode(image=cv_frame)
        self.fillMsg(text)
        
    def fillMsg(self, text : str | None):
        if text == None or len(text) == 0 or text[0] == None:
            return
        print(text)
        text = text[0]
        idx = int(text[0]) if text[1] == '.' else int(text[:2])
        print(idx)
        if (1 <= idx <= 15) and self.msg.data[idx-1].data == '':
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.msg.data[idx-1].data = text
            self.cnt += 1
            print('QR:', self.cnt)
        if (self.cnt == 15):
            m = Bool()
            m.data = True
            self.stop_pub_.publish(m)
            print('STOP')
        self.pub_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
