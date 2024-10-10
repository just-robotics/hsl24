import rclpy
import cv2 as cv
import numpy as np
import tf2_ros
import tf_transformations
import math
import time

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Time
from tf2_ros import TransformException, TransformBroadcaster
from geometry_msgs.msg import Quaternion, PointStamped, Vector3, TransformStamped, Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Bool
from std_msgs.msg import Float64


class SlaveController(Node):

    def __init__(self):
        super().__init__('slave_controller')
    
        self.create_subscription(Odometry, '/slave/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/slave/origin', self.slave_origin_callback, 10)
        self.create_subscription(Bool, '/slave_ride_permission', self.slave_ride_permission_callback, 10)
        self.create_subscription(Bool, '/aruco/found', self.aruco_callback, 10)
        self.create_subscription(Float64, '/aruco_pkg/delta_angle', self.delta_callback, 10)
        self.create_subscription(Bool, '/slave/start_rotation', self.start_rot_callback, 10)
        self.create_subscription(Bool, '/slave/start_translation', self.start_tran_callback, 10)
        self.create_subscription(PoseStamped, '/master/state', self.master_state_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/slave/cmd_vel', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/slave/odom/goal_pose', 10)
        self.rot_pub = self.create_publisher(Bool, '/slave/rotated', 10)
        self.tran_pub = self.create_publisher(Bool, '/slave/translated', 10)
        
        self.create_timer(0.005, self.rotation_callback)
        self.create_timer(0.01, self.translation_callback)
        
        self.get_logger().info('node started')

        self.ANGLE_ERR = 5  # degrees
        self.LINEAR_ERR = 0.05  # meters
        
        self.base_footprint_slave_frame = 'base_footprint_slave'
        self.map_frame = 'map'
        self.odom_slave_frame = 'odom_slave'

        self.goal_pose = PoseStamped()
        self.odom = Odometry()
        self.origin = PoseStamped()
        
        self.slave_ride_permission = True
        
        self.aruco = False
        
        self.start_rotation = False
        self.rotation = False
        
        self.start_translation = False
        self.translation = False
        
        self.target_angle = 0.0
        
        self.get_odom = False
        self.get_origin = False
        self.use_getting_origin = True
        self.delta = 0.0
        
        self.start_dist = 0.0
        
        self.master_state = PoseStamped()
        
        msg = Twist()
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        
    def aruco_callback(self, msg : Bool):
        if msg.data:
            self.aruco = True
            print('ARUCO')

    def slave_origin_callback(self, msg : PoseStamped):
        if self.use_getting_origin:
            self.sl_odom_origin = msg
            print('SLAVE ORIGIN', self.aruco)
            self.use_getting_origin = False
            self.get_origin = True
            
    def master_state_callback(self, msg : PoseStamped):
        self.master_state = msg
            
    def delta_callback(self, msg : Float64):
        self.delta = msg.data
        
    def start_rot_callback(self, msg : Bool):
        time.sleep(1)
        self.start_rotation = True
        
    def start_tran_callback(self, msg : Bool):
        time.sleep(1)
        self.start_translation = True
            
    def rotation_callback(self):
        
        msg = Twist()
        
        angle_error = abs(self.delta / 3.14 * 180)
    
        if angle_error > 20:
            msg.angular.z = 0.3 if self.delta < 0 else -0.3
        elif 10 < angle_error <= 20:
            msg.angular.z = 0.1 if self.delta < 0 else -0.1
        elif angle_error <= 10:
            msg.angular.z = 0.05 if self.delta < 0 else -0.05

        if self.start_rotation:
            print('START_ROTATION')
            self.start_rotation = False
            self.rotation = True
            
            self.cmd_vel_pub.publish(msg)
            return

        if self.rotation:
            self.get_logger().info(f'delta = {self.delta / 3.14 * 180}')
            
            if abs(self.delta / 3.14 * 180) < 20:
                self.rotation = False
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                self.get_logger().info(f'ok')
                rotated_msg = Bool()
                rotated_msg.data = True
                self.rot_pub.publish(rotated_msg)
            else:
                self.cmd_vel_pub.publish(msg)
                self.get_logger().info(f'not ok')
                
    def translation_callback(self):
        
        msg = Twist()
        msg.linear.x = 0.2
        
        if self.start_translation:
            print('START_TRANSLATION')
            self.start_translation = False
            self.translation = True
            self.start_odom = self.odom
            self.cmd_vel_pub.publish(msg)
            dx = self.start_odom.pose.pose.position.x - self.master_state.pose.position.x + self.origin.pose.position.x
            dy = self.start_odom.pose.pose.position.y - self.master_state.pose.position.y + self.origin.pose.position.y
            self.start_dist = math.sqrt(dx ** 2 + dy ** 2)
            return

        if self.translation:
            dx = self.origin.pose.position.x - self.odom.pose.pose.position.x
            dy = self.origin.pose.position.y - self.odom.pose.pose.position.y
            dist = math.sqrt(dx ** 2 + dy ** 2)
            self.get_logger().info(f'dist = {dist}, start_dist = {self.start_dist}')
            
            if dist > self.start_dist:
                self.translation = False
                msg.linear.x = 0.0
                self.cmd_vel_pub.publish(msg)
                self.get_logger().info(f'ok')
                translated_msg = Bool()
                translated_msg.data = True
                self.tran_pub.publish(translated_msg)
            else:
                self.cmd_vel_pub.publish(msg)
                self.get_logger().info(f'not ok')
        
    def odom_callback(self, msg: Odometry):
        self.odom = msg
        self.get_odom = True

    def slave_ride_permission_callback(self, msg):
        self.slave_ride_permission = msg


def euler_from_quaternion(x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return [roll_x, pitch_y, yaw_z] # in radians
        
        
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def main(args=None):
    rclpy.init(args=args)
    
    sc = SlaveController()

    rclpy.spin(sc)

    sc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()