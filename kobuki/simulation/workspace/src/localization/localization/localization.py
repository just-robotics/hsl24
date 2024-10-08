import rclpy
import cv2 as cv
import numpy as np
import tf2_ros
import tf_transformations
import math

from time import time
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Time
from tf2_ros import TransformException, TransformBroadcaster
from geometry_msgs.msg import Quaternion, PointStamped, Vector3, TransformStamped
from std_msgs.msg import Header, Bool


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')
    
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        self.point_publisher = self.create_publisher(PointStamped, '/slave_position', 10)
        self.res_pub = self.create_publisher(Bool, '/aruco/found', 10)
        
        self.bridge = CvBridge()
        
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        
        self.aruco_marker_size = 0.07  # m
        
        self.camera_link = 'camera_link'
        self.base_footprint_slave = 'base_footprint_slave'
        self.base_link = 'base_link'
        self.map = 'map'
        
        self.mtx = None
        self.dist = None
        self.mtx_init = False
        
        self.tf = TransformStamped()
        
        self.cl2bl_tf = TransformStamped()
        
        self.aruco_r = [TransformStamped() for _ in range(4)]
        
        self.aruco_r[0].transform.rotation.x = +0.000
        self.aruco_r[0].transform.rotation.y = +0.000
        self.aruco_r[0].transform.rotation.z = +0.000
        self.aruco_r[0].transform.rotation.w = +0.000
        
        self.aruco_r[1].transform.rotation.x = +0.000
        self.aruco_r[1].transform.rotation.y = +0.000
        self.aruco_r[1].transform.rotation.z = -0.707
        self.aruco_r[1].transform.rotation.w = +0.707
        
        self.aruco_r[2].transform.rotation.x = +0.000
        self.aruco_r[2].transform.rotation.y = +0.000
        self.aruco_r[2].transform.rotation.z = +1.000
        self.aruco_r[2].transform.rotation.w = +0.000
        
        self.aruco_r[3].transform.rotation.x = +0.000
        self.aruco_r[3].transform.rotation.y = +0.000
        self.aruco_r[3].transform.rotation.z = +0.707
        self.aruco_r[3].transform.rotation.w = +0.707
        
        self.aruco_tf_list = [TransformStamped() for _ in range(4)]

        self.detector = cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50),
                                               cv.aruco.DetectorParameters())
        
        timer_period = 10 / 1000  # seconds
        self.timer = self.create_timer(timer_period, self.transform_callback)
        
    def image_callback(self, msg):
        
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        cv.imshow('image', image)
        cv.waitKey(1)
        
        markerCorners, markersID, _ = self.detector.detectMarkers(image)
        if not markerCorners:
            return

        R, T = self.find_rvecs_tvecs(markerCorners)
        
        if not R or not T:
            print('nop')
            return
        
        T = T[0]
        R = R[0]
        marker_id = markersID[0][0]
        
        print(marker_id)
        
        aruco2clo = TransformStamped()

        aruco2clo.transform.translation.x = +T[2][0]
        aruco2clo.transform.translation.y = -T[0][0]
        aruco2clo.transform.translation.z = -T[1][0]

        rot_x = 0.0
        rot_y = 0.0
        rot_z = -R[0][1][0]

        q = quaternion_from_euler(rot_x, rot_y, rot_z)
        aruco2clo.transform.rotation = q
        
        if self.cl2bl_tf != None:
            self.tf = multiply_transforms(self.cl2bl_tf, multiply_transforms(aruco2clo, multiply_transforms(self.aruco_r[marker_id], self.aruco_tf_list[marker_id])))
            
            self.tf.header.stamp = self.get_clock().now().to_msg()
            self.tf.header.frame_id = self.base_link
            self.tf.child_frame_id = self.base_footprint_slave
            
        msg = Bool()
        msg.data = True
        self.res_pub.publish(msg)
        
    def transform_callback(self):
        # camera_link -> base_link
        try:
            transform = self.tf_buffer.lookup_transform(self.camera_link, self.base_link, rclpy.time.Time())
            q = Quaternion()
            q.x = transform.transform.rotation.x 
            q.y = transform.transform.rotation.y
            q.z = transform.transform.rotation.z
            q.w = transform.transform.rotation.w

            t = Vector3()
            t.x = transform.transform.translation.x
            t.y = transform.transform.translation.y
            t.z = transform.transform.translation.z

            self.cl2bl_tf.transform.translation = t
            self.cl2bl_tf.transform.rotation = q
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {1} to {1}: {ex}')
        
        # base_link -> base_footprint_slave
        try:
            transform = self.tf_buffer.lookup_transform(self.base_footprint_slave, self.base_link, rclpy.time.Time())
            q = Quaternion()
            q.x = transform.transform.rotation.x 
            q.y = transform.transform.rotation.y
            q.z = transform.transform.rotation.z
            q.w = transform.transform.rotation.w

            t = Vector3()
            t.x = transform.transform.translation.x
            t.y = transform.transform.translation.y
            t.z = transform.transform.translation.z
            
            tf = TransformStamped()

            tf.transform.translation = t
            tf.transform.rotation = q
            self.publish_point(tf, self.base_link)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {1} to {1}: {ex}')

        # aruco markers -> base_link_slave
        for idx in range(4):
            try:
                transform = self.tf_buffer.lookup_transform(self.base_footprint_slave, f'aruco_{idx}_cell_33', self.get_clock().now())
                q = Quaternion()
                q.x = transform.transform.rotation.x
                q.y = transform.transform.rotation.y
                q.z = transform.transform.rotation.z
                q.w = transform.transform.rotation.w

                t = Vector3()
                t.x = transform.transform.translation.x
                t.y = transform.transform.translation.y
                t.z = transform.transform.translation.z

                self.aruco_tf_list[idx].transform.translation = t
                self.aruco_tf_list[idx].transform.rotation = q
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {1} to {1}: {ex}')
        
        if self.tf != None:
            try:
                self.broadcaster.sendTransform(self.tf)
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {1} to {1}: {ex}')
    
    def transform_stamped_to_matrix(self, transform_stamped: TransformStamped):
        translation = transform_stamped.transform.translation
        return np.array([[translation.x], [translation.y], [translation.z]])
    
    def publish_point(self, T : TransformStamped, frame_id):
        tf = T if type(T) != TransformStamped else self.transform_stamped_to_matrix(T)
        
        point_msg = PointStamped()

        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = frame_id
        
        point_msg.point.x = tf[0][0]
        point_msg.point.y = tf[1][0]
        point_msg.point.z = tf[2][0]
        
        self.point_publisher.publish(point_msg)
        
    def camera_info_callback(self, msg):  # For simulation
        self.mtx = np.resize(np.asarray(msg.k), (3, 3))
        self.dist = np.asarray(msg.d)
        self.mtx_init = True

    def find_rvecs_tvecs(self, corners):

        if not self.mtx_init:
            return None, None
        
        sz = self.aruco_marker_size

        marker_points = np.array([[-sz / 2, +sz / 2, 0],
                                  [+sz / 2, +sz / 2, 0],
                                  [+sz / 2, -sz / 2, 0],
                                  [-sz / 2, -sz / 2, 0]], dtype=np.float32)
        rvecs = []
        tvecs = []
        for _ in corners:
            _, R, t = cv.solvePnP(marker_points, corners[0], self.mtx, self.dist, False, cv.SOLVEPNP_IPPE_SQUARE)

            rvecs.append(R)
            tvecs.append(t)
            
        return rvecs, tvecs
    
def transform_to_matrix(transform: TransformStamped):
    """Преобразовать TransformStamped в матрицу 4x4."""
    translation = [transform.transform.translation.x,
                   transform.transform.translation.y,
                   transform.transform.translation.z]
    rotation = [transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w]
    
    # Получить матрицу преобразования 4x4
    transform_matrix = tf_transformations.concatenate_matrices(
        tf_transformations.translation_matrix(translation),
        tf_transformations.quaternion_matrix(rotation)
    )
    return transform_matrix

def matrix_to_transform(matrix):
    translation = tf_transformations.translation_from_matrix(matrix)
    rotation = tf_transformations.quaternion_from_matrix(matrix)

    transform = TransformStamped()
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = rotation[0]
    transform.transform.rotation.y = rotation[1]
    transform.transform.rotation.z = rotation[2]
    transform.transform.rotation.w = rotation[3]
    return transform

def multiply_transforms(transform1, transform2):
    matrix1 = transform_to_matrix(transform1)
    matrix2 = transform_to_matrix(transform2)
    result_matrix = tf_transformations.concatenate_matrices(matrix1, matrix2)
    result_transform = matrix_to_transform(result_matrix)
    return result_transform

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

        q = Quaternion()
        q.x = cj*sc - sj*cs
        q.y = cj*ss + sj*cc
        q.z = cj*cs - sj*sc
        q.w = cj*cc + sj*ss

        return q

def main(args=None):
    rclpy.init(args=args)
    
    aruco = ArucoDetector()

    rclpy.spin(aruco)

    aruco.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()