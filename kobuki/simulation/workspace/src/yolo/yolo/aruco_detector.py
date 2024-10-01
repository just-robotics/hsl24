
import rclpy
import cv2 as cv
import numpy as np
import tf2_ros
import tf_transformations
import math

from time import time
from rclpy.node import Node
from cv_bridge import CvBridge
from .submodules.yolo import Yolo
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Time
from tf2_ros import TransformException, TransformBroadcaster
from geometry_msgs.msg import Quaternion, PointStamped, Vector3, TransformStamped
from std_msgs.msg import Header


def transform_point(T, point):
    point_homogeneous = np.array([point[0], point[1], point[2], 1.0])  # (x, y, z, 1)
    point_transformed = np.dot(T, point_homogeneous)
    return point_transformed[:3]  # Возвращаем только (x, y, z)

def transform_stamped_to_matrix(transform_stamped):
    # Извлекаем трансляцию из TransformStamped
    translation = transform_stamped.transform.translation
    t = np.array([translation.x, translation.y, translation.z])

    # Извлекаем кватернион из TransformStamped и преобразуем его в матрицу вращения
    rotation = transform_stamped.transform.rotation
    q = [rotation.x, rotation.y, rotation.z, rotation.w]
    rotation_matrix = tf_transformations.quaternion_matrix(q)  # 4x4 матрица

    # Заменяем последний столбец на трансляцию
    transformation_matrix = np.copy(rotation_matrix)
    transformation_matrix[0:3, 3] = t  # Добавляем трансляцию в правую колонку

    return transformation_matrix

def matrix_to_transform_stamped(transformation_matrix, f1, f2):
    # Создаем новый TransformStamped объект
    transform_stamped = TransformStamped()

    # Извлекаем трансляцию из матрицы
    translation = transformation_matrix[0:3, 3]  # Последний столбец, первые три значения

    # Заполняем поле трансляции
    transform_stamped.transform.translation.x = translation[0]
    transform_stamped.transform.translation.y = translation[1]
    transform_stamped.transform.translation.z = translation[2]

    # Преобразуем матрицу вращения в кватернион
    quaternion = tf_transformations.quaternion_from_matrix(transformation_matrix)

    # Заполняем поле вращения
    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]

    transform_stamped.child_frame_id = f2
    transform_stamped.header.frame_id = f1

    return transform_stamped

def quaternion_from_euler(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = Quaternion()
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    q.w = cy * cp * cr + sy * sp * sr

    return q

class ArucoDetector(Node):

    def __init__(self):

        self.aruco_marker_size = 0.07  # m
        self.ROBOT_RADIUS = 0.177
        self.bridge = CvBridge()
        self.matrix = []
        self.dist_params = []
        
        self.Rvec_ = np.empty((4, 1))
        self.Tvec_ = np.empty((4, 1))
        self.markersID = np.empty((4, 1))

        self.frame_id = 'camera_link'
        self.child_frame_id = 'base_footprint_slave'

        super().__init__('aruco_detector')

        # SUB        
        self.subscription1 = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10)
        self.subscription1

        self.subscription2 = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.get_calibrated_params,
            10)
        self.subscription2

        # PUB
        self.img_publisher_ = self.create_publisher(Image, '/aruco_detection', 10)
        self.point_publisher_ =self.create_publisher(PointStamped, '/slave_position', 10)

        # TF
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        timer_period = 10 / 1000  # seconds
        self.timer = self.create_timer(timer_period, self.transformCallback)
        T_p_ = self.get_clock().now()
        T_p_ # to prevent unused variable warning

        self.blm_bfs_tf_ = TransformStamped()
        self.aruco_bls_tf_list = [TransformStamped(), TransformStamped(),
                                  TransformStamped(), TransformStamped()]
        self.t = TransformStamped()
        
        # Broadcaster
        self.br = TransformBroadcaster(self)

    
    def get_calibrated_params(self, msg):  # For simulation
        self.matrix = np.resize(np.asarray(msg.k), (3, 3))
        self.dist_params = np.asarray(msg.d)

    def callback(self, msg):
        aruco5 = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)
        parameters = cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(aruco5, parameters)

        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        mtx, dist = self.matrix, self.dist_params

        # image = cv.rotate(image, cv.ROTATE_180)
        cv.imshow('rgb', image)
        cv.waitKey(1)
        markerCorners, self.markersID, _ = detector.detectMarkers(image)
        if markerCorners:
            Yolo.find_actual_aruco(markerCorners, self.markersID)

        self.Rvec_, self.Tvec_ = Yolo.find_rvecs_tvecs(markerCorners, self.aruco_marker_size, mtx, dist)      

        if self.Rvec_ and self.Tvec_:
            print(self.markersID[0])
            print('Tvec =', self.Tvec_[0][0][0])
            print('Rvec =', self.Rvec_[0])
        
            # TF braodcaster
            t = TransformStamped()

            frame_id = self.frame_id
            child_frame_id = self.child_frame_id

            # Заполнение заголовка
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = frame_id  # Название родительской рамки
            t.child_frame_id = child_frame_id  # Название дочерней рамки

            # Задание трансляции (положение)
            t.transform.translation.x = self.Tvec_[0][0][0] 
            t.transform.translation.y = self.Tvec_[0][1][0]
            t.transform.translation.z = self.Tvec_[0][2][0]

            # Задание вращения (ориентация в виде кватерниона)
            q = quaternion_from_euler(self.Rvec_[0][0][0], self.Rvec_[0][1][0], self.Rvec_[0][2][0])

            print('RVEC0', self.Rvec_[0][0][0])
            print('RVEC1', self.Rvec_[0][1][0])
            print('RVEC2', self.Rvec_[0][2][0])

            t.transform.rotation.x = q.x
            t.transform.rotation.y = q.y
            t.transform.rotation.z = q.z
            t.transform.rotation.w = q.w

            print('T', transform_stamped_to_matrix(t))
            print('TRANSSTAMPED', transform_stamped_to_matrix(self.aruco_bls_tf_list[self.markersID[0][0]]))

            t = np.dot(transform_stamped_to_matrix(t), transform_stamped_to_matrix(self.aruco_bls_tf_list[self.markersID[0][0]]))

            self.t = matrix_to_transform_stamped(t, frame_id, child_frame_id)

            self.publish_point(transform_stamped_to_matrix(self.blm_bfs_tf_))
            
    def transformCallback(self):

        # base_link_master -> base_link_slave
        try:
            transform = self.tf_buffer.lookup_transform(self.child_frame_id, 'base_link', rclpy.time.Time())
            q = Quaternion()
            q.x = transform.transform.rotation.x 
            q.y = transform.transform.rotation.y
            q.z = transform.transform.rotation.z
            q.w = transform.transform.rotation.w

            t = Vector3()
            t.x = transform.transform.translation.x
            t.y = transform.transform.translation.y
            t.z = transform.transform.translation.z

            self.blm_bfs_tf_.transform.translation = t
            self.blm_bfs_tf_.transform.rotation = q
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {1} to {1}: {ex}')

        # aruco markers -> base_link_slave
        for idx in range(4):
            try:
                transform = self.tf_buffer.lookup_transform(self.child_frame_id, f'aruco_{idx}_cell_33', rclpy.time.Time())
                q = Quaternion()
                q.x = transform.transform.rotation.x 
                q.y = transform.transform.rotation.y
                q.z = transform.transform.rotation.z
                q.w = transform.transform.rotation.w

                t = Vector3()
                t.x = transform.transform.translation.x
                t.y = transform.transform.translation.y
                t.z = transform.transform.translation.z

                self.aruco_bls_tf_list[idx].transform.translation = t
                self.aruco_bls_tf_list[idx].transform.rotation = q
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {1} to {1}: {ex}')
        
        try:
            self.br.sendTransform(self.t)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {1} to {1}: {ex}')

    def publish_point(self, coords, frame_id="base_link"):

        print('coords =', coords)

        point_msg = PointStamped()

        point_msg.header = Header()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = frame_id
        
        # Заполняем координаты точки
        point_msg.point.x = float(coords[0][3])
        point_msg.point.y = float(coords[1][3])
        point_msg.point.z = float(coords[2][3])

        print('X', point_msg.point.x)
        print('Y', point_msg.point.y)
        print('Z', point_msg.point.z)

        # Публикуем сообщение
        self.point_publisher_.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    
    aruco = ArucoDetector()

    rclpy.spin(aruco)

    aruco.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()