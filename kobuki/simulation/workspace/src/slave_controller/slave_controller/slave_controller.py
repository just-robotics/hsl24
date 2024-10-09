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
from geometry_msgs.msg import Quaternion, PointStamped, Vector3, TransformStamped, Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Bool


class SlaveController(Node):

    def __init__(self):
        super().__init__('slave_controller')
    
        self.create_subscription(PoseStamped, '/slave/target', self.controller_callback, 10)
        self.create_subscription(Odometry, '/slave/odom', self.sl_odom_callback, 10)
        self.create_subscription(Odometry, '/master/odom', self.mas_odom_callback, 10)
        self.create_subscription(PoseStamped, '/slave/origin', self.slave_origin_callback, 10)
        self.create_subscription(Bool, '/slave_ride_permission', self.slave_ride_permission_callback, 10)

        self.velocity_topic = '/slave/cmd_vel'
        self.point_topic = '/slave/goal_pose'
        self.master_gp_topic = '/goal_pose'
        self.slave_odom_topic = '/slave/odom'

        self.vel_publisher = self.create_publisher(Twist, self.velocity_topic, 10)
        self.point_publisher = self.create_publisher(PointStamped, self.point_topic, 10)
        self.gp_publisher = self.create_publisher(PoseStamped, self.master_gp_topic, 10)
        self.reached_gp_flag_publisher = self.create_publisher(PoseStamped, '/slave_target_is_reached', 10)

        self.max_linear_vel = 0.10
        self.max_angular_vel = 0.70
        self.ANGLE_ERR = 3  # degrees
        self.LINEAR_ERR = 0.05  # meters
        self.base_footprint_slave_link = 'base_footprint_slave'
        self.map_link = 'map'

        self.gp_vector = PoseStamped()
        self.slave_odom = Odometry()
        self.sl_odom_origin = PoseStamped()
        self.slave_odom_point = PointStamped()
        self.slave_ride_permission = False

    def controller_callback(self, goal_pose_: PoseStamped):
        if self.slave_ride_permission:
            goal_pose_in_so = self.map2bfs(goal_pose_)                                          # Преобразуем goal_pose в систему координат слейва
            gp_point = self.pose_st2point_st(goal_pose_in_so, self.base_footprint_slave_link)   # Получаем точку goal_pose в координатах slave_odom
            self.pose_from_points(self.slave_odom_point, gp_point)                              # Получаем вектор ориентации относительно slave_odom на точку goal_pose

            # Подсчет углов ориентации - - -
            slave_yaw = self.getYaw(self.slave_odom.pose.pose.orientation)
            gp_vector_yaw = self.getYaw(self.gp_vector.pose.orientation)
            # - - - - - - - - - - - - - - - 

            if abs(slave_yaw - gp_vector_yaw) >= self.ANGLE_ERR:
                self.turnLeft()
            elif abs(self.slave_odom.pose.pose.position.x - self.gp_vector.pose.position.x) >= self.LINEAR_ERR or abs(self.slave_odom.pose.pose.position.y - self.gp_vector.pose.position.y) >= self.LINEAR_ERR:
                self.moveForward()
            else:
                self.reached_gp_flag_publisher.publish(True)      

    def sl_odom_callback(self, msg: Odometry):
        self.slave_odom = msg

        point_st = PointStamped()
        point_st.header.frame_id = self.base_footprint_slave
        point_st.header.stamp = self.get_clock().now().to_msg()

        point_st.point.x = self.slave_odom.pose.pose.position.x
        point_st.point.y = self.slave_odom.pose.pose.position.y
        point_st.point.z = self.slave_odom.pose.pose.position.z

        self.slave_odom_point = point_st
    
    def mas_odom_callback(self, msg):
        self.master_odom = msg
    
    def slave_origin_callback(self, msg):
        self.sl_odom_origin = msg

    def slave_ride_permission_callback(self, msg):
        self.slave_ride_permission = msg

    def pose_from_points(self, point1: PointStamped, point2: PointStamped) -> PoseStamped:  # Рассчет в 'map', возвращает в системе координат 'slave_odom'
        # Вычисляем разницу координат между точками
        dx = point2.point.x - point1.point.x
        dy = point2.point.y - point1.point.y
        dz = point2.point.z - point1.point.z
        
        # Вычисляем углы yaw и pitch для ориентации
        yaw = math.atan2(dy, dx)       # Вращение вокруг оси z
        pitch = math.atan2(dz, math.sqrt(dx**2 + dy**2))  # Вращение вокруг оси y
        roll = 0.0  # Вращение вокруг оси x (обычно для таких задач не требуется)

        # Преобразуем углы yaw, pitch и roll в кватернион
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        
        # Создаем объект PoseStamped
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = rclpy.time.Time().to_msg()  # Текущее время ROS2
        pose.header.frame_id = self.map_link  # Укажите нужную систему координат

        # Устанавливаем позицию как point1
        pose.pose.position = point1

        # Устанавливаем ориентацию в кватернион
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # pose = self.map2bfs(pose)  # Перевод позиции в систему координат слейва

        self.gp_vector = pose

    def map2bfs(self, pose_: PoseStamped):
        pose_.pose.position.x = pose_.pose.position.x - self.sl_odom_origin.pose.position.x
        pose_.pose.position.y = pose_.pose.position.y - self.sl_odom_origin.pose.position.y

        new_orient = tf_transformations.euler_from_quaternion(pose_.pose.orientation)
        sl_odom_orient = tf_transformations.euler_from_quaternion(self.sl_odom_origin.pose.orientation)
        new_orient[0] = new_orient[0] - sl_odom_orient[0]
        new_orient[0] = new_orient[1] - sl_odom_orient[1]
        new_orient[0] = new_orient[2] - sl_odom_orient[2]
        pose_.pose.orientation = tf_transformations.quaternion_from_euler(new_orient)

        return pose_

    def getYaw(q_orient: Quaternion):
        orient = tf_transformations.euler_from_quaternion(q_orient)
        yaw = orient.z

        return yaw

    def moveForward(self):
        speed = Twist()
        speed.angular = 0.0
        speed.linear = self.max_linear_vel

        self.vel_publisher.publish(speed)
        
    def turnLeft(self):
        speed = Twist()
        speed.linear = 0.0
        speed.angular = -self.max_angular_vel

        self.vel_publisher.publish(speed)

    def publish_point(self, p : Point, frame_id):
        point_msg = PointStamped()

        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = frame_id
        
        point_msg.point.x = p.x
        point_msg.point.y = p.y
        point_msg.point.z = p.z
        
        self.point_publisher.publish(point_msg)

    def pose_st2point_st(self, pose_: PoseStamped, frame_id) -> PointStamped:
        point_st = PointStamped()
        point_st.header.frame_id = frame_id
        point_st.header.stamp = self.get_clock().now().to_msg()

        point_st.point.x = pose_.pose.position.x
        point_st.point.y = pose_.pose.position.y
        point_st.point.z = pose_.pose.position.z

        return point_st


def transform_pose_stamped(pose: PoseStamped, transform: TransformStamped) -> PoseStamped:
    # Извлекаем позицию и ориентацию из PoseStamped
    position = pose.pose.position
    orientation = pose.pose.orientation
    
    # Преобразуем позицию в массив
    p = [position.x, position.y, position.z]
    
    # Получаем кватернион из ориентации
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    
    # Получаем матрицы трансформации
    translation_matrix = tf_transformations.translation_matrix([transform.transform.translation.x,
                                                                transform.transform.translation.y,
                                                                transform.transform.translation.z])
    rotation_matrix = tf_transformations.quaternion_matrix([transform.transform.rotation.x,
                                                            transform.transform.rotation.y,
                                                            transform.transform.rotation.z,
                                                            transform.transform.rotation.w])
    
    # Объединяем матрицы
    transform_matrix = tf_transformations.concatenate_matrices(translation_matrix, rotation_matrix)
    
    # Преобразуем позицию в гомогенные координаты
    p_homogeneous = tf_transformations.concatenate_matrices(transform_matrix, tf_transformations.translation_matrix(p + [1.0]))
    
    # Извлекаем новую позицию из гомогенных координат
    new_position = p_homogeneous[:3, 3]
    
    # Преобразуем кватернион в новую ориентацию
    new_orientation = tf_transformations.quaternion_multiply(q, [transform.transform.rotation.x,
                                                                 transform.transform.rotation.y,
                                                                 transform.transform.rotation.z,
                                                                 transform.transform.rotation.w])
    
    # Создаем новый PoseStamped
    new_pose = PoseStamped()
    new_pose.header = pose.header  # Копируем заголовок
    new_pose.pose.position.x = new_position[0]
    new_pose.pose.position.y = new_position[1]
    new_pose.pose.position.z = new_position[2]
    new_pose.pose.orientation.x = new_orientation[0]
    new_pose.pose.orientation.y = new_orientation[1]
    new_pose.pose.orientation.z = new_orientation[2]
    new_pose.pose.orientation.w = new_orientation[3]
    
    return new_pose

def main(args=None):
    rclpy.init(args=args)
    
    sc = SlaveController()

    rclpy.spin(sc)

    sc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()