import rclpy
from rclpy.node import Node

# from astra_camera_msgs.srv import GetCameraParams  
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Bool, Float64, Int64

from cv_bridge import CvBridge
from tf2_ros import TransformException, TransformBroadcaster, Buffer
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import cv2 as cv
import numpy as np
import math


DEBUG = True
DEBUG_VISUAL = False

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

def runningAverage(f, name, size=3):
    # Проверяем, есть ли уже атрибут с этим именем (буфер) у функции
    if not hasattr(runningAverage, name):
        # Если нет, создаем буфер с начальным значением f, заполняя его 10 раз
        setattr(runningAverage, name, [f] * size)

    # Получаем буфер по имени
    buffer = getattr(runningAverage, name)

    # Move buffer to actually values ( [0, 1, 2] -> [1, 2, 3] )
    buffer = buffer[1:]
    buffer.append(f)

    # Обновляем буфер в атрибутах функции
    setattr(runningAverage, name, buffer)
    return sum(buffer) / len(buffer)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('detector')
        self.log = self.get_logger()

        self.declare_parameters(namespace='', parameters=[('camera_topic', ''),
                                                          ('aruco_size', 0.0),])
        
        camera_topic = self.get_parameter('camera_topic').value
        self.marker_size = self.get_parameter('aruco_size').value
        
        self.get_logger().info(f'camera_topic: {camera_topic}')
        self.get_logger().info(f'aruco_size: {self.marker_size}')

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)


        self.subscription = self.create_subscription(  # Можно добавить QoS если будут потери изображений
            Image,
            camera_topic,
            self.img_callback,
            qos_profile=qos_policy)
        
        self.ros2cv = CvBridge()
        self.subscription

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_link_slave_aruco_data =  TransformStamped()

        self.map_base_tf_buffer = Buffer()
        self.listener_map_base = TransformListener(self.map_base_tf_buffer, self)

        self.slave_master_tf_buffer = Buffer()
        self.listener_slave_master = TransformListener(self.slave_master_tf_buffer, self)

        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)
        self.aruco_params = cv.aruco.DetectorParameters()
        self.aruco_detector = cv.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.aruco_joint_list = [0.145, 0.17, 0.145, 0.17]
        
        # self.camera_matrix = np.array([[609.590,   0. ,        316.434 ],  # FOR REALSENSE
        #                                [  0. ,        608.161, 251.895],
        #                                [  0. ,          0. ,          1. ]], 
        #                                dtype=np.float64
        #                             )
        
        self.camera_matrix = np.array([[514.72893356391262,     0.0,    326.21895139522309],
                                    [  0.0, 516.08852372837407,     238.60031737476828],
                                    [  0. ,          0. ,          1. ]], 
                                    dtype=np.float64
                                )

        # self.dist_params = np.array([  1.8403912183064975e-03, 1.0106307489363739e+00, 4.3760504109051135e-04, -2.8568588407256457e-03, -3.7677955492638255e+00], dtype=np.float64)  # FOR REALSENSE
        self.dist_params = np.array([  0.087729253395234366, -0.35939483701819186,-0.006612650385822135, -0.0030275371611007723, 0.41418694816580887], dtype=np.float64)

        cap_width = 640
        cap_height = 480

        cap_x_center = cap_width // 2
        cap_y_center = cap_height // 2
        self.cap_center = (cap_x_center, cap_y_center)

        self.params_ret = False

        # slave finded publisher
        self.is_slave_found_publisher = self.create_publisher(Bool, '/aruco/found', 10)

        # slave pos world publisher
        self.slave_pos_publisher = self.create_publisher(PoseStamped, 'aruco_pkg/slave_pose', 10)

        # slave rotate angle publisher
        self.delta_angle = 0.0
        self.slave_diff_angle_publisher = self.create_publisher(Float64, 'aruco_pkg/delta_angle', 10)

        # master rotate pixels publisher
        self.master_diff_pixels = self.create_publisher(Int64, 'aruco_pkg/delta_pixels', 10)
    
    def find_rvecs_tvecs(self, image_corners) -> (list, list):
        marker_sz = self.marker_size
        cap_mtx = self.camera_matrix
        cap_dist = self.dist_params

        real_aruco_points = np.array([[marker_sz / 2, -marker_sz / 2, 0],
                                            [-marker_sz / 2, -marker_sz / 2, 0],
                                            [-marker_sz / 2, marker_sz / 2, 0],
                                            [marker_sz / 2, marker_sz / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for aruco_image_corners_set in image_corners:
            nada, R, t = cv.solvePnP(real_aruco_points, aruco_image_corners_set, cap_mtx, cap_dist, False, cv.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs
    
    def find_center(self, marker) -> (int, int):  # for DEBUG
        x = marker.sum(axis=0) / len(marker)
        return x.astype(int)

    def draw_marker_pose(self, img, marker_corners):
        if not marker_corners:
            return
        
        for marker in marker_corners:
            x, y = self.find_center(marker[0])
            cv.circle(img, (x, y), 4, (0, 255, 0), 2)
            cv.line(img, (x, y), self.cap_center, (0, 255, 255), 2) 

    def send_delta_angle(self):
        msg_diff_angle = Float64()
        msg_diff_angle.data = self.delta_angle
        self.slave_diff_angle_publisher.publish(msg_diff_angle)

    def send_delta_pixels(self, corners):
        if not corners:
            return
        
        for marker in corners:
            x, y = self.find_center(marker[0])
        diff_pixels_msg = Int64()  # Отправляем разницу в пикселях для поворота мастера
        diff_pixels_msg.data = self.cap_center[0] - int(x)
        self.master_diff_pixels.publish(diff_pixels_msg) 

    def send_tf(self):
        self.log.info(f'Get aruco_{self.marker_id} position')
        
        def rerange_aruco_3(angle):  # 4 маркера аруко образовывают окружность
            if angle < 0:
                angle += 1.5*math.pi
                return angle
            elif angle > 0:
                angle -= 0.5*math.pi
                return angle
            else:
                return angle
            

        def rerange_aruco_2(angle):
            if angle < 0:
                angle += math.pi
                return angle
            elif angle > 0:
                angle -= math.pi
                return angle
            else:
                return angle
            

        def rerange_aruco_1(angle):
            if angle < 0:
                angle += 0.5*math.pi
                return angle
            elif angle > 0:
                angle -= 1.5*math.pi
                return angle
            else:
                return angle


        # Переход в точке -п и п
        def aruco_rotation_fix(angle):
            if angle < -math.pi:
                angle += 2*math.pi
                return angle
                
            elif angle > math.pi:
                angle -= 2*math.pi
                return angle
            
            else:
                return angle

        # to_frame_rel = 'aruco_' + str(self.marker_id) + '_cell_33'
        to_frame_rel = 'aruco_' + str(self.marker_id) + '_cell_33'
        from_frame_rel = 'base_link_slave'

        # Получаем tf от бейс линка слейва до нужного аруко-маркера
        try:
            self.base_link_slave_aruco_data = self.tf_buffer.lookup_transform(
            to_frame_rel,
            from_frame_rel,
            rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # Создаем сообщение tf для отправки и заполняем
        self.tf_to_send = TransformStamped()
        self.tf_to_send.header.stamp = self.get_clock().now().to_msg()
        self.tf_to_send.header.frame_id = 'camera_link'
        self.tf_to_send.child_frame_id = 'base_link_slave'

        # Проверяем переход точки -п и п
        aruco_angle_orig = aruco_rotation_fix(self.slave_angle_from_aruco[1][0])


        # ЗАПОЛНЯЕМ ВЕКТОР ПЕРЕНОСА
        #
        # Координаты маркера показаны снизу, оси не совпадают.
        # Например, вращение вокруг оси z робота - вращение вокруг оси y маркера.
        # Аналогично с положением.
        #
        #       z
        #      /
        # x---0     АРУКО МАРКЕР - ВИД СПЕРЕДИ
        #     |
        #     y
        aruco_angle_for_translation = aruco_angle_orig

        # Складывается из положения аруко маркера + расстояние от маркера до центра робота с учетом поворота

        x_unfiltered = self.slave_pose_from_aruco[2][0] + self.aruco_joint_list[self.marker_id] * -math.cos(aruco_angle_for_translation)
        y_unfiltered = -self.slave_pose_from_aruco[0][0]+ self.aruco_joint_list[self.marker_id] * +math.sin(aruco_angle_for_translation)
        z_unfiltered = self.slave_pose_from_aruco[1][0] - self.base_link_slave_aruco_data.transform.translation.z
        
        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! UP 
        #! ARTEM IS EATING PRYANICS RIGHT NOW

        # Бегущее среднее для переноса
        self.tf_to_send.transform.translation.x = runningAverage(x_unfiltered, "tf_to_send_translation_x_buffer")
        self.tf_to_send.transform.translation.y = runningAverage(y_unfiltered, "tf_to_send_translation_y_buffer")
        self.tf_to_send.transform.translation.z = runningAverage(z_unfiltered, "tf_to_send_translation_z_buffer")


        # ЗАПОЛНЯЕМ ВЕКТОР ПОВОРОТА
        aruco_angle = aruco_angle_orig
        # Нормализуем угол, чтобы 4 маркера образовывали окружность от -п до п
        if self.markerIds[:1] == 3:
            aruco_angle =  rerange_aruco_3(aruco_angle_orig)
            self.log.info("Remap 3!")

        if self.markerIds[:1] == 2:
            aruco_angle =  rerange_aruco_2(aruco_angle_orig)
            self.log.info("Remap 2!")
            
        if self.markerIds[:1] == 1:
            aruco_angle =  rerange_aruco_1(aruco_angle_orig)
            self.log.info("Remap 1!")

        # Преобразуем в кватернионы
        slave_rot_x = 0.0
        slave_rot_y = 0.0
        slave_rot_z = -aruco_angle
        q = quaternion_from_euler(slave_rot_x, slave_rot_y, slave_rot_z)

        q0_unfiltered = q[0]
        q1_unfiltered = q[1]
        q2_unfiltered = q[2]
        q3_unfiltered = q[3]

        self.tf_to_send.transform.rotation.x = runningAverage(q0_unfiltered, "tf_to_send_rotation_q0_buffer")
        self.tf_to_send.transform.rotation.y = runningAverage(q1_unfiltered, "tf_to_send_rotation_q1_buffer")
        self.tf_to_send.transform.rotation.z = runningAverage(q2_unfiltered, "tf_to_send_rotation_q2_buffer")
        self.tf_to_send.transform.rotation.w = runningAverage(q3_unfiltered, "tf_to_send_rotation_q3_buffer")

        # Отправляем тф
        self.tf_broadcaster.sendTransform(self.tf_to_send)


        # ----------------DEBUG----------------
        # print(euler_from_quaternion(q[0], q[1], q[2], q[3])) # x y z w
        # print(euler_from_quaternion(x_tmp, y_tmp, z_tmp, w_tmp))
        # print("\n")
        # print("\n")
        # print("\n")
        # print(aruco_angle_for_translation)
        # print(self.slave_pos[2][0], self.slave_pos[0][0], self.slave_pos[1][0])
        # print(self.base_link_slave_aruco_data.transform.translation.x, self.base_link_slave_aruco_data.transform.translation.y, self.base_link_slave_aruco_data.transform.translation.z)
        # print(self.tf_to_send.transform.translation.x, self.tf_to_send.transform.translation.y, self.tf_to_send.transform.translation.z)
        # print("\n")
        # print("\n")
        # print("\n")
        # ----------------DEBUG----------------
    
    def send_slave_pos(self):
        self.slave_pos = PoseStamped()

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
        
            return roll_x, pitch_y, yaw_z # in radians

        from_frame1_rel = 'base_link'  # base_link осительно map
        to_frame1_rel = 'map'

        from_frame2_rel = 'base_link_slave'# base_link_slave осительно base_link
        to_frame2_rel = 'base_link'
        
        # Получаем tf от map -> base_link и base_link_slave -> base_link
        try:
            self.listener_map_base = self.map_base_tf_buffer.lookup_transform(
            to_frame1_rel,
            from_frame1_rel,
            rclpy.time.Time())

            self.listener_slave_master =  self.slave_master_tf_buffer.lookup_transform(
            to_frame2_rel,
            from_frame2_rel,
            rclpy.time.Time())

        except TransformException as ex:
            self.get_logger().info(
            f'Could not get transform because of error: {ex}')
            self.slave_pos.pose.position.x = 0.0
            self.slave_pos.pose.position.y = 0.0
            self.slave_pos.pose.position.z = -100.0

            self.slave_pos.pose.orientation.x = 0.0
            self.slave_pos.pose.orientation.y = 0.0
            self.slave_pos.pose.orientation.z = 0.0
            self.slave_pos.pose.orientation.w = 0.0
            return

        # fill header
        self.slave_pos.header.frame_id = "map"

        # ANGLE
        a = self.listener_map_base.transform.rotation
        b = self.listener_slave_master.transform.rotation

        # Перевод в эйлера для легкой математики
        r1 = euler_from_quaternion(a.x, a.y, a.z, a.w)
        r2 = euler_from_quaternion(b.x, b.y, b.z, b.w)
        r = r2[2] + r1[2]

        # Угол для поврота слейва
        # self.delta_angle = runningAverage(r2[2], "msg_diff_angle_buffer", 3)
        self.delta_angle = r2[2]
        angle_master_map = r1[2]

        # Делаем обратно кватернионы и записываем
        q = quaternion_from_euler(0.0, 0.0, r)

        q0_unfiltered_pos = q[0]
        q1_unfiltered_pos = q[1]
        q2_unfiltered_pos = q[2]
        q3_unfiltered_pos = q[3]

        self.slave_pos.pose.orientation.x = runningAverage(q0_unfiltered_pos, "slave_pos_to_send_rotation_q0_buffer")
        self.slave_pos.pose.orientation.y = runningAverage(q1_unfiltered_pos, "slave_pos_to_send_rotation_q1_buffer")
        self.slave_pos.pose.orientation.z = runningAverage(q2_unfiltered_pos, "slave_pos_to_send_rotation_q2_buffer")
        self.slave_pos.pose.orientation.w = runningAverage(q3_unfiltered_pos, "slave_pos_to_send_rotation_q3_buffer")

        # POSITION

        # Из них получаем позицию робота 
         # Из них получаем позицию робота 
        x_unfiltered_pos = (self.listener_map_base.transform.translation.x + self.listener_slave_master.transform.translation.x*math.cos(angle_master_map) - self.listener_slave_master.transform.translation.y*math.sin(angle_master_map))
        y_unfiltered_pos = (self.listener_map_base.transform.translation.y + self.listener_slave_master.transform.translation.x*math.sin(angle_master_map) + self.listener_slave_master.transform.translation.y*math.cos(angle_master_map))
        z_unfiltered_pos = (self.listener_map_base.transform.translation.z + self.listener_slave_master.transform.translation.z)

        self.slave_pos.pose.position.x = runningAverage(x_unfiltered_pos, "slave_pos_to_send_translation_x_buffer")
        self.slave_pos.pose.position.y = runningAverage(y_unfiltered_pos, "slave_pos_to_send_translation_y_buffer")
        self.slave_pos.pose.position.z = runningAverage(z_unfiltered_pos, "slave_pos_to_send_translation_z_buffer")

        self.slave_pos_publisher.publish(self.slave_pos)

        return

    def img_callback(self, msg):
        frame = self.ros2cv.imgmsg_to_cv2(msg, "bgr8")
        markerCorners, self.markerIds, _ = self.aruco_detector.detectMarkers(frame)
        pickedMarkerCorners = markerCorners[:1]
        rvec, tvec = self.find_rvecs_tvecs(pickedMarkerCorners)

        # slave found publisher
        msg_slave_found = Bool()
        msg_slave_found.data = False #default not found    

        if rvec and tvec:
            self.slave_pose_from_aruco = np.array([[float(tvec[0][0])], [float(tvec[0][1])], [float(tvec[0][2])]])
            self.slave_angle_from_aruco = np.array([[float(rvec[0][0]    )], [float(rvec[0][1])], [float(rvec[0][2])]] )
            self.marker_id = self.markerIds[:1][0][0]

            # slave found
            msg_slave_found.data = True

            # Сюда вот захуячиваем тимфортреc
            self.send_tf()

            # calculate and send slave pose
            self.send_slave_pos()

            # Отправляем дельта угол слейву
            self.send_delta_angle()

            # Отправляем дельта пиксели мастеру
            self.send_delta_pixels(pickedMarkerCorners)

            if DEBUG:
                self.log.info(f"MARKER ID is {self.marker_id}")
                self.log.info(f"COORDINATES:\n {self.slave_pos.pose.position}")
                self.log.info(f"ANGLES:\n {self.slave_angle_from_aruco}")
                if DEBUG_VISUAL:
                    self.draw_marker_pose(frame, pickedMarkerCorners)
                    cv.aruco.drawDetectedMarkers(frame, pickedMarkerCorners, self.markerIds[:1])
                    cv.drawFrameAxes(frame, self.camera_matrix, self.dist_params, rvec[0], tvec[0], 0.1)
        
        # Публиковать в любом случае
        self.is_slave_found_publisher.publish(msg_slave_found)

        if DEBUG_VISUAL:
            cv.circle(frame, self.cap_center, 4, (255, 0, 255), 2)
            cv.imshow('Camera view', frame)
            cv.waitKey(50)
        
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.stop()
    image_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()