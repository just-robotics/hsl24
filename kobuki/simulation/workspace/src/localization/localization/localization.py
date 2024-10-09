from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

# from astra_camera_msgs.srv import GetCameraParams  
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped, Transform, Quaternion, PoseStamped
from std_msgs.msg import Bool, Float64



from cv_bridge import CvBridge
from tf2_ros import TransformException, TransformBroadcaster, Buffer
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


import cv2 as cv
import numpy as np
import math

DEBUG = True


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('detector_sub')

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)


        self.subscription = self.create_subscription(  # Можно добавить QoS если будут потери изображений
            Image,
            '/camera/image_raw',
            self.img_callback,
            qos_profile=qos_policy)
        
        
        self.ros2cv = CvBridge()
        self.subscription

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_base_tf_buffer = Buffer()
        self.listener_map_base = TransformListener(self.map_base_tf_buffer, self)

        self.slave_master_tf_buffer = Buffer()
        self.listener_slave_master = TransformListener(self.slave_master_tf_buffer, self)

        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)
        self.aruco_params = cv.aruco.DetectorParameters()
        self.aruco_detector = cv.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.aruco_joint_list = [0.177, 0.177, 0.177, 0.177]

        # self.get_calibrated_params()
        self.camera_matrix = np.array([[515.17841924,   0. ,        310.9425784 ],
                                       [  0. ,        514.31564875, 237.93036664],
                                       [  0. ,          0. ,          1.        ]], 
                                       dtype=np.float64
                                    )

        self.dist_params = np.array([ 0.07784166, -0.58879724, -0.00375178, -0.00643462,  1.07082767], dtype=np.float64)

        self.marker_size = 0.07
        cap_width = 640
        cap_height = 480
        
        self.slave_pos = PoseStamped()

        cap_x_center = cap_width // 2
        cap_y_center = cap_height // 2
        self.cap_center = (cap_x_center, cap_y_center)

        self.listener_data =  TransformStamped()

        # slave finded publisher
        self.publisher_ = self.create_publisher(Bool, '/aruco/found', 10)

        # slave pos world publisher
        self.slave_pos_publisher = self.create_publisher(PoseStamped, '/slave_pose', 10)

        self.diff_angle_pub = self.create_publisher(Float64, '/delta_angle', 10)
    
    def find_rvecs_tvecs(self, corners, marker_sz, cap_mtx, cap_dist):

        marker_points = np.array([[marker_sz / 2, -marker_sz / 2, 0],
                                            [-marker_sz / 2, -marker_sz / 2, 0],
                                            [-marker_sz / 2, marker_sz / 2, 0],
                                            [marker_sz / 2, marker_sz / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv.solvePnP(marker_points, c, cap_mtx, cap_dist, False, cv.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs
    
    def find_center(self, marker):  # for DEBUG
        x = marker.sum(axis=0) / len(marker)
        return x.astype(int)

    def draw_marker_pose(self, img, marker_list):  # for DEBUG
        if not marker_list:
            return
        for marker in marker_list:
            x, y = self.find_center(marker[0])
            cv.circle(img, (x, y), 4, (0, 255, 0), 2)
            cv.line(img, (x, y), self.cap_center, (0, 255, 255), 2)

    @staticmethod
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

    
    def send_tf(self):

        def rerange_aruco_3(angle):
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


        def aruco_rotation_fix(angle):
            if angle < -math.pi:
                angle += 2*math.pi
                # print (angle)
                return angle
                
            elif angle > math.pi:
                angle -= 2*math.pi
                # print (angle)
                return angle
            
            else:
                return angle
                
                
        to_frame_rel = 'aruco_' + str(self.marker_frame[0][0]) + '_cell_33'
        from_frame_rel = 'base_link_slave'

        try:
            self.listener_data = self.tf_buffer.lookup_transform(
            to_frame_rel,
            from_frame_rel,
            rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        


        self.t = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'camera_link'
        self.t.child_frame_id = 'base_link_slave'

        aruco_angle_orig = aruco_rotation_fix(self.slave_angle[1][0])
        aruco_angle_for_translate = aruco_angle_orig
        aruco_angle = aruco_angle_orig

        self.t.transform.translation.x = self.slave_pos_from_aruco[2][0] + self.aruco_joint_list[self.marker_frame[0][0]] * -math.cos(aruco_angle_for_translate)
        self.t.transform.translation.y = -self.slave_pos_from_aruco[0][0]+ self.aruco_joint_list[self.marker_frame[0][0]] * +math.sin(aruco_angle_for_translate)
        self.t.transform.translation.z = -self.slave_pos_from_aruco[1][0] + self.listener_data.transform.translation.z
        if self.markerIds[:1] == 3:
            aruco_angle =  rerange_aruco_3(aruco_angle_orig)
            print("remap_3!")

        if self.markerIds[:1] == 2:
            aruco_angle =  rerange_aruco_2(aruco_angle_orig)
            print("remap_2!")
            
        if self.markerIds[:1] == 1:
            aruco_angle =  rerange_aruco_1(aruco_angle_orig)
            print("remap_1!")


        slave_rot_x = 0.0
        slave_rot_y = 0.0
        slave_rot_z = -aruco_angle

        q = self.quaternion_from_euler(slave_rot_x, slave_rot_y, slave_rot_z)

        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(self.t)


    def get_slave_pos(self):
        msg_angle = Float64()

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

        to_frame1_rel = 'base_link'
        from_frame1_rel = 'map'

        from_frame2_rel = 'base_link_slave'
        to_frame2_rel = 'base_link'
        

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
            # self.get_logger().info(
            # f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            self.slave_pos.pose.position.x = 0.0
            self.slave_pos.pose.position.y = 0.0
            self.slave_pos.pose.position.z = -100.0

            self.slave_pos.pose.orientation.x = 0.0
            self.slave_pos.pose.orientation.y = 0.0
            self.slave_pos.pose.orientation.z = 0.0
            self.slave_pos.pose.orientation.w = 0.0
            return

        
        self.slave_pos.pose.position.x = (self.listener_map_base.transform.translation.x - self.listener_slave_master.transform.translation.x)
        self.slave_pos.pose.position.y = (self.listener_map_base.transform.translation.y - self.listener_slave_master.transform.translation.y)
        self.slave_pos.pose.position.z = (self.listener_map_base.transform.translation.z + self.listener_slave_master.transform.translation.z)

        a = self.listener_map_base.transform.rotation
        b = self.listener_slave_master.transform.rotation


        r1 = euler_from_quaternion(a.x, a.y, a.z, a.w)
        r2 = euler_from_quaternion(b.x, b.y, b.z, b.w)

        r = -(r2[2] + r1[2])

        msg_angle.data = r2[2]


        q = self.quaternion_from_euler(0.0, 0.0, r)

        self.slave_pos.pose.orientation.x = q[0]
        self.slave_pos.pose.orientation.y = q[1]
        self.slave_pos.pose.orientation.z = q[2]
        self.slave_pos.pose.orientation.w = q[3]

        
        self.diff_angle_pub.publish(msg_angle)

        return

    def img_callback(self, msg):
        frame = self.ros2cv.imgmsg_to_cv2(msg, "bgr8")
        markerCorners, self.markerIds, _ = self.aruco_detector.detectMarkers(frame)
        rvec, tvec = self.find_rvecs_tvecs(markerCorners[:1], self.marker_size, self.camera_matrix, self.dist_params)

        # slave found publisher
        msg = Bool()
        msg.data = False
        
        print(markerCorners)
        print(self.markerIds)


        if rvec and tvec:
            self.slave_pos_from_aruco = np.array([[float(tvec[0][0])], [float(tvec[0][1])], [float(tvec[0][2])]] )
            self.slave_angle = np.array([[float(rvec[0][0])], [float(rvec[0][1])], [float(rvec[0][2])]] )
            self.marker_frame = self.markerIds[:1]
            #Сюда вот захуячиваем тимфортреc
            self.slave_pos = PoseStamped()
            self.send_tf()

            # slave found
            msg.data = True
            self.get_slave_pos()

            if DEBUG:
                print(self.markerIds[:1])
                # self.draw_marker_pose(frame, markerCorners[:1])
                # cv.aruco.drawDetectedMarkers(frame, markerCorners[:1], self.markerIds[:1])
                # cv.drawFrameAxes(frame, self.camera_matrix, self.dist_params, rvec[0], tvec[0], 0.1)

        # publish is_slave_finded
        self.publisher_.publish(msg)
        self.slave_pos.header.stamp = self.get_clock().now().to_msg()
        self.slave_pos.header.frame_id = 'map'
        self.slave_pos.pose.orientation.z *= -1.0
        self.slave_pos_publisher.publish(self.slave_pos)
    
        if DEBUG:
            cv.circle(frame, self.cap_center, 4, (255, 0, 255), 2)
            cv.imshow('Camera view', frame)
            cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.stop()
    image_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
