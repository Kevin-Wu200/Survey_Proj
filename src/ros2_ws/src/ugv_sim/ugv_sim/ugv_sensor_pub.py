#!/usr/bin/env python3
"""
UGV 传感器发布节点
功能：模拟 LiDAR（16线）、双目相机、IMU 传感器数据
- /ugv/lidar/points (PointCloud2) - 16线 LiDAR 点云
- /ugv/camera/left/image_raw/compressed (CompressedImage) - 左目相机
- /ugv/camera/right/image_raw/compressed (CompressedImage) - 右目相机
- /ugv/imu (Imu) - IMU 惯性测量单元
"""

import math
import time
import struct
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, Imu, CompressedImage, CameraInfo
from std_msgs.msg import Header


class UGVSensorPub(Node):
    """UGV 传感器仿真 - 发布 LiDAR、双目相机、IMU 数据"""

    def __init__(self):
        super().__init__('ugv_sensor_pub')

        # --- 参数 ---
        self.declare_parameter('lidar_lines', 16)
        self.declare_parameter('lidar_range', 100.0)      # LiDAR 量程 (m)
        self.declare_parameter('lidar_hz', 10.0)           # LiDAR 频率
        self.declare_parameter('lidar_points_per_line', 1800)  # 每线点数
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_hz', 30.0)
        self.declare_parameter('imu_hz', 100.0)

        lidar_lines = self.get_parameter('lidar_lines').value
        lidar_range = self.get_parameter('lidar_range').value
        lidar_hz = self.get_parameter('lidar_hz').value
        self.lidar_points_per_line = self.get_parameter('lidar_points_per_line').value
        camera_width = self.get_parameter('camera_width').value
        camera_height = self.get_parameter('camera_height').value
        camera_hz = self.get_parameter('camera_hz').value
        imu_hz = self.get_parameter('imu_hz').value

        # --- QoS ---
        best_effort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- 发布者 ---
        self.lidar_pub = self.create_publisher(
            PointCloud2, '/ugv/lidar/points', best_effort_qos)
        self.left_cam_pub = self.create_publisher(
            CompressedImage, '/ugv/camera/left/image_raw/compressed', 10)
        self.right_cam_pub = self.create_publisher(
            CompressedImage, '/ugv/camera/right/image_raw/compressed', 10)
        self.imu_pub = self.create_publisher(
            Imu, '/ugv/imu', best_effort_qos)

        # --- 定时器 ---
        self.lidar_timer = self.create_timer(
            1.0 / lidar_hz, self.lidar_callback)
        self.camera_timer = self.create_timer(
            1.0 / camera_hz, self.camera_callback)
        self.imu_timer = self.create_timer(
            1.0 / imu_hz, self.imu_callback)

        self.frame_seq = 0
        self.get_logger().info(
            f'UGV 传感器发布节点已启动 '
            f'(LiDAR: {lidar_hz}Hz, Camera: {camera_hz}Hz, IMU: {imu_hz}Hz)')

    # ------------------------------------------------------------------
    # LiDAR 点云生成 (16线)
    # ------------------------------------------------------------------
    def lidar_callback(self):
        """生成模拟16线 LiDAR 点云"""
        lines = 16
        points_per_line = self.lidar_points_per_line
        total_points = lines * points_per_line

        # 构建点云数据
        # 每点: x (float32), y (float32), z (float32), intensity (float32) = 16 bytes
        point_step = 16
        data = bytearray(total_points * point_step)
        t = time.time()

        # 垂直视场角: -15° ~ +15°, 均匀分布
        vert_angles = np.linspace(-15.0, 15.0, lines) * math.pi / 180.0
        # 水平角度: 0 ~ 360°
        horiz_angles = np.linspace(0, 2 * math.pi, points_per_line)

        for i, v_angle in enumerate(vert_angles):
            cos_v = math.cos(v_angle)
            sin_v = math.sin(v_angle)
            for j, h_angle in enumerate(horiz_angles):
                # 模拟一个有起伏的地面
                r = 5.0 + 3.0 * math.sin(h_angle * 3 + t) * math.cos(v_angle * 5)
                x = r * cos_v * math.cos(h_angle)
                y = r * cos_v * math.sin(h_angle)
                z = r * sin_v + 0.5  # LiDAR 安装高度0.5m

                intensity = max(0.0, min(255.0, 255.0 * (1.0 - r / 100.0)))

                offset = (i * points_per_line + j) * point_step
                struct.pack_into('<ffff', data, offset, x, y, z, intensity)

        # 构建消息
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ugv_lidar_frame'
        msg.height = 1
        msg.width = total_points
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = total_points * point_step
        msg.data = bytes(data)
        msg.is_dense = True

        self.lidar_pub.publish(msg)

    # ------------------------------------------------------------------
    # 双目相机图像生成
    # ------------------------------------------------------------------
    def camera_callback(self):
        """生成模拟双目相机图像"""
        width, height = 1280, 720
        t = time.time()

        # 左目图像
        left_img = self._generate_camera_image(width, height, t, 'LEFT')
        _, left_jpg = cv2.imencode('.jpg', left_img, [cv2.IMWRITE_JPEG_QUALITY, 80])

        left_msg = CompressedImage()
        left_msg.header = Header()
        left_msg.header.stamp = self.get_clock().now().to_msg()
        left_msg.header.frame_id = 'ugv_camera_left_frame'
        left_msg.format = 'jpeg'
        left_msg.data = left_jpg.tobytes()
        self.left_cam_pub.publish(left_msg)

        # 右目图像 (略有视差)
        right_img = self._generate_camera_image(width, height, t, 'RIGHT')
        _, right_jpg = cv2.imencode('.jpg', right_img, [cv2.IMWRITE_JPEG_QUALITY, 80])

        right_msg = CompressedImage()
        right_msg.header = Header()
        right_msg.header.stamp = self.get_clock().now().to_msg()
        right_msg.header.frame_id = 'ugv_camera_right_frame'
        right_msg.format = 'jpeg'
        right_msg.data = right_jpg.tobytes()
        self.right_cam_pub.publish(right_msg)

        self.frame_seq += 1

    @staticmethod
    def _generate_camera_image(width, height, t, side):
        """生成模拟相机图像"""
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # 地面 (下2/3)
        img[height // 3:, :] = [60, 80, 50]  # 棕绿色地面

        # 天空 (上1/3)
        img[:height // 3, :] = [200, 180, 100]  # 浅蓝天空

        # 道路
        road_left = width // 2 - 100
        road_right = width // 2 + 100
        for y in range(height // 3, height):
            # 透视收缩
            progress = (y - height // 3) / (height * 2 / 3)
            rl = int(road_left * (1.0 - progress * 0.6))
            rr = int(road_right * (1.0 + progress * 0.6))
            img[y, max(0, rl):min(width, rr)] = [80, 80, 80]  # 灰色道路

        # 路标
        for y in range(height // 3 + 50, height, 120):
            cy = y + int(10 * math.sin(t * 2 + y * 0.01))
            cv2.line(img, (width // 2, y), (width // 2, cy), (255, 255, 255), 3)

        # 信息叠加
        timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(img, f'UGV {side} Camera | {timestamp_str}',
                    (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        return img

    # ------------------------------------------------------------------
    # IMU 数据生成
    # ------------------------------------------------------------------
    def imu_callback(self):
        """生成模拟 IMU 数据 (带轻微噪声)"""
        t = time.time()
        noise = lambda amp: amp * math.sin(t * 1.7) * 0.1

        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ugv_imu_frame'

        # 线加速度 (传感器坐标系)
        msg.linear_acceleration.x = noise(0.5)
        msg.linear_acceleration.y = noise(0.3)
        msg.linear_acceleration.z = 9.81 + noise(0.1)

        # 角速度
        msg.angular_velocity.x = noise(0.01)
        msg.angular_velocity.y = noise(0.01)
        msg.angular_velocity.z = noise(0.05)

        # 协方差矩阵 (对角线填充)
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UGVSensorPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('UGV 传感器发布节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
