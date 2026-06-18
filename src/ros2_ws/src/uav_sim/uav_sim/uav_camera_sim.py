#!/usr/bin/env python3
"""
UAV 相机仿真节点
功能：模拟倾斜相机定时触发拍照，发布 /uav/image_raw
- 在无人机悬停状态下按固定间隔发布模拟图像数据
- 发布 CameraInfo 和 CompressedImage
"""

import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Header
from common_interfaces.msg import UAVStatus


class UAVCameraSim(Node):
    """UAV 相机仿真 - 模拟倾斜相机拍照"""

    def __init__(self):
        super().__init__('uav_camera_sim')

        # --- 参数 ---
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('capture_interval', 5.0)    # 拍照间隔 (秒)
        self.declare_parameter('jpeg_quality', 80)

        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.capture_interval = self.get_parameter('capture_interval').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # --- 状态 ---
        self.uav_in_hover = False
        self.last_capture_time = time.time()
        self.frame_seq = 0

        # --- QoS ---
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- 发布者 ---
        self.image_pub = self.create_publisher(
            CompressedImage, '/uav/image_raw/compressed', reliable_qos)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/uav/camera_info', reliable_qos)

        # --- 订阅者 ---
        self.status_sub = self.create_subscription(
            UAVStatus, '/uav/status', self.status_callback, reliable_qos)

        # --- 定时器 ---
        self.capture_timer = self.create_timer(
            0.5, self.capture_check_callback)  # 每0.5秒检查是否该拍照

        self.get_logger().info('UAV 相机仿真节点已启动')

    def status_callback(self, msg: UAVStatus):
        """接收无人机状态，判断是否在悬停"""
        self.uav_in_hover = (msg.flight_mode == 2)  # 飞行模式2=悬停

    def capture_check_callback(self):
        """检查是否满足拍照条件并触发"""
        now = time.time()
        if self.uav_in_hover and (now - self.last_capture_time >= self.capture_interval):
            self._capture_image()
            self.last_capture_time = now

    def _capture_image(self):
        """生成模拟图像并发布"""
        # 生成模拟图像：带有时序信息的检测卡
        img = np.zeros((self.img_height, self.img_width, 3), dtype=np.uint8)
        # 渐变色背景 (模拟天空地面分界)
        for y in range(self.img_height):
            if y < self.img_height * 0.6:
                # 天空 - 蓝色渐变
                color = (int(200 + y * 20 / self.img_height),
                         int(150 + y * 30 / self.img_height),
                         int(50 + y * 100 / self.img_height))
            else:
                # 地面 - 绿色/棕色
                color = (int(50 + (y - self.img_height * 0.6) * 50 / self.img_height),
                         int(100 + (y - self.img_height * 0.6) * 80 / self.img_height),
                         int(20 + (y - self.img_height * 0.6) * 30 / self.img_height))
            cv2.line(img, (0, y), (self.img_width, y), color, 1)

        # 十字标线
        cv2.line(img, (self.img_width // 2, 0),
                 (self.img_width // 2, self.img_height), (255, 255, 255), 1)
        cv2.line(img, (0, self.img_height // 2),
                 (self.img_width, self.img_height // 2), (255, 255, 255), 1)

        # 时间戳文字
        timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(img, f'UAV Camera | {timestamp_str}',
                    (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        cv2.putText(img, f'Frame: {self.frame_seq}',
                    (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.putText(img, 'Lat: 30.0000  Lon: 120.0000  Alt: 100.0m',
                    (30, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1)

        # JPEG 压缩
        _, jpeg_data = cv2.imencode('.jpg', img,
                                     [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])

        # 发布图像
        img_msg = CompressedImage()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'uav_camera_frame'
        img_msg.format = 'jpeg'
        img_msg.data = jpeg_data.tobytes()
        self.image_pub.publish(img_msg)

        # 发布相机信息
        cam_info = CameraInfo()
        cam_info.header = img_msg.header
        cam_info.height = self.img_height
        cam_info.width = self.img_width
        cam_info.distortion_model = 'plumb_bob'
        self.camera_info_pub.publish(cam_info)

        self.frame_seq += 1
        self.get_logger().info(f'拍照完成: 帧 #{self.frame_seq - 1}')


def main(args=None):
    rclpy.init(args=args)
    node = UAVCameraSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('UAV 相机仿真节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
