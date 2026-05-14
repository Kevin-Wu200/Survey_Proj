"""
图像校正节点 (Image Rectify Node)

订阅 UAV 原始图像，基于相机内参进行去畸变校正，发布校正后的图像。
支持 CompressedImage 和 Image 两种输入格式。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

# 尝试导入 cv_bridge（ROS2 环境中通常可用）
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

# 尝试导入 OpenCV
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# 尝试导入 tf2
try:
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


class ImageRectifyNode(Node):
    """图像校正节点

    订阅话题:
        /uav/image_raw/compressed (sensor_msgs/CompressedImage)
        /uav/image_raw (sensor_msgs/Image)

    发布话题:
        /fusion/image_rectified (sensor_msgs/Image)
    """

    def __init__(self):
        super().__init__('image_rectify_node')

        # ── 声明参数 ──────────────────────────────────────────────
        # 相机内参矩阵 K (3x3，行优先)
        self.declare_parameter('camera_matrix',
                               [800.0, 0.0, 960.0,
                                0.0, 800.0, 540.0,
                                0.0, 0.0, 1.0])
        # 畸变系数 D [k1, k2, p1, p2, k3]
        self.declare_parameter('distortion_coeffs',
                               [-0.2, 0.1, 0.0, 0.0, 0.0])
        # 图像分辨率（用于 newCameraMatrix）
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        # 是否广播 tf
        self.declare_parameter('broadcast_tf', True)

        # ── 读取参数 ──────────────────────────────────────────────
        K_flat = self.get_parameter('camera_matrix').get_parameter_value().double_array_value
        self.K = np.array(K_flat, dtype=np.float64).reshape(3, 3)

        D_flat = self.get_parameter('distortion_coeffs').get_parameter_value().double_array_value
        self.D = np.array(D_flat, dtype=np.float64)

        self.img_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.img_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.broadcast_tf = self.get_parameter('broadcast_tf').get_parameter_value().bool_value

        # ── 初始化 cv_bridge ──────────────────────────────────────
        if not CV_BRIDGE_AVAILABLE:
            self.get_logger().error('cv_bridge 不可用，节点无法运行。请安装 ros-${ROS_DISTRO}-cv-bridge')
            raise RuntimeError('cv_bridge 不可用')

        self.bridge = CvBridge()

        # ── 预计算去畸变映射表（效率优化）─────────────────────────
        if CV2_AVAILABLE:
            new_k, _ = cv2.getOptimalNewCameraMatrix(
                self.K, self.D, (self.img_width, self.img_height), 1.0)
            self.map_x, self.map_y = cv2.initUndistortRectifyMap(
                self.K, self.D, None, new_k,
                (self.img_width, self.img_height), cv2.CV_32FC1)
            self.get_logger().info(
                f'去畸变映射表已预计算，K={self.K.tolist()}, D={self.D.tolist()}')
        else:
            self.get_logger().error('OpenCV (cv2) 不可用，节点无法运行。')
            raise RuntimeError('OpenCV 不可用')

        # ── 订阅者 ────────────────────────────────────────────────
        # 同时订阅 CompressedImage 和 Image 两种主题
        self.compressed_sub = self.create_subscription(
            CompressedImage, '/uav/image_raw/compressed',
            self.compressed_image_callback, 10)
        self.raw_sub = self.create_subscription(
            Image, '/uav/image_raw',
            self.raw_image_callback, 10)

        # ── 发布者 ────────────────────────────────────────────────
        self.rectified_pub = self.create_publisher(
            Image, '/fusion/image_rectified', 10)

        # ── TF 广播器 ─────────────────────────────────────────────
        if TF2_AVAILABLE and self.broadcast_tf:
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            self._broadcast_static_tf()
        else:
            self.tf_broadcaster = None

        self.get_logger().info('图像校正节点已启动')

    def _broadcast_static_tf(self):
        """广播 camera_optical → camera_rectified 静态变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_optical'
        t.child_frame_id = 'camera_rectified'
        # 光学坐标系到校正坐标系的固定变换（此处为恒等变换，因为仅做去畸变）
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('静态 TF 已广播: camera_optical → camera_rectified')

    def _undistort(self, cv_image: np.ndarray) -> np.ndarray:
        """对 OpenCV 图像进行去畸变

        Args:
            cv_image: BGR 格式的 OpenCV 图像

        Returns:
            去畸变后的 BGR 图像
        """
        if cv_image.ndim == 2:
            # 灰度图
            return cv2.remap(cv_image, self.map_x, self.map_y,
                             cv2.INTER_LINEAR)
        else:
            return cv2.remap(cv_image, self.map_x, self.map_y,
                             cv2.INTER_LINEAR)

    def compressed_image_callback(self, msg: CompressedImage):
        """接收 CompressedImage 消息并处理"""
        # 解码 JPEG/PNG → OpenCV
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().error('无法解码 CompressedImage')
            return
        self._process_and_publish(cv_image, msg.header.stamp)

    def raw_image_callback(self, msg: Image):
        """接收 Image 消息并处理"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._process_and_publish(cv_image, msg.header.stamp)

    def _process_and_publish(self, cv_image: np.ndarray, stamp):
        """去畸变并发布图像"""
        try:
            undistorted = self._undistort(cv_image)
        except Exception as e:
            self.get_logger().error(f'去畸变失败: {e}')
            return

        # OpenCV → ROS Image
        rectified_msg = self.bridge.cv2_to_imgmsg(undistorted, encoding='bgr8')
        rectified_msg.header.stamp = stamp
        rectified_msg.header.frame_id = 'camera_rectified'
        self.rectified_pub.publish(rectified_msg)

    def destroy_node(self):
        self.get_logger().info('图像校正节点正在关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImageRectifyNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'节点启动失败: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
