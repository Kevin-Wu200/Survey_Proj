"""
IMU 预积分模块单元测试 (5.2)
测试 IMUPreintegrator 的积分精度、信息矩阵和异常处理。
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from src.slam.data_types import IMUData, UGVFusionConfig
from src.slam.ugv_fusion import IMUPreintegrator


class TestIMUPreintegrator:
    """IMU 预积分器测试套件"""

    @pytest.fixture
    def config(self):
        return UGVFusionConfig(
            imu_frequency=100.0,
            imu_accel_noise=0.01,
            imu_gyro_noise=0.001
        )

    @pytest.fixture
    def integrator(self, config):
        return IMUPreintegrator(config)

    def test_initial_state(self, integrator):
        """测试初始状态：预积分器应初始化为恒等变换"""
        assert np.allclose(integrator.delta_R, np.eye(3))
        assert np.allclose(integrator.delta_v, np.zeros(3))
        assert np.allclose(integrator.delta_p, np.zeros(3))
        assert integrator.dt == 0.0

    def test_first_imu_no_integration(self, integrator):
        """测试第一帧 IMU 数据不应触发积分（需要两帧）"""
        imu = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                       gyro=np.array([0.0, 0.0, 0.0]))
        result = integrator.integrate(imu)
        assert result is False
        # 第一帧后 delta 保持恒等
        assert np.allclose(integrator.delta_R, np.eye(3))
        assert np.allclose(integrator.delta_p, np.zeros(3))

    def test_static_imu_integration(self, integrator):
        """测试静止 IMU 积分：仅重力加速度，不应产生水平位移"""
        # 第一帧
        imu1 = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                        gyro=np.array([0.0, 0.0, 0.0]))
        integrator.integrate(imu1)

        # 连续 10 帧静止
        for _ in range(10):
            imu = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                           gyro=np.array([0.0, 0.0, 0.0]))
            integrator.integrate(imu)

        pose = integrator.get_relative_pose()
        # 静止时旋转应接近恒等
        assert np.allclose(pose.R, np.eye(3), atol=0.01)
        # 水平平移应接近 0（仅重力方向有积分）
        assert abs(pose.t[0]) < 0.1
        assert abs(pose.t[1]) < 0.1

    def test_pure_translation_integration(self, integrator):
        """测试纯平移运动：沿x轴加速→减速"""
        # 第一帧（静止）
        imu1 = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                        gyro=np.array([0.0, 0.0, 0.0]))
        integrator.integrate(imu1)

        # 加速 5 帧（x 方向 1m/s²）
        for _ in range(5):
            imu = IMUData(accel=np.array([1.0, 0.0, 9.81]),
                           gyro=np.array([0.0, 0.0, 0.0]))
            integrator.integrate(imu)

        # 匀速 5 帧
        for _ in range(5):
            imu = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                           gyro=np.array([0.0, 0.0, 0.0]))
            integrator.integrate(imu)

        # 减速 5 帧（-1m/s²）
        for _ in range(5):
            imu = IMUData(accel=np.array([-1.0, 0.0, 9.81]),
                           gyro=np.array([0.0, 0.0, 0.0]))
            integrator.integrate(imu)

        pose = integrator.get_relative_pose()
        # 沿 x 轴应有正向位移
        assert pose.t[0] > 0, f"期望 x > 0，实际 x = {pose.t[0]:.4f}"
        # y 方向应接近 0
        assert abs(pose.t[1]) < 0.1

    def test_pure_rotation_integration(self, integrator):
        """测试纯旋转运动：绕z轴旋转"""
        # 第一帧
        imu1 = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                        gyro=np.array([0.0, 0.0, 0.0]))
        integrator.integrate(imu1)

        # 绕 z 轴以 ~1 rad/s 旋转 10 帧
        for _ in range(10):
            imu = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                           gyro=np.array([0.0, 0.0, 1.0]))
            integrator.integrate(imu)

        pose = integrator.get_relative_pose()
        # 旋转矩阵应偏离恒等
        assert not np.allclose(pose.R, np.eye(3), atol=0.01), \
            "旋转矩阵应与恒等不同"

        # 验证旋转矩阵正交性
        RRT = pose.R @ pose.R.T
        assert np.allclose(RRT, np.eye(3), atol=1e-5), \
            f"旋转矩阵非正交: R*R^T =\n{RRT}"

    def test_information_matrix(self, integrator):
        """测试信息矩阵计算"""
        # 积分几帧
        imu1 = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                        gyro=np.array([0.0, 0.0, 0.0]))
        integrator.integrate(imu1)
        for _ in range(5):
            imu = IMUData(accel=np.array([0.5, 0.0, 9.81]),
                           gyro=np.array([0.0, 0.1, 0.0]))
            integrator.integrate(imu)

        info = integrator.get_information_matrix()
        # 应为 6x6 矩阵
        assert info.shape == (6, 6)
        # 对角线应为正
        assert np.all(np.diag(info) > 0)
        # 应为对称矩阵
        assert np.allclose(info, info.T)

    def test_reset(self, integrator):
        """测试重置后状态恢复"""
        # 积分一些数据
        imu1 = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                        gyro=np.array([0.0, 0.0, 0.0]))
        integrator.integrate(imu1)
        for _ in range(10):
            integrator.integrate(IMUData(
                accel=np.array([1.0, 2.0, 9.81]),
                gyro=np.array([0.1, 0.2, 0.3])))

        # 重置
        integrator.reset()

        assert np.allclose(integrator.delta_R, np.eye(3))
        assert np.allclose(integrator.delta_v, np.zeros(3))
        assert np.allclose(integrator.delta_p, np.zeros(3))
        assert integrator.dt == 0.0

    def test_so3_exp_identity(self, integrator):
        """测试 SO(3) 指数映射：零旋转→恒等矩阵"""
        R = IMUPreintegrator._so3_exp(np.array([0.0, 0.0, 0.0]))
        assert np.allclose(R, np.eye(3))

    def test_so3_exp_axis(self, integrator):
        """测试 SO(3) 指数映射：绕轴旋转的正确性"""
        omega = np.array([0.0, 0.0, np.pi / 4])  # 绕z轴45°
        R = IMUPreintegrator._so3_exp(omega)

        # 验证正交性
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-5)
        # 验证行列式为 1
        assert abs(np.linalg.det(R) - 1.0) < 1e-5
        # z 轴应不变
        assert np.allclose(R @ np.array([0, 0, 1]), np.array([0, 0, 1]), atol=1e-5)

    def test_so3_exp_small_angle(self, integrator):
        """测试 SO(3) 指数映射：小角度近似"""
        omega = np.array([1e-10, 1e-10, 1e-10])
        R = IMUPreintegrator._so3_exp(omega)
        assert np.allclose(R, np.eye(3), atol=1e-8)

    def test_large_imu_buffer(self, integrator):
        """测试大量 IMU 数据积分稳定性"""
        imu1 = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                        gyro=np.array([0.0, 0.0, 0.0]))
        integrator.integrate(imu1)

        # 积分 100 帧
        np.random.seed(42)
        for _ in range(100):
            imu = IMUData(
                accel=np.array([0.0, 0.0, 9.81]) + np.random.randn(3) * 0.01,
                gyro=np.random.randn(3) * 0.001
            )
            integrator.integrate(imu)

        pose = integrator.get_relative_pose()
        # 长时间积分后旋转矩阵仍应保持正交
        RRT = pose.R @ pose.R.T
        assert np.allclose(RRT, np.eye(3), atol=1e-4), \
            f"长时间积分后旋转矩阵正交性退化"


class TestIMUDataTypes:
    """IMU 数据类型测试"""

    def test_imu_data_defaults(self):
        """测试 IMUData 默认值"""
        imu = IMUData(accel=np.array([1.0, 2.0, 3.0]),
                       gyro=np.array([0.1, 0.2, 0.3]))
        assert imu.accel.shape == (3,)
        assert imu.gyro.shape == (3,)
        assert imu.timestamp > 0
        assert imu.accel_cov is None
        assert imu.gyro_cov is None

    def test_imu_data_with_covariance(self):
        """测试带协方差矩阵的 IMUData"""
        imu = IMUData(
            accel=np.array([1.0, 2.0, 3.0]),
            gyro=np.array([0.1, 0.2, 0.3]),
            accel_cov=np.eye(3) * 0.01,
            gyro_cov=np.eye(3) * 0.001
        )
        assert imu.accel_cov.shape == (3, 3)
        assert imu.gyro_cov.shape == (3, 3)
