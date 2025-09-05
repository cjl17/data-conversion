#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
import yaml
from pathlib import Path
import math

# ---------------- 配置 ----------------
OUTDIR = Path("output")   # 输出目录
ENABLE_MS33_FILTER = True  # 是否启用 ms33 筛选逻辑（仿 C++）
SAVE_FREQ = 100.0          # 保存频率 (Hz)
# --------------------------------------

def quaternion_to_yaw(q):
    """四元数 -> 偏航角 yaw"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def rotate_to_map(x, y, yaw):
    """将局部坐标系下的速度/加速度旋转到地图坐标系"""
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return cos_y * x - sin_y * y, sin_y * x + cos_y * y

class LocalizationLogger(Node):
    def __init__(self):
        super().__init__('localization_logger')

        # 订阅话题
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", self.pose_callback, 10)
        self.sub_pose_position = self.create_subscription(
            NavSatFix, "/sensing/gnss/fix", self.pose_position_callback, 10)
        self.sub_twist = self.create_subscription(
            TwistWithCovarianceStamped, "/localization/twist_estimator/twist_with_covariance", self.twist_callback, 10)
        self.sub_imu = self.create_subscription(
            Imu, "/sensing/imu/imu_data", self.imu_callback, 10)

        # 存储数据
        self.pose = None
        self.pose_position = None
        self.twist = None
        self.imu = None

        # 定时保存 (100 Hz)
        self.timer = self.create_timer(1.0 / SAVE_FREQ, self.timer_callback)

        # 输出目录
        OUTDIR.mkdir(exist_ok=True)

    def pose_callback(self, msg):
        self.pose = msg

    def pose_position_callback(self, msg):
        self.pose_position = msg

    def twist_callback(self, msg):
        self.twist = msg

    def imu_callback(self, msg):
        self.imu = msg

    def timer_callback(self):
        if not self.pose or not self.pose_position or not self.twist:
            self.get_logger().warn("等待 pose/pose_position/twist 数据...")
            return

        now = self.get_clock().now()
        ts = now.nanoseconds / 1e9

        # -------- 模仿 C++ 逻辑 --------
        if ENABLE_MS33_FILTER:
            nanosec = now.nanoseconds % int(1e9)   # 秒内纳秒数
            ms = nanosec / 1e6                     # 转毫秒
            ms33 = ms if ms < 10.0 else math.fmod(ms, 10.0)
            if not (0.0 <= ms33 <= 10.0):
                return   # 不在时间窗口，直接跳过
        # --------------------------------

        orientation = self.pose.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation)

        vel = self.twist.twist.twist.linear
        acc = self.twist.twist.twist.angular
        vel_x_map, vel_y_map = rotate_to_map(vel.x, vel.y, yaw)
        acc_x_map, acc_y_map = rotate_to_map(acc.x, acc.y, yaw)

        angularV = {"x": 0.0, "y": 0.0, "z": 0.0}
        if self.imu:
            angularV = {
                "x": float(self.imu.angular_velocity.x),
                "y": float(self.imu.angular_velocity.y),
                "z": float(self.imu.angular_velocity.z)
            }

        posCov = [float(x) for x in self.pose.pose.covariance] if hasattr(self.pose.pose, "covariance") else [0.0]*36
        velCov = [float(x) for x in self.twist.twist.covariance] if hasattr(self.twist.twist, "covariance") else [0.0]*9

        entry = {
            "header": {"frameId": "vehicle-lidar", "timestampSec": ts},
            "pose": {
                "orientation": {
                    "x": float(orientation.x),
                    "y": float(orientation.y),
                    "z": float(orientation.z),
                    "w": float(orientation.w)
                },
                "position": {
                    "x": float(self.pose_position.longitude),
                    "y": float(self.pose_position.latitude),
                    "z": float(self.pose_position.altitude)
                }
            },
            "vel": {"x": vel_x_map, "y": vel_y_map, "z": float(vel.z)},
            "acc": {"x": acc_x_map, "y": acc_y_map, "z": float(acc.z)},
            "angularV": angularV,
            "consistencyToMap": 1.0,
            "mode": 1,
            "poseConfidence": 1.0,
            "status": 2,
            "worldFrame": "WGS84",
            "posCov": posCov,
            "velCov": velCov
        }

        filename = OUTDIR / f"{int(ts*1e2)}.yaml"
        with open(filename, "w") as f:
            yaml.dump(entry, f, sort_keys=False)

        self.get_logger().info(f"✅ 保存 {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
