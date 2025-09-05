#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from std_srvs.srv import SetBool
import yaml
from pathlib import Path
import math
import argparse

# ---------------- 可修改参数 ----------------
DATA_PER_FOLDER = 1000         # 每个文件夹包含多少条数据
ENABLE_MS_FILTER = True        # 是否按整百毫秒输出
# ------------------------------------------

def quaternion_to_yaw(q):
    """从四元数转换为航向角（yaw）"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def rotate_to_map(x_car, y_car, yaw):
    """将自车坐标系下的速度/加速度数据转换到 MAP 坐标系"""
    x_map = math.cos(yaw) * x_car - math.sin(yaw) * y_car
    y_map = math.sin(yaw) * x_car + math.cos(yaw) * y_car
    return x_map, y_map

class LocalizationExporter(Node):
    def __init__(self, outdir):
        super().__init__('localization_exporter')
        self.outdir = Path(outdir)
        self.outdir.mkdir(exist_ok=True)

        # 最新数据缓存
        self.pose = None
        self.pose_position = None
        self.twist = None
        self.imu = None

        # 控制是否保存数据
        self.save_enabled = True

        # 订阅话题
        self.create_subscription(PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', self.pose_cb, 10)
        self.create_subscription(NavSatFix, '/sensing/gnss/fix', self.pose_position_cb, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/localization/twist_estimator/twist_with_covariance', self.twist_cb, 10)
        self.create_subscription(Imu, '/sensing/imu/imu_data', self.imu_cb, 10)

        # 创建 Service 用于控制保存
        self.create_service(SetBool, '~/set_enable_save', self.enable_save_callback)

        # 输出计数
        self.count = 0
        self.folder_idx = 0
        self.current_folder = self.outdir / f"{self.folder_idx}"
        self.current_folder.mkdir(exist_ok=True)

        # 定时器 100 Hz
        self.create_timer(0.1, self.timer_cb)

    # Service 回调
    def enable_save_callback(self, request, response):
        self.save_enabled = request.data
        response.success = True
        response.message = "Save enabled" if self.save_enabled else "Save disabled"
        self.get_logger().info(f"Service set_enable_save called: {response.message}")
        return response

    # 回调更新最新数据
    def pose_cb(self, msg):
        self.pose = msg

    def pose_position_cb(self, msg):
        self.pose_position = msg

    def twist_cb(self, msg):
        self.twist = msg

    def imu_cb(self, msg):
        self.imu = msg

    def timer_cb(self):
        if not self.save_enabled:
            return  # 当前不保存，直接跳过
        if self.pose is None or self.pose_position is None:
            self.get_logger().warn("等待 pose/pose_position 数据...")
            return

        # 按整百毫秒过滤输出
        if ENABLE_MS_FILTER:
            nanosec = self.pose.header.stamp.nanosec % int(1e9)
            ms = nanosec / 1e6
            ms_hundred = ms if ms < 10.0 else math.fmod(ms, 10.0)
            if not (0.0 <= ms_hundred <= 100.0):
                return

        # 获取时间戳
        ts_sec = self.pose.header.stamp.sec
        ts_nsec = self.pose.header.stamp.nanosec

        # 获取方向
        orientation = self.pose.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation)

        # 获取速度/加速度，转换到 MAP 坐标系
        vel = self.twist.twist.twist.linear if self.twist else None
        acc = self.twist.twist.twist.angular if self.twist else None
        vel_x_map, vel_y_map = rotate_to_map(vel.x if vel else 0.0, vel.y if vel else 0.0, yaw)
        acc_x_map, acc_y_map = rotate_to_map(acc.x if acc else 0.0, acc.y if acc else 0.0, yaw)

        # IMU 补零
        angularV = {"x": 0.0, "y": 0.0, "z": 0.0}
        if self.imu:
            angularV = {
                "x": float(self.imu.angular_velocity.x),
                "y": float(self.imu.angular_velocity.y),
                "z": float(self.imu.angular_velocity.z)
            }

        # Covariance
        posCov = [float(x) for x in self.pose.pose.covariance] if hasattr(self.pose.pose, "covariance") else [0.0]*36
        velCov = [float(x) for x in self.twist.twist.covariance] if self.twist and hasattr(self.twist.twist, "covariance") else [0.0]*9

        # 输出 YAML
        entry = {
            "header": {"frameId": "vehicle-lidar", "timestampSec": ts_sec + ts_nsec*1e-9},
            "pose": {
                "position": {
                    "x": float(self.pose_position.longitude),
                    "y": float(self.pose_position.latitude),
                    "z": float(self.pose_position.altitude)
                },
                "orientation": {
                    "x": float(orientation.x),
                    "y": float(orientation.y),
                    "z": float(orientation.z),
                    "w": float(orientation.w)
                }
            },
            "vel": {"x": vel_x_map, "y": vel_y_map, "z": vel.z if vel else 0.0},
            "acc": {"x": acc_x_map, "y": acc_y_map, "z": acc.z if acc else 0.0},
            "angularV": angularV,
            "consistencyToMap": 1.0,
            "mode": 1,
            "posCov": posCov,
            "velCov": velCov
        }

        # 按 DATA_PER_FOLDER 分文件夹
        if self.count > 0 and self.count % DATA_PER_FOLDER == 0:
            self.folder_idx += 1
            self.current_folder = self.outdir / f"{self.folder_idx}"
            self.current_folder.mkdir(exist_ok=True)

        # 保存文件
        filename = self.current_folder / f"{ts_sec}_{ts_nsec}.yaml"
        with open(filename, 'w') as f:
            yaml.dump(entry, f, sort_keys=False)

        self.get_logger().info(f"✅ 已生成 YAML: {filename}")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Localization Data Exporter")
    parser.add_argument("--outdir", type=str, default="/tmp/pointcloud_data",
                        help="输出目录路径 (默认: /tmp/pointcloud_data)")
    cli_args = parser.parse_args()

    node = LocalizationExporter(cli_args.outdir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
