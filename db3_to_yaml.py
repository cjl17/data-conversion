#!/usr/bin/env python3
import sqlite3
import sys
import os
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml
import bisect

def main(db3_file):
    if not os.path.exists(db3_file):
        print(f"文件不存在: {db3_file}")
        sys.exit(1)

    # 输出路径固定为输入文件夹下 output_yaml
    db3_path = Path(db3_file).parent
    outdir = db3_path / "output_yaml"
    outdir.mkdir(exist_ok=True)

    rclpy.init()
    conn = sqlite3.connect(db3_file)
    cursor = conn.cursor()

    # 获取 topic id
    cursor.execute("SELECT id, name FROM topics")
    topics = {name: id for id, name in cursor.fetchall()}

    need_topics = {
        "pose": "/sensing/gnss/pose_with_covariance",
        "twist": "/localization/twist_estimator/twist_with_covariance",
        "imu": "/sensing/imu/imu_data"
    }

    for key in ["pose", "twist"]:
        if need_topics[key] not in topics:
            print(f"缺少topic: {need_topics[key]}")
            sys.exit(1)

    imu_exists = need_topics["imu"] in topics

    # 消息类型
    msg_types = {
        "pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
        "twist": "geometry_msgs/msg/TwistWithCovarianceStamped",
        "imu": "sensor_msgs/msg/Imu"
    }
    msg_classes = {k: get_message(v) for k, v in msg_types.items()}

    # 读取并排序
    all_data = {"pose": [], "twist": [], "imu": []}
    for key in all_data.keys():
        if key not in need_topics or need_topics[key] not in topics:
            continue
        tid = topics[need_topics[key]]
        for row in cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={tid}"):
            ts = row[0] / 1e9
            msg = deserialize_message(row[1], msg_classes[key])
            all_data[key].append((ts, msg))
        all_data[key].sort(key=lambda x: x[0])

    # 二分查找最近时间戳
    def find_closest(data_list, target_ts):
        if not data_list:
            return None
        timestamps = [ts for ts, _ in data_list]
        pos = bisect.bisect_left(timestamps, target_ts)
        if pos == 0:
            return data_list[0][1]
        if pos >= len(data_list):
            return data_list[-1][1]
        before_ts, before_msg = data_list[pos-1]
        after_ts, after_msg = data_list[pos]
        if abs(before_ts - target_ts) <= abs(after_ts - target_ts):
            return before_msg
        else:
            return after_msg

    # 输出 YAML（10 Hz）
    if not all_data["pose"]:
        print("❌ pose 数据为空，无法生成 YAML 文件")
        rclpy.shutdown()
        sys.exit(1)

    start_ts = all_data["pose"][0][0]
    end_ts = all_data["pose"][-1][0]
    freq = 10.0  # Hz
    dt = 1.0 / freq

    count = 0
    ts = start_ts
    while ts <= end_ts:
        pose_msg = find_closest(all_data["pose"], ts)
        twist_msg = find_closest(all_data["twist"], ts)
        imu_msg = find_closest(all_data["imu"], ts) if imu_exists else None

        # IMU 补零
        angularV = {"x":0.0, "y":0.0, "z":0.0}
        if imu_msg:
            angularV = {
                "x": float(imu_msg.angular_velocity.x),
                "y": float(imu_msg.angular_velocity.y),
                "z": float(imu_msg.angular_velocity.z)
            }

        # Pose
        orientation = pose_msg.pose.pose.orientation
        position = pose_msg.pose.pose.position

        # Twist
        vel = twist_msg.twist.twist.linear if twist_msg else None
        acc = twist_msg.twist.twist.angular if twist_msg else None

        # Covariance 转 float 列表
        posCov = [float(x) for x in pose_msg.pose.covariance] if hasattr(pose_msg.pose, "covariance") else [0.0]*36
        velCov = [float(x) for x in twist_msg.twist.covariance] if twist_msg and hasattr(twist_msg.twist, "covariance") else [0.0]*9

        entry = {
            "header": {"frameId": "vehicle-lidar", "timestampSec": ts},
            "pose": {
                "orientation": {"x": float(orientation.x), "y": float(orientation.y), "z": float(orientation.z), "w": float(orientation.w)},
                "position": {"x": float(position.x), "y": float(position.y), "z": float(position.z)}
            },
            "vel": {"x": float(vel.x) if vel else 0.0, "y": float(vel.y) if vel else 0.0, "z": float(vel.z) if vel else 0.0},
            "acc": {"x": float(acc.x) if acc else 0.0, "y": float(acc.y) if acc else 0.0, "z": float(acc.z) if acc else 0.0},
            "angularV": angularV,
            "consistencyToMap": 1.0,
            "mode": 1,
            "poseConfidence": 1.0,
            "status": 2,
            "worldFrame": "WGS84",
            "posCov": posCov,
            "velCov": velCov
        }

        with open(outdir / f"{int(ts*1e9)}.yaml", "w") as f:
            yaml.dump(entry, f, sort_keys=False)
        count += 1
        ts += dt

    print(f"✅ 已生成 {count} 个 yaml 文件，保存在 {outdir}/")
    rclpy.shutdown()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: ./123.py your.db3")
        sys.exit(1)
    main(sys.argv[1])
