数据处理与ROS2工具脚本集

本项目包含五个常用的 Python 脚本，帮助完成相机标定数据处理、坐标变换计算、文件批量重命名以及 ROS2 实时定位数据的记录与导出。
适用于自动驾驶、传感器标定、数据采集等场景。

目录结构示例：

project/
 ├─ README.md
 ├─ generate_json_from_yaml.py
 ├─ yaml_to_json.py
 ├─ transform_matrix.py
 ├─ rename_files.py
 └─ localization_csv_exporter.py

统一环境要求

Python：3.8 及以上

通用依赖：

pip install numpy pyyaml


第五个脚本需 ROS2：推荐 ROS2 Humble 或兼容版本（需已安装 rclpy 与相关消息包）。

1️⃣ generate_json_from_yaml.py

功能
从相机标定的 YAML 文件中提取：

相机内参矩阵（intrinsic）

畸变系数（distortion）

外参（extrinsic，可选）

平移参数（translation，可选）
并保存为格式化 JSON 文件，方便后续校准或应用。

命令示例

python3 generate_json_from_yaml.py input.yaml output.json


参数

input.yaml：相机标定文件。

output.json：输出文件路径。

输出结构

{
  "extrinsic": [...],
  "intrinsic": [...],
  "distortion": [...],
  "translation": [...]
}


注意事项

如果 YAML 中缺少外参或平移参数，会自动写入 null。

使用 yaml.safe_load 保障安全。

2️⃣ yaml_to_json.py

功能
通用的 YAML → JSON 转换工具。
保留所有原始键值对，仅改变文件格式。

用法

python3 yaml_to_json.py input.yaml output.json


适用场景

配置文件格式转换。

数据管道中需要统一 JSON 格式时。

3️⃣ transform_matrix.py

功能
计算相机到车辆基准坐标系的 4×4 齐次变换矩阵，并写入指定 JSON 文件。

工作流程

从 sensors_calibration.yaml 中读取 base_link → sensor_kit_base_link 的平移和旋转。

从 sensor_kit_calibration.yaml 中读取 sensor_kit_base_link → cameraX/camera_link 的平移和旋转。

合成 base_link → camera_link 变换并求逆，得到 camera_link → base_link 矩阵。

命令示例

python3 transform_matrix.py camera6 output.json


camera6：可替换为 camera0、camera1 等相机编号。

output.json：结果保存路径。

输出示例

camera6 -> Camera to Base Transformation Matrix:
['1.000', '0.000', '0.000', '0.123']
['0.000', '1.000', '0.000', '-0.456']
['0.000', '0.000', '1.000', '0.789']
['0.000', '0.000', '0.000', '1.000']


JSON 文件中：

{
  "extrinsic": [16个浮点数…],
  "translation": [x, y, z],
  ...
}


注意事项

确保 sensors_calibration.yaml 与 sensor_kit_calibration.yaml 路径正确。

角度单位：弧度。

4️⃣ rename_files.py

功能
按顺序批量重命名指定文件夹中的所有文件：

默认起始编号：175636611607

默认步长：10

后缀统一为 .jpg
执行后文件示例：
175636611607.jpg, 175636611617.jpg, 175636611627.jpg, ...

命令示例

python3 rename_files.py /path/to/folder


脚本参数修改
在代码顶部可直接更改：

start_index = 175636611607  # 起始编号
step = 10                   # 递增步长


执行流程

临时重命名：将每个文件改名为 原名_tmp.ext，避免覆盖冲突。

正式重命名：按顺序改为 <编号>.jpg。

注意

强烈建议先备份文件。

脚本对文件字典序排序后重命名，包含子目录会报错。

5️⃣ localization_csv_exporter.py

功能
ROS2 节点：订阅 GNSS 和车辆速度信息，实时记录到 CSV 和日志文件中。

订阅话题

话题	消息类型	说明
/sensing/gnss/fix	sensor_msgs/NavSatFix	GPS经纬度与高度
/localization/twist_estimator/twist_with_covariance	geometry_msgs/TwistWithCovarianceStamped	线速度与角速度

服务接口

服务名称	类型	功能
/localization_exporter/set_enable_save	std_srvs/SetBool	开启/关闭数据保存

CSV 格式

timestamp_ms, lat, lon, alt, vx, vy, vz, wx, wy, wz


v* 为线速度 (m/s)

w* 为角速度 (rad/s)

运行示例

# 启动节点（可带输出目录参数）
python3 localization_csv_exporter.py /tmp/localization_data
# 或在 ROS2 包中
ros2 run your_pkg localization_csv_exporter.py /tmp/localization_data

# 启用保存
ros2 service call /localization_exporter/set_enable_save std_srvs/srv/SetBool "{data: true}"


输出文件

localization.csv：实时数据。

localization.log：记录状态与统计。

注意事项

需在有 ROS2 话题发布的环境下运行。

每 1000 条数据自动在日志中输出一次进度。

退出时自动关闭文件句柄并写入日志。

开发建议

所有脚本遵循 PEP8 规范，易于扩展。

建议为不同任务建立独立的虚拟环境，避免依赖冲突。
