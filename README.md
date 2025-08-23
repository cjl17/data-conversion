# data-conversion

本仓库包含两个 Python 工具脚本，用于 **相机参数转换** 和 **外参矩阵计算**。  
主要功能：
- 将相机标定参数（YAML）转换为 JSON 格式，便于后续处理或与其他系统对接。
- 根据标定文件计算从 `base_link` 到指定相机坐标系的变换矩阵。

---

## 克隆仓库

```bash
git clone https://github.com/cjl17/data-conversion.git
cd data-conversion
脚本说明与用法
1. convert_to_tencent_format.py

功能：
将相机标定参数的 YAML 文件 转换为 JSON 文件，输出包含相机内参、畸变系数、外参和平移参数。

用法：

python convert_to_tencent_format.py <输入YAML文件路径> <输出JSON文件路径>


示例：

python convert_to_tencent_format.py camera_calib.yaml camera_calib.json


输出 JSON 格式示例：

{
    "extrinsic": [...],
    "intrinsic": [...],
    "distortion": [...],
    "translation": [...]
}

2. generate_extrinsics.py

功能：
读取标定文件，计算从 base_link 到指定相机坐标系的变换矩阵，并打印到终端（保留三位小数）。

默认文件路径写在脚本里：

sensors_calibration.yaml

sensor_kit_calibration.yaml

用法：

python generate_extrinsics.py <camera_name>


其中 <camera_name> 为相机编号，如 camera0、camera1、camera2 等。

示例：

python generate_extrinsics.py camera0


输出示例：

camera0 -> Camera Transformation Matrix:
['0.999', '0.010', '-0.030', '1.200']
['-0.010', '0.999', '0.005', '0.100']
['0.030', '-0.005', '0.999', '0.800']
['0.000', '0.000', '0.000', '1.000']
