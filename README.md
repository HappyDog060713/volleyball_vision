# volleyball_vision

基于迈德威视长基线双目工业相机的排球高速视觉感知功能包 (ROS 2)。

## 功能特性
- **双目工业相机硬触发支持**：封装迈德威视 Linux C++ SDK，支持外部 GPIO Line 0 硬件同步脉冲触发（微秒级对齐），支持短曝光时间（<2ms）与增益调节。
- **极线校正查找表 (LUT)**：基于标定参数预先计算 OpenCV Remap 查找表，极速完成未畸变与极线水平校正。
- **YOLO 目标检测**：支持加载排球轻量级 ONNX 模型（在 AMD Ryzen AI / Radeon 880M iGPU 上高效运行）。
- **极线几何约束三角测距**：对左右视图检出的排球候选框进行垂直极线误差校验（$|v_L - v_R| < \epsilon$）与正视差校验，通过几何相交计算 3D 空间坐标 $(X, Y, Z)$。

## 通信接口
- **发布话题**:
  - `/volleyball/position` (`geometry_msgs/msg/PointStamped`): 原始空间三维坐标。
  - `/volleyball/debug_image` (`sensor_msgs/msg/Image`): 调试图（可选使能）。

## 编译与启动
```bash
colcon build --packages-select volleyball_vision
source install/setup.bash
ros2 launch volleyball_vision vision.launch.py
```
