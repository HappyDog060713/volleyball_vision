# 排球检测 YOLO 预训练模型说明

本目录下的模型来源于开源项目 [volleyball-tracking](https://github.com/jadidimohammad/volleyball-tracking)，基于 **VolleyVision 排球专用数据集**（约 17,000 张训练集，5,000 张验证集）完成训练。

---

## 1. 预训练权重清单与性能对比

| 模型权重文件 | 架构类型 | 文件大小 | 精确率 (Precision) | 召回率 (Recall) | 推荐场景 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`yolov8n.pt`** | YOLOv8 Nano | 5.4 MB | 92.5% | 81.4% | **【首选推荐】** 超低延迟，OpenCV DNN 极佳兼容，实机 150 FPS 首选 |
| **`yolo11n.pt`** | YOLO11 Nano | 5.3 MB | 92.8% | 80.4% | **【首选推荐】** 兼顾速度与精度，特征表达力强 |
| **`yolov9t.pt`** | YOLOv9 Tiny | 4.0 MB | 92.2% | 80.8% | 极致轻量化 |
| **`yolo11s.pt`** | YOLO11 Small| 19.0 MB| 93.0% | 85.2% | 高精度高召回（算力充足时选用） |
| **`yolov10n.pt`**| YOLOv10 Nano| 5.5 MB | 93.3% | 78.9% | 无 NMS 架构 |
| **`yolov10s.pt`**| YOLOv10 Small| 16.0 MB| 93.1% | 84.1% | 无 NMS 架构 |
| **`YOLOv12s.pt`**| YOLOv12 Small| 19.0 MB| 94.0% | 82.9% | 注意力机制架构 |

> 注：`yolov8s.pt` 原仓库文件大小为 2 字节（为空文件/占位符），已剔除。

---

## 2. 导出为 `volleyball_vision` 所需 ONNX 格式

`volleyball_vision` 节点通过 OpenCV DNN (`cv::dnn::readNetFromONNX`) 直接加载 ONNX 进行高速推理。

在具备 PyTorch 和 Ultralytics 环境的电脑上，运行以下命令即可完成导出：

```bash
# 进入当前模型目录
cd /home/aerialstewart/26_RC_Volleyball/src/volleyball_vision/models

# 一键导出 yolov8n 为 ONNX
yolo export model=yolov8n.pt format=onnx imgsz=640 opset=12 simplify=True

# 或一键导出 yolo11n 为 ONNX
yolo export model=yolo11n.pt format=onnx imgsz=640 opset=12 simplify=True
```

导出生成的 `yolov8n.onnx` 直接供节点加载使用。
