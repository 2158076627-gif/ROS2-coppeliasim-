# 模块一: 视觉识别

目标：从双目图像、激光点云和标定信息中稳定输出机械臂可用的装配目标位姿。

## 正式代码

- `src/stereo_hole_detector`: YOLO/Mock 孔洞检测、CLAHE增强、Canny轮廓、椭圆拟合。
- `src/laser_pointcloud_processor`: 点云降采样、离群点滤波、RANSAC平面、边缘提取、3D圆拟合。
- `src/axis_hole_fusion`: 2D ROI 与 3D 点云融合，输出 `PoseStamped`。
- `src/axis_hole_interfaces`: `Hole2D`, `Hole2DArray`, `AxisHole3D` 消息。
- `src/calibration_manager`: 双目标定、手眼标定、激光-相机外参框架。

## 归档原型

- `reference/vision_recognition/peg_in_hole_visual_alignment_prototype.py`
- `reference/vision_recognition/camera_hand_eye_calibration_reference.py`
- `reference/vision_recognition/gr_cnn_grasp_reference.py`
- `reference/vision_recognition/openmv_eye_to_hand_sorting.py`
- `reference/vision_recognition/arduino_color_sorting_reference.py`

## 输出契约

视觉模块的主输出是：

- `/perception/holes_2d`
- `/perception/axis_hole_3d_fused`
- `/perception/axis_hole_pose`

下游轨迹规划只依赖 `/perception/axis_hole_pose`，因此后续接入真实YOLO模型或真实传感器时，不需要改动轨迹和力控模块。
