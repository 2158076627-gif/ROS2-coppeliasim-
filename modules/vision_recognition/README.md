# 模拟孔位输入

当前只有 `axis_hole_bringup/mock_perception_node` 发布已知合成 `PoseStamped`，没有读取图像、推理 YOLO、拟合椭圆、处理点云或融合传感器。双目与激光包不在仓库中。

模拟目标为 `robot_base` 中 (0.4, 0, 0.2) m，固定单位姿态、10 Hz 系统时间戳。真实感知是未来替换点，须实现标定、算法和帧变换后独立验证。
