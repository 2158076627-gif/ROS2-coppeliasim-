# 当前模块与历史描述的对应

审查基线：`main` 提交 `310f37e97fe0e538eed6ca3ea1a01468928b9454`。

原先文档提到的 `claude_code/`、`pycharm-code/`、`Github-code/`、`reference/`、`stereo_hole_detector`、`laser_pointcloud_processor`、`axis_hole_fusion`、`axis_hole_interfaces`、`demo_data`、`calibration_manager` 未包含在该基线。它们不能作为本仓库中已提供、可运行或已溯源的源码。

| 当前源码 | 实现范围 | 后续需要补充 |
|---|---|---|
| `src/axis_hole_bringup` | 已知模拟孔位、数值位置积分、单次装配执行与总启动 | 图像/点云真实感知、物理仿真桥 |
| `src/assembly_trajectory_planner` | 六阶段笛卡尔几何路点、PoseArray/Path 输出 | IK、碰撞检查、控制器轨迹时间参数化 |
| `src/compliant_servo_control` | 人工力信号、阈值接触、启发式速度反馈 | 标定、真实六维力变换、动态导纳模型、学习式异常识别 |

消息类型、坐标和运行限制以根目录 README 为准；此修复不声明恢复了历史原型或视觉算法。
