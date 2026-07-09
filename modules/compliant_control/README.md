# 模块三: 力触觉多模态感知与阻抗伺服驱动的柔顺反馈控制

目标：根据力/力矩或触觉反馈，在插入过程中输出受限的伺服修正，降低卡滞、偏心和硬碰撞风险。

## 正式代码

- `src/compliant_servo_control`
  - `admittance_controller.py`: 导纳/阻抗式修正核心，独立于ROS 2便于测试。
  - `compliant_servo_node.py`: 订阅力/力矩，发布柔顺修正速度。
  - `mock_force_sensor_node.py`: 无硬件演示用的模拟力传感器。
  - `config/compliant_servo_params.yaml`: 力阈值、速度上限和增益。

## 控制逻辑

- 低轴向力：进入搜索状态，沿装配轴慢速接近。
- 横向力超过阈值：按导纳增益生成 X/Y 修正。
- 力矩超过阈值：生成姿态微调角速度。
- 总力过大：触发 emergency stop，输出零速度。

## 归档原型

- `reference/compliant_control/ur5_force_admittance_reference.py`
- `reference/compliant_control/vision_force_assembly_reference.py`

## 输出契约

- 输入：`/force_torque/wrench` (`geometry_msgs/WrenchStamped`)
- 输出：`/assembly/compliant_twist` (`geometry_msgs/TwistStamped`)
- 输出：`/assembly/contact_state` (`std_msgs/Bool`)

真实机械臂上建议把该输出接入 MoveIt Servo 或底层速度控制器，并在硬件层保留独立急停保护。
