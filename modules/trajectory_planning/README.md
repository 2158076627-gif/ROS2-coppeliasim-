# 模块二: 轨迹规划

目标：把视觉模块输出的装配目标位姿转换成机械臂可执行的阶段化装配轨迹。

## 正式代码

- `src/assembly_trajectory_planner`
  - `planner_core.py`: 独立于 ROS 2 的轨迹规划核心。
  - `trajectory_planner_node.py`: ROS 2 节点，订阅 `/perception/axis_hole_pose`。
  - `config/trajectory_planner_params.yaml`: 观测、预对准、接触搜索、插入和退出距离参数。

## 轨迹阶段

1. `observe`: 退到较远观察位，适合二次视觉确认。
2. `pre_align`: 到达孔轴上方的预对准位。
3. `contact_search`: 慢速接近接触区。
4. `insert`: 沿孔轴执行插入。
5. `hold`: 保持短暂停留。
6. `retract`: 退出到安全位。

## 归档原型

- `reference/trajectory_planning/coppeliasim_basic_motion_reference.py`
- `reference/trajectory_planning/coppeliasim_ur5_assembly_reference.py`

## 输出契约

- `/assembly/trajectory_plan`: `geometry_msgs/PoseArray`
- `/assembly/trajectory_path`: `nav_msgs/Path`
- `/assembly/trajectory_stage_names`: `std_msgs/String`

实际部署时可以用 MoveIt 2、ros2_control 或 CoppeliaSim bridge 消费这些轨迹。
