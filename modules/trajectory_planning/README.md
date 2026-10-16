# 几何规划与数值执行

`assembly_trajectory_planner` 将目标转换为 observe、pre_align、contact_search、insert、hold、retract 六个笛卡尔路点，通过 PoseArray 与 Path 发布。它不执行 IK 或碰撞检查。

`axis_hole_bringup/demo_executor_node` 只执行固定姿态、同 XY、−Z 插入的模拟协议，在插入段应用柔顺速度；数值模型的实际位姿反馈用于推进阶段。PoseArray 仅传几何，执行速度与容差由演示执行协议固定，详见根目录 README。
