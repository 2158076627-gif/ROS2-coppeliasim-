# ROS 2 轴孔装配：数值闭环与实验性接触原型

本仓库目前提供 **固定姿态、三维平移的数值演示**：模拟孔位 → 笛卡尔路点 → 速度指令 → 工具位置积分 → 模拟六维力 → 接触判别与速度修正 → 下一步运动。它便于检查 ROS2 数据连通、记录话题以及连接后续算法。

本分支还提供独立可选的 `coppeliasim_contact`：参数化场景 JSON 与构建器、三移动副动态试验台、六维力传感器、ZeroMQ 桥和五类实验配置。**场景源码已交付；引擎生成的 `.ttt`、真实碰撞和物理闭环尚未验收。**

| 层次 | 已有内容与证据 | 能力边界 |
|---|---|---|
| 数值闭环 | 三包与六节点；PR #1 Jazzy 17 passed | 固定姿态平移积分、人工力，非接触物理 |
| ROS2 桥接接口 | 真实 DDS + 注入传输测试已通过；PR #2 总计 38 passed、1 skipped | 注入后端不计算物理，未证明 ZeroMQ 引擎通信 |
| CoppeliaSim 接触原型 | 第四包、场景构建源码、ODE 参数与实验协议 | 待场景生成、力符号、碰撞稳定性和五工况实测 |
| 六轴机器人/真机 | 尚未实现控制器适配、IK 或关节控制 | 3P 试验台及六维力消息均不代表六轴机器人控制能力 |

没有真实图像识别、双目/激光融合或神经网络异常识别。默认数值演示的六维力仍是人工反馈。物理入口不启动人工力源，使用待实测的引擎后端。

## 环境与运行

目标环境：Ubuntu 24.04 / ROS 2 Jazzy（或对应 WSL2 环境），系统 Python 3。本地与真实 Jazzy CI 的测试证据见下表。默认无界面，不需要相机、GPU、力传感器、MoveIt 或 CoppeliaSim。

在已安装 ROS 2 Jazzy 的终端中：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-pytest python3-rosdep
source /opt/ros/jazzy/setup.bash
# 如果从未初始化 rosdep，先执行 sudo rosdep init
rosdep update

git clone https://github.com/2158076627-gif/ROS2-coppeliasim-.git
cd ROS2-coppeliasim-
# 若审查尚未合并的提交，请先切到对应 PR 分支
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
ros2 launch axis_hole_bringup assembly_agent_demo.launch.py
```

成功的预期现象：`/assembly/execution_state` 从等待输入转入 `observe → pre_align → contact_search → insert → hold → retract → complete`；插入时工具 Z 低于 0.2 m，模拟 Fz 超过 12 N 时接触为真，轴向速度从 −0.001 降到 −0.00035 m/s；最后回撤至约 0.21 m 并持续发布零速度。完成后不会自动重复装配；重新启动全部节点开始新一轮。

另开终端，同样 source 两个 setup 文件后观察：

```bash
ros2 node list
ros2 topic info /force_torque/wrench --verbose
ros2 topic echo /assembly/execution_state
ros2 topic echo /assembly/contact_state
ros2 topic echo /assembly/compliant_twist
# 记录整个闭环；Ctrl+C 停止记录
ros2 bag record -o assembly_demo /perception/axis_hole_pose \
  /assembly/trajectory_plan /assembly/trajectory_path /simulation/tool_pose \
  /force_torque/wrench /assembly/compliant_twist /assembly/contact_state \
  /assembly/emergency_stop /simulation/cmd_twist /assembly/execution_state
```

可选 RViz：先安装 `ros-jazzy-rviz2`，再传 `use_rviz:=true`。需要图形桌面／WSLg，手动将 Fixed Frame 设为 `robot_base` 并添加 `/assembly/trajectory_path` 的 Path 显示；没有预置机器人模型与 RViz 配置。

## 包、节点与接口

当前分支有四个 `ament_python` 包；前三包构成默认数值闭环，第四包为独立实验性后端：

| 包 | 可执行节点 | 作用 |
|---|---|---|
| `axis_hole_bringup` | `mock_perception_node` | 10 Hz 发布已知模拟孔位 (0.4, 0, 0.2) m |
| `assembly_trajectory_planner` | `trajectory_planner_node` | 从孔位生成六个有序几何路点和可视化路径 |
| `axis_hole_bringup` | `mock_plant_node` | 50 Hz 积分模拟速度，输出工具位置 |
| `compliant_servo_control` | `mock_force_sensor_node` | 根据工具位置生成模拟力／矩 |
| `compliant_servo_control` | `compliant_servo_node` | 50 Hz 进行阈值接触判别、启发式速度修正和过载停止 |
| `axis_hole_bringup` | `demo_executor_node` | 等待输入、跟踪路点、插入段应用力反馈，输出仅供模拟器使用的速度 |
| `coppeliasim_contact` | `coppeliasim_bridge`、`build_contact_scene`、`contact_experiment` | 引擎桥、场景构建与五工况基准探针（待引擎实测） |

下表发布者对应默认数值入口；物理入口由桥替代数值模型与人工力源，新增原始力和接触遥测见[物理指南](docs/coppeliasim_contact.md)。

| 话题 | ROS2 类型 | 发布者 → 订阅者 |
|---|---|---|
| `/perception/axis_hole_pose` | `geometry_msgs/msg/PoseStamped` | 模拟感知 → 规划 |
| `/assembly/trajectory_plan` | `geometry_msgs/msg/PoseArray` | 规划 → 执行 |
| `/assembly/trajectory_path` | `nav_msgs/msg/Path` | 规划 → 可视化/记录 |
| `/assembly/trajectory_stage_names` | `std_msgs/msg/String` | 规划 → 记录（CSV 顺序） |
| `/simulation/tool_pose` | `geometry_msgs/msg/PoseStamped` | 模拟模型 → 模拟力、执行 |
| `/force_torque/wrench` | `geometry_msgs/msg/WrenchStamped` | 模拟力 → 柔顺、执行 |
| `/assembly/compliant_twist` | `geometry_msgs/msg/TwistStamped` | 柔顺 → 执行 |
| `/assembly/contact_state` | `std_msgs/msg/Bool` | 柔顺 → 观察/记录 |
| `/assembly/emergency_stop` | `std_msgs/msg/Bool` | 柔顺 → 执行 |
| `/simulation/cmd_twist` | `geometry_msgs/msg/TwistStamped` | 执行 → 模拟模型 |
| `/assembly/execution_state` | `std_msgs/msg/String` | 执行 → 观察/记录 |

默认数值入口全部带 Header 的消息使用 `robot_base`，位置 m、线速度 m/s、力 N、矩 N·m、角速度 rad/s，四元数为单位姿态。没有隐含 TF 转换。模拟力的正负号是“误差为正”的人工约定，不能直接套用真实力传感器的反作用力符号。

`PoseArray` 只包含几何路点，不编码速度、时间或阶段。演示执行器只接受固定姿态、同 XY、沿 −Z 插入的六阶段计划；自身采用 0.08 / 0.02 / 0.004 / 0.004 / 0 / 0.02 m/s 阶段速度、1 mm 到达容差及 0.5 s 保持。规划器核心的速度元数据没有通过 `PoseArray` 传输。变更这套协议需要同时修改执行器和测试。

## 参数、启动与停止

总启动加载 `axis_hole_bringup/config/demo.yaml`，可通过 `params_file:=/绝对路径/demo.yaml` 替换。它将观察/接近/接触距离设为 30/20/2 mm，插入深度 4 mm，回撤距离 10 mm。**演示配置区别于独立规划启动的原始 40 mm 插入深度；后者不可直接用于这个数值模型。** 柔顺默认参数定义在 `AdmittanceConfig`；独立力控启动的 YAML 使用相同默认值。

节点并行启动，不依赖固定延时；周期发布的输入使晚加入订阅者能够接入。执行器必须同时获得新鲜的计划、位姿、力和修正速度才运动；任一输入缺失、帧不符、数值非有限或超时则归零。默认超时 0.5 s，源时间戳与本地接收时间均检查，未来时间戳被拒绝。默认使用系统时间，不启动 `/clock`，不要将 `use_sim_time` 改为 true。

过载（合力 ≥45 N）停止在柔顺节点及执行器中锁存；恢复正常输入也不自动恢复，需重新启动演示。执行器还有独立 45 N 检查，因此提高柔顺阈值不会提高执行器的限制。计划中途改变或不符合执行协议时也锁存停止。任一必需节点退出，总启动结束其他节点；模拟模型有独立命令超时归零。以上仅为数值演示保护逻辑，不是机器人安全功能。

独立节点启动文件仍提供：

```bash
ros2 launch assembly_trajectory_planner trajectory_planner.launch.py
ros2 launch compliant_servo_control compliant_servo.launch.py use_mock_force:=false
```

独立规划只输出路点；独立模拟力节点需要 `/simulation/tool_pose`，不会在没有位姿源时伪造周期正弦力。总启动始终使用模拟数据，不提供直接切换真机的参数。

## 测试与已知验证范围

```bash
# 不需要 ROS2，需 pytest 与 setuptools
python3 -m pytest -q -rs

# 完整 ROS2：先构建并 source install/setup.bash，停止其他演示实例
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
python3 -m pytest -q -rs
```

`tests/test_ros_loop.py` 会真实启动总 launch，检查话题类型、六阶段、接触变化、速度变化、插入和回撤；另有独立柔顺进程测试检查丢输入、错误帧、过期数据、NaN 和过载锁存；逆序启动测试检查停止力源后执行器归零，模型测试检查命令丢失后位置停止。未安装／未 source ROS2 时整个模块明确跳过。测试使用专用 ROS domain，避免与其他演示进程混用。

| 2026-10-16 本次执行 | 结果 |
|---|---|
| 原缺包/缺入口回归检查 | 修复前失败，修复后通过 |
| 纯 Python 单元、打包契约、数值反馈测试 | 结果与命令见 [验证记录](docs/verification.md) |
| colcon 包发现与 Python 包构建 | 接触分支四包完成；仅构建不代表节点运行 |
| ROS2 进程、DDS、实际 launch/参数解析 | PR #1：17 passed；PR #2：38 passed、1 skipped，含原四项进程测试及注入后端桥接测试；本地无 ROS2 |
| CoppeliaSim 接触物理 / 真机 | 接触原型源码已提供，引擎测试未执行；六轴与真机控制未实现 |

CI 配置位于 `.github/workflows/ros2-demo.yml`。[Jazzy 实际通过记录](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38857876199)验证了代码提交 `97f47cc`：构建三包、包内 10 项测试，以及仓库级 17 项测试（无跳过）均通过。该结果针对默认配置和 CI 环境，不代表 RViz、rosbag、其他 ROS 发行版或真实机器人已验证。

[PR #2 Jazzy 记录](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38906535641)：head `c5aa35c`，合并预览 `450ae11`；四包构建、包内 10 项零失败；仓库级 **38 passed、1 skipped**，唯一跳过 `tests/test_contact_live.py`。包内 10 项已包含在仓库统计中，不相加。详见[接触验证记录](docs/contact_verification.md)。文档提交之后的 CI 应按各自 SHA 单独查看，不能把历史 run 写成新 head 的结果。

## 后续接入边界

真实视觉应另行实现标定、图像/点云算法和坐标变换，再替换模拟孔位源；这次没有补造 `stereo_hole_detector` 或 `axis_hole_fusion` 的空壳来冒充识别能力。本分支已提供 CoppeliaSim 场景构建和桥接源码，但引擎兼容、碰撞有效性、接触参数与动态链稳定性仍需实测；六轴机器人运动学尚未实现。真机还需要控制器适配、时间/坐标标定、力矩作用点变换、载荷补偿和设备保护。

`/simulation/cmd_twist` 在数值入口接数值模型，在物理入口接 3P 引擎桥；`/assembly/compliant_twist` 是启发式反馈输出，插入段含轴向搜索/插入速度，**不能不加处理地叠加到任意真实机械臂控制器上**。本次修复不验证舱段对接精度、装配成功率、训练性能或迁移效果。

## 独立物理入口与验收门槛

2026-10-21 五工况验收增量复用原场景/配置/判据，增加可重放原始证据、三态结论、
对象与接触位置、连续卡滞、过载锁存/停止距离及力–位移曲线。
见[验收命令与判据](docs/contact_acceptance.md)和[增量验证边界](docs/contact_acceptance_verification.md)。
单元测试和ROS接口CI不能替代物理引擎实测；PR #2继续保持Draft。

```bash
# 默认数值闭环：无需引擎
ros2 launch axis_hole_bringup assembly_agent_demo.launch.py
# 实验性物理闭环：先按指南生成场景并完成低力符号校准
ros2 launch coppeliasim_contact contact.launch.py params_file:=/tmp/physical-calibrated.yaml
```

两入口共用话题，须用不同 ROS_DOMAIN_ID 隔离，不能同时在同一 domain 启动。物理配置默认 `reaction_sign=0` 禁止运动；生成场景、ZeroMQ 依赖和校准步骤见[部署指南](docs/coppeliasim_contact.md)。探针是独立基准采集，不等于 ROS 柔顺闭环通过。

PR #2 最初堆叠在 PR #1 的 `fix/minimal-assembly-loop` 上。先以 merge commit 保留 PR #1 提交历史，再将 PR #2 的 base 改为 `main`；不要先把 PR #2 合入数值分支，也不要将 PR #1 squash/rebase 后又重复引入旧提交。PR #2 保持实验性、待引擎验收；合并前重新检查目标差异、当前 head 的 Jazzy CI、审查状态和[实测清单](docs/contact_verification.md)。
