# 审查与验证记录（2026-10-16）

审查基线：`310f37e97fe0e538eed6ca3ea1a01468928b9454`。执行环境为 Ubuntu 24.04.3、Python 3.12.14，未安装 ROS2（无 `/opt/ros`、`ros2`、`rclpy`）。为验证打包安装了 colcon 及 pytest；没有将这件事表述为建立了 ROS2 环境。

## 根因与修复

| 发现 | 影响 | 修复 |
|---|---|---|
| bringup 缺 package.xml/setup.py/setup.cfg/resource 标记 | colcon 不能安装总启动包 | 补齐 ament_python 元数据与 launch/config 安装 |
| 两个现有包缺 resource 标记 | setup.py 引用的资源不存在 | 补齐三个包的标记文件 |
| setup.py 三个 console script 指向缺失模块 | ros2 run 无法导入 | 补齐规划、柔顺和模拟力节点 |
| 总启动依赖缺失的 fusion/stereo/demo_data 包 | launch 在包查找阶段失败 | 改为仅依赖三个实际存在的包 |
| 原力反馈与轨迹没有执行反馈通路 | 即使输出话题也不是运动闭环 | 添加工具位置积分、执行器与位置依赖力信号 |
| NaN/Inf 或极大有限力、无效接近轴 | 非法运动或范数溢出 | 有限值/轴校验、稳定 hypot、零输出/停止 |
| 新版 setuptools 忽略 tests_require | colcon test 选择 unittest，0 tests/exit 5 | 改为 extras_require[test] 并实际重跑 |
| README/模块说明列出未提供目录与能力 | 用户照文档无法复现 | 重写为当前真实代码和明确限制 |

修复前已运行失败回归：缺包/资源、缺入口、无效轴、非法力；独立审查发现的大有限力范数溢出也先复现失败后修复。

## 本地已执行

```bash
python -m pytest -q -rs
# 13 passed, 1 skipped, 3 subtests passed
# SKIPPED tests/test_ros_loop.py: ROS 2 is not installed/sourced

colcon list
# assembly_trajectory_planner (ros.ament_python)
# axis_hole_bringup (ros.ament_python)
# compliant_servo_control (ros.ament_python)

colcon build --symlink-install
# Summary: 3 packages finished

colcon test --event-handlers console_direct+ --return-code-on-test-failure
# Summary: 3 packages finished
colcon test-result --verbose
# Summary: 10 tests, 0 errors, 0 failures, 0 skipped

python -m compileall -q src tests
git diff --check
# exit 0
```

仓库级 13 项由三包的 10 项与额外的 3 项打包/启动契约测试组成；subtests 不是额外独立测试数量。被跳过的是整个 ROS2 测试模块，模块包含四项真实进程测试，不应解读为只跳过一项通信断言。

纯 Python 反馈测试确实反复执行“规划→控制→模型位置更新→新力反馈”，并断言接触出现、插入深度、完成回撤与零速度；它未运行 ROS 消息传输。colcon 构建验证 Python 包安装和资源部署，不能证明 ROS 依赖已安装或消息 API 已执行。

## 已在 GitHub CI 执行：真实 ROS2 Jazzy

[成功运行记录](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38857070443)：代码提交 `5f8e82e62e78ce2eb011738cc5b4af12ea72dbe6`，官方 `ros:jazzy-ros-base` 容器、Python 3.12.3、pytest 7.4.4。

```text
colcon build: 3 packages finished
colcon test-result: 10 tests, 0 errors, 0 failures, 0 skipped
python3 -m pytest -q -rs: 17 passed in 33.53s
```

四项真实进程测试均实际运行并通过，没有用 ROS 替身或跳过。这确认默认 launch、消息类型、默认参数解析、正常闭环、逆序节点启动、力源丢失、模型命令丢失、柔顺非法输入/超时及过载锁存。

前两轮 CI 没有通过，已定位并修复：

- [第一轮](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38856822412)：`self.executor` 覆盖 `rclpy.Node.executor`，执行器启动失败；改为 `self.follower`。
- [第二轮](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38856967541)：`self.guards` 与只读 `rclpy.Node.guards` 冲突；改为 `self.input_guards`。随后对六个节点成员名与官方 Jazzy Node API 进行交叉检查。

实际进程失败是本地纯 Python 检查未覆盖的边界，不能省略。第三轮通过的是修复后的代码；不改变本地没有 ROS2 的事实。

[文档更新后的复跑](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38857287264)出现 1 failed / 16 passed：闭环已完成，但 DDS 发现较慢使观察者漏收短暂的 observe/pre_align 状态。测试改为单独核对完整六阶段计划与可观测的接触/插入/保持/回撤/完成状态；纯核心测试明确断言六阶段的实际访问顺序。没有删掉接触、速度、位移或停止断言。

[下一次复跑](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38857544040)进一步发现，故障注入测试在已收到状态、尚未发现位姿发布者时就停止力源，访问未收到的 pose 导致 KeyError（1 failed / 16 passed）。修复为先同时观察实际位姿、非零速度及接触搜索/插入状态，再注入故障；停止断言等待带新时间戳的位姿反馈而非固定休眠。模型看门狗测试也先确认命令订阅者发现完成。此修复不改变被测控制节点。

## 最新数值闭环 CI 证据

[运行 38857876199](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38857876199) 已成功完成。日志检出的是 PR #1 合并预览 `d60682f`（head `97f47cc38a3485669805dcfdd232642bfedb11d0`，base `310f37e`），不是 CoppeliaSim 物理运行。

```text
colcon build: 3 packages finished
colcon test-result: 10 tests, 0 errors, 0 failures, 0 skipped
python3 -m pytest -q -rs: 17 passed in 35.72s
```

这是观察者发现时序修复后的最新代码证据；上文保留此前失败与成功的历史，不再以旧 run 代替此结果。仓库级 17 项包含包内 10 项，不能将两者相加成 27 项独立测试。PR #2 的新增桥接与引擎验证应单独记录在其分支的 `docs/contact_verification.md`。

## 本机复现 ROS2 运行验证

在已有 Jazzy 的干净终端，按 README 安装依赖、构建后：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
python3 -m pytest -v -m ros tests/test_ros_loop.py
```

四项进程测试分别验证：

1. 实际总 launch、话题类型、阶段推进、接触/速度变化、插入、回撤、最终零速度。
2. 独立柔顺节点输入超时、错误帧、过期时间戳、NaN、过载锁存。
3. 六个节点逆序启动后接通，单独停止力源后执行器归零并停止位置变化。
4. 单独模拟模型的命令丢失看门狗。

默认 ROS2 参数 YAML、DDS 消息传输和 rclpy 消息 API 已由上述 CI 覆盖。第一轮执行节点崩溃的日志也实际记录了总启动向其余五个节点传播 SIGINT 并关闭。RViz 显示、rosbag 录制、其他 ROS 发行版、其他 RMW/QoS 配置仍未验证。

## 明确不在验证范围

真实相机/激光感知、学习式异常分类、动态导纳、完整 SE(3) 执行、CoppeliaSim 真实接触、机器人 IK/碰撞检测、MoveIt/ros2_control 驱动、真机装配与迁移学习均未实现或验证。模型是沿固定姿态的三维平移积分，不会执行角速度；人工六维力消息不代表获得六自由度物理装配能力。
