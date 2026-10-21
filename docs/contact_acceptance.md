# 五工况物理验收增量

本增量以 PR #2 的 `6e73d19f5788a4e2be4a82cc8e357473e91d62b9` 为基线。
复用 `scene.build_scene`、`experiments.case_config/assess`、`CoppeliaBackend`、
`MotionGuard` 及原 ROS 桥。未修改 main 的原三个数值包，也不解除 PR #2 的 Draft。

**验收对象仍是固定姿态的三移动副滑台，不是六轴机械臂。**
新增代码及合成数据测试通过，不等于 CoppeliaSim 接触行为通过；下述阈值是
待实测的验收参数，不是已测性能或工业安全指标。

## 交付与证据口径

| 文件 | 职责 |
|---|---|
| `acceptance.py` | 三态判定、接触对象/区域、有限几何重叠、连续卡滞时长、锁存/停止距离 |
| `acceptance_live.py` | 实际几何/材料/电机读回、子步接触、停止后采样、双轴符号/力矩原点校核、文件哈希 |
| `acceptance_cli.py` | 配置→构建/保存/重载→校准→采集→重放；独立 ROS 话题观察器；CSV/SVG |
| `test_contact_acceptance*.py` | 边界与失败回归；独立五工况 opt-in 引擎测试 |

`PASS`：必要判据均有合格证据。`FAIL`：观测到违反判据，或在完整有效试验中未激发
预定工况。`INSUFFICIENT`：原始数据、标定、对象、时间连续性或停止后日志不足。
已知的过大穿透、越界、解除锁存等失败优先于证据缺失，不能靠删日志消除失败。
没有原始日志时绝不能用单元测试的 PASS 填写物理结果。

## 判据和默认门槛

五工况均要求：`.ttt` 真正构建/保存/重载、配置与实际几何一致、ODE/版本/修订号/
步长/摩擦/ERP/CFM/质量/电机力限读回、有效的轴向及横向低力校准、逐帧原始力与
变换矩阵、完整接触列表、连续仿真时间、完整采集及文件哈希。校准绑定几何、
材料、步长和引擎版本；更改这些条件必须重新校准。

| 工况 | 通过 | 失败（完整且有效的观测） | 证据不足的例子 |
|---|---|---|---|
| normal | 实际插深≥3.8 mm；同轴路径无碰撞；合力≤5 N；无过载/锁存 | 未到深度、意外碰壁/过载、超过重叠门槛 | 无有效原始力、缺基线、遗漏接触记录 |
| offset | 复用孔偏 +X 0.4 mm；实际 peg–wall 入口接触点且反力 Fz≥0.5 N | 没有激发入口碰撞，或安全门槛失败 | 只有非零力，接触对象/点无法核对 |
| sidewall | 同轴先插约2 mm（±0.3 mm）；随后 +X；入口以下至少0.2 mm的内壁接触；Fx≤−0.5 N | 先撞入口，只有入口接触，或没有横向接触 | 法向/对象缺失，深度或横向命令未知 |
| jam | peg–physical_stop 接触；连续≥0.5 s向下实际命令；力≥12 N；整段三维位移最大变化≤0.05 mm；无过载 | 只短时冲击、接触/受力/命令中断、持续运动，或过载 | 时间断档、传感器失效、缺接触信息 |
| overload | 在实体挡块处实测合力≥45 N；当帧锁存、检测后立即零命令；此后保持锁存并阻断新非零请求；继续采样≥1 s；触发后最大三维位移≤0.2 mm；末0.5 s位移变化≤0.02 mm | 全程未触发、锁存解除、后续施加非零命令、停止位移超限 | 到阈值立即截断日志、缺后续挑战或位姿/命令记录 |

`jam` 的名称沿用现有干预：它证明实体阻挡/卡滞候选，**不证明摩擦两点楔紧**。
offset 的通过表示入口干预得到验证，不表示纠偏或插入成功。
直接探针的 sidewall 会主动横移；原有 `contact.launch.py` 不会自动生成这一横移阶段，
不能将普通同轴 ROS 演示充当五工况闭环验收。

`getContactInfo` 返回对象、接触点、接触力和法向，不返回求解器穿透深度。
本增量用读回确认的有限长方体及竖直圆柱计算逐采样几何重叠；默认上限0.1 mm。
这既不是用无限侧壁半空间外推，也不是已测接触刚度。
子步接触保留原始记录，最后子步用于与当前位姿核对；外周期内的瞬时穿透峰值
仍可能漏采。需进一步实现引擎侧子步位姿记录，才能给出子步峰值上限。
减小外步长和 ODE 步长的对照不能省略。

停止距离以首次达到45 N的**实际轴尖位置**为起点，取后续位置的最大三维距离，
包含反弹；另报上一个样本至触发样本的位移。它不等于从连续时间阈值交点开始的
停止距离，也不保证阈值之前的制动距离。全程保留指令前/后的时序。

## 软件与初始检查

目标仍是 Ubuntu24.04 / ROS2 Jazzy / 官方 ZeroMQ 客户端2.0.4 / 支持本仓库旧式ODE
参数接口的 CoppeliaSim。未实测的版本不得因名称相似而自动判为兼容。

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv-contact
touch .venv-contact/COLCON_IGNORE
source .venv-contact/bin/activate
python3 -m pip install pytest matplotlib coppeliasim-zmqremoteapi-client==2.0.4
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=74
export ROS_LOCALHOST_ONLY=1
ros2 run coppeliasim_contact contact_acceptance preflight --out /tmp/contact-environment.json
ros2 run coppeliasim_contact contact_acceptance prepare \
  --config "$PWD/src/coppeliasim_contact/config/scene.json" --out /tmp/contact-configs
```

所有输出目录独占新建，已有目录不覆盖。试验记录放在仓库之外。
live build/capture 要求代码工作区干净：先提交源码变更，再执行引擎试验，
避免只记下一个无法重建的 dirty diff 哈希。准备配置、离线单元测试和 report 不要求此条件。
无 ROS 的离线环境可用 `PYTHONPATH="$PWD/src/coppeliasim_contact" python3 -m coppeliasim_contact.acceptance_cli ...`；
引擎采集仍需 ZeroMQ 客户端及独立 CoppeliaSim。
模拟器与采集端必须能读取同一绝对路径的`.ttt`，例如同机或挂载同一目录。
使用一个专用 CoppeliaSim 实例：启动主脚本/ZeroMQ 服务，场景停止；不要与原数值
演示、另一个桥或 stepping 客户端共用它。构建器仍拒绝已有 shape，不会清空用户场景。

## 真实构建、保存和重载

每执行一次 build，先在 GUI 中新建停止态空场景并删除默认地面等 shape。
不要删除主脚本。build 会生成 `scene.ttt`，重新加载并核对实际几何、材料和电机。
任何 API 缺失、参数被引擎改写、质量/姿态错误均写入 `build.json`，退出码2。

```bash
ros2 run coppeliasim_contact contact_acceptance build \
  --config /tmp/contact-configs/calibration_axial.json --out /tmp/contact-build-axial
# 再新建停止态空场景，然后：
ros2 run coppeliasim_contact contact_acceptance build \
  --config /tmp/contact-configs/calibration_lateral.json --out /tmp/contact-build-lateral
```

同样逐个构建五工况，将输出设为 `/tmp/contact-suite/build/normal`、`offset`、
`sidewall`、`jam`、`overload`；五次都需要新的空场景。
capture 会从各自的已封存 build 包重载，重查当前引擎与对象；重载后 handle
可能变化，因此不会沿用构建时旧对象编号。

## 低力符号和力矩原点校核

校准采集限5 N，候选 sign 只影响记录中的变换方向；保护使用范数。
先采轴向挡块和横向壁面两次原始数据，再根据 +Z、−X 的已知反力方向选择 sign。
同一传感器的 sign 必须同时适用于两个方向以及力矩。

```bash
ros2 run coppeliasim_contact contact_acceptance capture --case jam \
  --build /tmp/contact-build-axial --calibration-trial axial \
  --candidate-sign 1 --out /tmp/contact-cal-axial
ros2 run coppeliasim_contact contact_acceptance capture --case sidewall \
  --build /tmp/contact-build-lateral --calibration-trial lateral \
  --candidate-sign 1 --out /tmp/contact-cal-lateral
# sign=1仅为命令示例；依据原始反力方向决定，不能预先当作已校准。
ros2 run coppeliasim_contact contact_acceptance calibrate \
  --axial /tmp/contact-cal-axial --lateral /tmp/contact-cal-lateral \
  --sign 1 --out /tmp/contact-suite/calibration
```

校核要求：自由运动基线≤1 N；两轴各有≥0.2 s连续、已核对的接触；反力方向
与施加运动相反；低力合力≤8 N；以接触力大小加权的接触点质心检验轴尖参考
力矩，残差≤0.01 N·m。保存完整矩阵、raw wrench、接触点和校核统计。
这只是符号/原点与低力基线校核，**不是可溯源的六维力传感器增益标定**；
复杂多接触、动态惯性或不平行接触下不能据此推定标定合格。
`load_calibration` 会用封存的两个原始日志重算，不能只填写一个 PASS 标志。

## 五工况直接探针

```bash
for case in normal offset sidewall jam overload; do
  ros2 run coppeliasim_contact contact_acceptance capture --case "$case" \
    --build "/tmp/contact-suite/build/$case" \
    --calibration /tmp/contact-suite/calibration \
    --out "/tmp/contact-suite/runs/$case-01"
done
```

每次最多45 s**仿真时间**，不是要求45 s墙钟内完成。默认25 N电机，overload 60 N。
达到过载后不再前进，但持续发起不会被执行的非零请求并采样1 s，检查锁存。
通信/姿态/传感器错误保留之前的原始行、异常类型与清理失败原因，不伪造补齐数据。
若需变更门槛，复制并在试验前冻结 `limits.json`，capture 使用 `--limits PATH`；
不能在看到结果后改阈值再报告同一次试验通过。

独立引擎测试入口（先完成上述构建和校准，不能与 ROS 桥同时运行）：

```bash
COPPELIA_ACCEPTANCE=1 COPPELIA_ACCEPTANCE_ROOT=/tmp/contact-suite \
  python3 -m pytest tests/test_contact_acceptance_live.py -q -rs \
  --basetemp=/tmp/contact-live-test-01
```

五个 case 独立生成原始文件与判定。未设置 opt-in 才跳过；设置后缺引擎、
缺场景/校准或未满足判据应失败，不能回退到替身后端。

## ROS 物理闭环单独记录

先复制 `physical.yaml`，在 `coppeliasim_bridge.ros__parameters` 下填写已测 sign、
`acceptance_build: /tmp/contact-suite/build/normal` 和
`acceptance_calibration: /tmp/contact-suite/calibration`。保留 `use_sim_time=false`。
新参数为空时原桥路径不变；启用验收模式时桥会加载封存场景并校核 sign。
`contact.launch.py` 仍是原规划/执行/柔顺节点的独立物理入口。

先启动独立话题观察器及 rosbag，再启动物理 launch：

```bash
ros2 run coppeliasim_contact contact_acceptance observe --case normal \
  --build /tmp/contact-suite/build/normal --calibration /tmp/contact-suite/calibration \
  --params /tmp/physical-calibrated.yaml --duration 90 \
  --out /tmp/contact-suite/ros-normal-01
```

```bash
ros2 bag record -o /tmp/contact-suite/rosbag-normal-01 \
  /simulation/cmd_twist /simulation/tool_pose /force_torque/wrench \
  /simulation/raw_sensor_wrench /simulation/contact_telemetry /simulation/bridge_stop \
  /assembly/compliant_twist /assembly/contact_state /assembly/emergency_stop \
  /assembly/execution_state /perception/axis_hole_pose /assembly/trajectory_plan
```

```bash
ros2 launch coppeliasim_contact contact.launch.py params_file:=/tmp/physical-calibrated.yaml
```

观察器只订阅，不驱动或步进引擎；记录所有话题的完整消息、消息自带源时间及
接收墙钟/单调钟。telemetry 保存独立仿真时间、原始力、当前接触对象、当前配置
及校准关联。不要启动直接 capture 与 ROS bridge 争夺同一个 stepping。
验收模式先等待 telemetry 订阅者被 DDS 发现，再启动步进，并强制最初0.2 s零运动。
日志必须从第一个引擎步开始，基线从至少5个实际静止无接触样本重算；
漏掉启动段或发生物理链异常跳变不能作为合格试验。只填写基线统计不够。

ROS 通过还需要全部12个话题被观察，原始数据和桥验收元数据一致，且对应物理
判据通过。只收到 DDS 消息不构成物理通过。overload 的原执行器可能自行停止：
若没有观察到锁存后的新非零请求，记录为缺少挑战证据；不要自动假设它已被阻断。
测试专用 domain 中可用独立测试驱动补充该阶段；不能同时运行两个执行器。
sidewall、卡滞的持续向下命令等若被原柔顺控制改变，应按实际观测判定失败/不足，
不能用直接探针结果代替。

bag 是独立旁证，观察器的 `topics.jsonl` 是自动判定使用的原始话题记录。
停止录包后以 `ros2 bag info /tmp/contact-suite/rosbag-normal-01` 检查时间范围/话题数量；
在另一个 domain 中回放，避免重放命令进入仍运行的仿真控制链。
保留 bag 目录及 info 输出与同名试次，不能把未录制/未回放写成已验证。

## 输出与重放

每个 capture/observe 包含 `manifest.json`、`samples.jsonl`、环境信息、场景 build
快照、完整校准原始日志快照及 `artifacts.sha256.json`；ROS 包另有 `topics.jsonl`
和所用参数快照。manifest 关联 run_id、case、直接/ROS模式、代码SHA、变更摘要哈希、
引擎版本/修订号、材料/步长/几何读回、校准 sign 和原始日志哈希。

```bash
ros2 run coppeliasim_contact contact_acceptance report /tmp/contact-suite/runs/jam-01
```

重放不连引擎：核对完整哈希清单、实际场景文件与配置/引擎对应、重新计算校准和判据。
ROS 模式还逐条重放 `topics.jsonl`，重算话题计数，并与 `samples.jsonl` 的 telemetry
及接收时间逐项比较；manifest 中手写计数不能代替原始消息。输出：

- `derived/report.json`：每项 PASS/FAIL/INSUFFICIENT、理由与数值；
- `derived/curves.csv`：仿真时间、实际插深/XYZ位移、六维反力、合力、逐采样重叠量、停止锁存；
- `derived/force_displacement.svg`：Matplotlib 绘制的轴向及横向位移–反力曲线，时间顺序连接，无拟合或平滑；
- `derived/stop_response.svg`：有停止锁存时绘制后续合力、最大三维位移及锁存时序。

matplotlib 为可选 report 依赖，缺失时仍输出 CSV，并写明图形未生成。

report 可重建派生文件，不覆盖原始记录。退出码：0通过，1失败，2证据不足。
校准 trial 的 `ACQUIRED_UNCALIBRATED` 只表示采集结束，不能作为校准或物理验收通过。
文件哈希能发现记录不一致，不是防伪签名，不能证明日志确由物理引擎生成。

## 当前实测边界与后续门槛

本环境无 ROS2 Jazzy / rclpy / CoppeliaSim。只执行独立 Python 单元/协议测试、
源码编译与差异检查。`test_contact_live.py` 仍只覆盖自由运动烟测；新的五项
`test_contact_acceptance_live.py` 都是待部署的真实引擎验收，不是已有实测结果。

待实测清单：实际 `.ttt` 构建/重载、ZeroMQ往返与独占step、ODE响应、双轴符号及
力矩原点、五工况原始日志、穿透/停止曲线、真实ROS物理闭环、bag录制/回放、
摩擦/ERP/CFM/外周期/ODE子步敏感性。每项分别留空，不由绿色CI补齐。
六轴IK/关节控制、真实力传感器和真机验证仍未实现。

官方 API 复核：2026-10-21。

- https://manual.coppeliarobotics.com/en/sim/simGetContactInfo.htm
- https://manual.coppeliarobotics.com/en/sim/simGetShapeGeomInfo.htm
- https://manual.coppeliarobotics.com/en/accessingSceneObjects.htm
- https://manual.coppeliarobotics.com/en/sim/simGetObject.htm
- https://github.com/CoppeliaRobotics/include/blob/master/simLib/simConst.h
