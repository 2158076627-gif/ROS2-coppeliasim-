# CoppeliaSim 轴孔接触原型：部署与验证

基线：PR #1 的 `97f47cc`。新增 `coppeliasim_contact` 包，原三包的源码、配置和数值启动入口保持不变。
**本次未运行物理引擎。源码、离线测试和部署协议可审查，不代表接触仿真或真机验证通过。**

## 模型及能力边界

- 动态链：静态锚点 → X移动副 → 动态辅助质量 → Y移动副 → 动态辅助质量 → Z移动副 → 动态辅助质量 → 六维力传感器 → 动态圆柱轴。
- 电机速度控制，由物理引擎计算位移、约束力、碰撞和传感器力；不逐帧设置轴位姿，不用位置公式生成力。
- 孔是32段相互搭接的静态可响应凸长方体，内切半径5.2 mm；轴半径5 mm、长30 mm、质量0.1 kg。孔深20 mm，入口世界坐标(0.4,0,0.2) m；初始轴尖(0.4,0,0.23) m。
- 这是三移动副固定姿态试验台；没有六轴机器人、IK、关节空间控制、视觉、倒角、结构弹性或真实装配间隙标定。多边形孔也会引入离散化效应。
- 重力明确设为0，便于隔离接触与惯性；不是已完成重力补偿的机器人模型。腕力仍含轴的惯性/约束效应，不能把所有非零力当作接触力。
- `scene.json` 为可复现的场景源；需在 CoppeliaSim 上运行构建器才能得到 `.ttt`。不提供未经引擎生成的伪二进制场景。

## 依赖、构建、生成场景

目标：Ubuntu24.04/ROS2 Jazzy，CoppeliaSim4.7或之后保留下列API的版本（版本兼容尚待实测），官方ZeroMQ客户端2.0.4。ODE参数使用兼容的旧API，缺失API或参数读回不一致会报错，不能静默使用另一个引擎。

```bash
# 在仓库根目录；切到交付分支
 git switch feat/coppeliasim-contact
 source /opt/ros/jazzy/setup.bash
 rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
 python3 -m venv --system-site-packages .venv-contact
 touch .venv-contact/COLCON_IGNORE
 source .venv-contact/bin/activate
 python3 -m pip install coppeliasim-zmqremoteapi-client==2.0.4
 python3 -m colcon build --symlink-install
 source install/setup.bash
 export ROS_DOMAIN_ID=74
 export ROS_LOCALHOST_ONLY=1
```

**确保构建/运行都使用可导入该客户端的同一个Python环境。** 发行版ROS的Python ABI必须匹配；先运行 `python3 -c 'import rclpy, coppeliasim_zmqremoteapi_client'`。客户端不是rosdep依赖，不会强迫原数值演示安装模拟器依赖。

启动单独的CoppeliaSim进程（GUI更便于核查碰撞体），确保ZeroMQ插件/附加组件启用，默认 localhost:23000。新建空场景，删除该新场景中默认地面等形状，保留主脚本、相机和灯光。停止仿真。
构建器拒绝含有任何已有shape的场景，不清空或删除用户场景；构建失败时可能留下部分新对象，请重新新建空场景。

```bash
ros2 run coppeliasim_contact build_contact_scene \
  --config "$PWD/src/coppeliasim_contact/config/scene.json" \
  --output /tmp/contact_normal.ttt
```

`--output` 是**模拟器所在主机**的绝对路径，目录需已存在。远程客户端无法替模拟器主机创建目录。场景附带完整参数、模拟器版本、步长读回值。每次试验加载保存好的停止态场景；不要同时运行GUI自动播放、其他remote client或另一个步进控制器。

## 坐标、符号和控制周期

| 接口 | 类型与语义 |
|---|---|
| `/simulation/cmd_twist` | TwistStamped，robot_base，世界轴线速度；角速度必须为0 |
| `/simulation/tool_pose` | PoseStamped，robot_base→轴尖，m、xyzw；真实读回，姿态漂移则拒绝 |
| `/force_torque/wrench` | WrenchStamped，**世界轴且力矩参考robot_base原点**，N、N·m；供原柔顺节点使用 |
| `/simulation/raw_sensor_wrench` | WrenchStamped，wrist_sensor局部轴且参考传感器原点；保留原生符号 |
| `/simulation/contact_telemetry` | String JSON，仿真时间、姿态、传感器矩阵、原始六维力、轴尖参考力矩、接触对象/点/反力/法向、实际命令、停止锁存及场景参数 |
| `/simulation/bridge_stop` | Bool，桥接层故障锁存，独立于原控制器emergency_stop |

robot_base与世界原点、坐标轴重合；+Z向上，插入沿−Z。主wrench遵循frame_id所指原点；与人工力演示中自定义的力矩含义不同。原控制器的物理专用配置禁用旋转修正。
原始传感器时刻的3×4矩阵包含坐标与原点位置，可用于离线复核，未发布TF树。

变换：`F_world = sign * R * F_sensor`，`tau_at_P = sign * R * tau_sensor + (p_sensor - p_P) × F_world`。
telemetry中的`reaction_at_tip`以轴尖为P；主ROS力矩以世界原点为P。符号必须作用于力和力矩两者。

物理配置默认为 `reaction_sign: 0`，**不允许运动、不发布控制用力，只记录原始传感器反馈**。先进行下述低力符号校准，再把复制到本地的参数文件改为+1或−1；不得凭经验认定原生读数是环境对轴的反力。

校准方法：将场景motor_force_n设5 N，生成带挡块的场景；用引擎探针缓慢沿−Z接近挡块，查看原生Fz。环境反力应朝+Z，据此选sign。再在浅插入后朝+X压壁，确认变换后的Fx朝−X，且传感器→轴尖力矩移位方向符合右手系。保存两次原始日志和参数，之后再运行25/60 N实验。
探针使用的sign不影响范数停止，但在校准前其有符号输出只能标为候选约定。

物理专用柔顺配置 `lateral_gain=-0.0007`，使速度沿环境反力远离壁面；轴向仍沿−Z接近、达到12 N后减速。阈值是复用的启发式参数，未证明对本模型最优。7.5 N横向阈值可能使低载荷接触无纠偏；应记录并比较开环/闭环，不保证能解除卡滞。

外循环20 ms、ODE子步2 ms；单客户端同步step。ROS消息使用**墙钟**，保持原有超时/执行器协议，`use_sim_time=false`；仿真时间单独存储，不发布`/clock`。这不是硬实时控制或ROS仿真时钟闭环。RPC开销导致仿真速度慢于墙钟时必须检查telemetry；停顿超过0.5 s锁存。重置/跳步/第二步进者导致步长异常则退出。

指令检查：frame、源时间和接收时间、有限数、范数≤0.08 m/s、零角速度。任意非法新命令清掉旧命令；0.5 s无新命令则归零。边界沿用x∈[.3,.5]、y∈[−.1,.1]、z∈[.18,.4] m，合力≥45 N锁存停止；按离散步长检测，不能视作连续时间绝对边界。无有效传感器读数时归零且停止发布主wrench，让原执行器超时。

ZeroMQ socket超时3 s；通信失败进程退出，独立launch停止其他节点。正常退出设置零速度并stopSimulation。若失联无法发送停止，保持stepping的模拟器预期冻结在最后步；**不得手动解除stepping继续运行**。这些是实验保护，非工业机器人安全功能。

## 原柔顺控制闭环

先复制物理参数并完成符号校准：

```bash
cp src/coppeliasim_contact/config/physical.yaml /tmp/physical-calibrated.yaml
# 编辑 reaction_sign；不要改动原 demo.yaml
ros2 launch coppeliasim_contact contact.launch.py \
  params_file:=/tmp/physical-calibrated.yaml
```

此入口包含已知孔位发布器、原规划器、原执行器、物理专用配置下的原柔顺节点和新桥。**不启动mock_plant_node或mock_force_sensor_node。** 已知孔位是仿真先验，不是视觉识别。孔偏移实验保留规划名义位置，故意引入感知误差。请用独立ROS domain，不能同时运行原数值演示争用相同话题。

原执行器为单次任务，1 mm位置容差可能使4 mm目标插深提前结束。判断实际插深用工具位姿，不用“complete”代替几何验收。原接口只允许固定姿态，物理姿态漂移会使链路停止；尚需实测ODE刚性约束误差能否满足此契约。

## 五类基础实验与可观测条件

下表是干预与验收条件，**不是已经取得的结果，也不是训练标签**。

| case | 干预 | 必需观测与解释 |
|---|---|---|
| normal | 正间隙、无挡块、名义同轴 | 实际轴尖到目标深度；接触记录/力接近自由运动基线，无过载；轴向运动通过孔而非穿透孔壁 |
| offset | 孔相对名义轨迹+X偏0.4 mm，超过0.2 mm径向间隙 | 几何误差已知；核查孔口接触点和轴位移。可能撞入口，不能直接叫侧壁接触 |
| sidewall | 同轴先插2 mm，再沿+X以0.2 mm/s运动 | 接触点Z低于入口至少0.2 mm、侧壁对象和横向反力、深度保持；如果先撞孔口，本实验未满足条件 |
| jam | 插深3 mm处增加实体挡块，电机限25 N | 有接触、持续向下指令、至少0.5 s位移<0.05 mm且力≥12 N，只判为**阻挡/卡滞候选**；不是已复现两点摩擦楔紧 |
| overload | 同一挡块，电机限60 N，达到45 N软件阈值即终止 | 真实读回合力≥45 N、桥/原执行器锁存、后续零指令；若电机/引擎未产生阈值，报告“未触发” |

`contact_experiment`是**直接驱动物理滑台**的基准采集探针，不经原ROS柔顺节点。它可与ROS闭环结果比较，不能用探针通过代替ROS闭环通过。

每种case先生成配置，再在**新的停止态空场景**中构建；不要对已有场景重复构建。

```bash
ros2 run coppeliasim_contact contact_experiment configure \
  --config "$PWD/src/coppeliasim_contact/config/scene.json" \
  --case jam --output /tmp/contact-jam.json
ros2 run coppeliasim_contact build_contact_scene \
  --config /tmp/contact-jam.json --output /tmp/contact-jam.ttt
# sign仅示例，必须来自你的校准结果
ros2 run coppeliasim_contact contact_experiment run \
  --case jam --reaction-sign 1 --output /tmp/contact-jam-run01.jsonl
```

configure输出可自行编辑motor力限以做5 N校准；校准模式请使用下面的独立探针说明：常规run会严格要求case的25/60 N配置。低力校准使用 `--calibration` 保留5 N限值，例如在run命令末尾加 `--calibration`（同时须先编辑场景JSON，将motor_force_n设5，再重新构建）。日志独占创建，已有文件不覆盖。探针最多45 s仿真时间，传感器无效不运动，达到45 N或越界停止。统计含有效样本、接触、实际深度、入口以下接触、卡滞候选和过载；观测不到就记false，不能补造结果。

## 记录与数据集边界

```bash
ros2 bag record -o contact_run01 \
  /simulation/cmd_twist /simulation/tool_pose /force_torque/wrench \
  /simulation/raw_sensor_wrench /simulation/contact_telemetry /simulation/bridge_stop \
  /assembly/compliant_twist /assembly/contact_state /assembly/emergency_stop \
  /assembly/execution_state /perception/axis_hole_pose /assembly/trajectory_plan
```

停止录像后一起保存：bag、`.ttt`、场景JSON、校准参数YAML、git提交、模拟器版本/引擎、case、试次、open/closed-loop、种子/确定性设置、异常说明。bag和JSONL不提交仓库。
每条telemetry保留simulation_time_s与wall_stamp_ns以检查速率、延迟和对齐；raw_sensor_wrench不单独暗示已补偿重力或完成滤波标定。

先验证正负符号与无接触基线，再扫摩擦0.1/0.2/0.4、ODE子步1/2 ms、soft_erp0.1/0.2/0.4、soft_cfm1e−6/1e−5/1e−4，逐一改变变量。ERP/CFM是数值约束柔度参数，**不是已经标定的材料接触刚度**；组合接触规则、求解器和步长都会影响等效刚度。轴向压靠曲线需报告实际位移—力、穿透量、峰值与步长敏感性。

用接触对象/点和几何核查补充阈值，保留 uncertain/invalid 类。自由运动惯性峰值、rim碰撞、底部阻挡与真正侧壁/楔紧应分开。训练/验证/测试按零件几何、试次和摩擦设置隔离，不能随机切相邻帧后声称跨零件泛化。此版本没有训练模型，也没有真机迁移结果。

## 测试与待实测清单

```bash
python3 -m pytest -q -rs
python3 -m compileall -q src tests
colcon build --symlink-install
colcon test --return-code-on-test-failure
colcon test-result --verbose
# ROS环境会运行 test_contact_ros.py：真实DDS、注入传输，不计算物理接触
# 以下需先加载 NORMAL .ttt，仿真停止；不能与bridge同时运行
COPPELIA_LIVE=1 python3 -m pytest tests/test_contact_live.py -q
```

离线可测：孔几何/参数、坐标和力矩移位、时戳/旋转/NaN/速度约束、停止锁存、构建API调用序列、步进跳变、接触数据保留。记录式API替身只检查适配器调用，不证明API版本兼容或物理有效。

尚未实测：场景实际生成/保存/加载、ODE碰撞和传感器方向、动态链稳定性、物理ROS闭环、五类工况、超载后的实际停止距离、摩擦/柔度/周期敏感性、rosbag记录与回放、六轴机器人、真机实验。本次本地没有ROS2，真实ROS测试明确跳过；新提交CI结果需另行查看，不能继承PR #1的17项通过作为本分支通过证据。

官方API核对（2026-10-16）：
- https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm
- https://manual.coppeliarobotics.com/en/sim/simReadForceSensor.htm
- https://manual.coppeliarobotics.com/en/sim/simCreateForceSensor.htm
- https://manual.coppeliarobotics.com/en/sim/simCreateJoint.htm
- https://manual.coppeliarobotics.com/en/sim/simGetContactInfo.htm
- https://manual.coppeliarobotics.com/en/accessingSceneObjects.htm
- https://manual.coppeliarobotics.com/en/physicsEngineDifferences.htm
