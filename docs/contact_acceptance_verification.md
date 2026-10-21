# 五工况验收增量验证记录 — 2026-10-21

基线：PR #2 / `6e73d19f5788a4e2be4a82cc8e357473e91d62b9`。
GitHub API及CI原始日志已重新读取：PR #1 merged，PR #2 open + Draft；
基线 Jazzy run `38933607174` 成功，其合并预览 `b362e10333706d1562873c0b88c4e5a402914016`
是 head `6e73d19` 与 main `715114d` 的组合。日志确认为38 passed、1 skipped。
PR #1合并时刻是2026-10-16 16:38:14 UTC（北京时间10月17日00:38:14）。

## 当前执行环境

Ubuntu24.04.3 / Python3.12；无 `/opt/ros/jazzy`、rclpy、ros2命令或CoppeliaSim。
安装官方ZeroMQ客户端2.0.4仅为导入兼容检查；不代表服务连接或引擎行为成立。
安装colcon工具用于四个ament_python包的打包检查；colcon不等于ROS2运行时。

本增量本地执行：

```text
python -m pytest -q -rs
69 passed, 8 skipped, 4 packaging subtests passed

python -m colcon list
4 ros.ament_python packages

python -m colcon build --symlink-install
4 packages finished

python -m colcon test --return-code-on-test-failure
python -m colcon test-result --verbose
10 tests, 0 errors, 0 failures, 0 skipped

python -m compileall -q src tests
git diff --check
exit 0
```

包内10项已包含在仓库pytest统计内，不可再相加。4个subtest是安装目录检查，
不是额外物理实验。八项跳过为两个ROS模块、原自由运动引擎烟测一项、
新增五工况真实引擎验收五项。新增测试使用明确标注的合成数据与API替身；
不会把生成的单元测试曲线作为实验结果交付。

增量已进行独立代码审查并增加以下回归：连续接触/力/命令/位移窗口，
反弹停止距离，阈值后持续锁存，新非零请求被阻断，错误对象/接触位置，
外周期断档与子步接触，原始力/变换一致性，空/不完整哈希清单、场景错配、
校准重放来源、启动段与真实基线、校准标志绕过、原始ROS话题缺失、非法JSON结构。

原三个数值包相对 main 的源码/配置差异为空。原 `scene.py`、`backend.py`、
`contracts.py`、`experiments.py` 未改动。桥只增加可选验收入口和额外时序字段，
原数值入口及默认物理桥路径保持独立。

## 不可混用的验证层级

| 层级 | 当前新增证据 | 结论 |
|---|---|---|
| Python单元/协议、包构建 | 本地测试及构建记录 | 通过 |
| 真实ROS2 Jazzy接口 | 基线已通过；增量需查看独立草稿PR的head对应CI | 不用基线结果替代增量CI |
| CoppeliaSim物理引擎 | 当前无引擎，未生成实际五工况数据 | 未实测，全部证据不足 |
| 真实六轴机械臂 | 无驱动/硬件/测量 | 未验证 |

| 工况 | 实际引擎日志 | 当前判断 |
|---|---|---|
| normal | 无 | INSUFFICIENT |
| offset | 无 | INSUFFICIENT |
| sidewall | 无 | INSUFFICIENT |
| jam | 无 | INSUFFICIENT |
| overload | 无 | INSUFFICIENT |

待实测项：实际构建/保存/重载、ZeroMQ往返、ODE动态链、轴向/横向符号与力矩原点、
实际接触对象/点、位移–反力、逐采样及子步穿透、卡滞时长、停止锁存/位移、
ROS物理闭环、bag记录和回放、摩擦/ERP/CFM/周期敏感性。部署命令与逐项判据见
[contact_acceptance.md](contact_acceptance.md)。Draft PR #2不会自动转为ready或合并。
