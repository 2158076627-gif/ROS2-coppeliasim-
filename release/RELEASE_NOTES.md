# 2026-10-16 交付与验证边界

## 数值闭环基线（PR #1）

补齐三个包的 ament 元数据、资源标记与六个节点；总启动不再依赖缺失的感知包。添加位置反馈、人工六维力、单次执行、超时归零、过载停止锁存和测试。先前发布说明中的“完整感知包”“归档 reference 原型”“mock 图像/点云资产”在审查基线中没有相应文件，已撤回这些能力描述。

代码 `97f47cc` 的 Jazzy CI 为 17 passed；文档更新 `f451ccf` 的[复跑 38933253801](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38933253801) 同样为 17 passed。PR #1 已以 merge commit `715114d` 合入 main，保留共享祖先。证据与历史见 [verification.md](../docs/verification.md)。

## 实验性接触原型（PR #2，尚未发布为已验证物理系统）

新增第四个可选包 `coppeliasim_contact`，交付参数化场景 JSON/构建器、3P 动态试验台、六维力传感器、ZeroMQ 桥、独立启动和五类实验配置。场景源码已提供；实际 .ttt 生成/加载与引擎接触结果尚未验收，不提供六轴机器人或真机接口。

代码 `c5aa35c` 的[Jazzy CI 38906535641](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38906535641) 为 **38 passed、1 skipped**，唯一跳过真实引擎测试。新桥接测试使用真实 DDS 与注入传输，不计算接触物理。包内 10 项已包含在仓库统计中，不相加；后续文档 SHA 应检查各自 CI，不能继承历史结果。

PR #2 已改向 main，保持 Draft，等待 [contact_verification.md](../docs/contact_verification.md) 中的引擎、符号校准、五工况、物理 ROS 闭环和记录回放证据。默认数值入口与实验性物理入口保持独立，详见 [README](../README.md) 与[部署指南](../docs/coppeliasim_contact.md)。两入口共用话题，使用不同 ROS domain 隔离。
