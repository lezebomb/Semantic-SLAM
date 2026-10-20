# 整理版本检查记录：2026-10-20

环境：现有 Ubuntu22.04 / ROS2 Humble / x86_64 CPU，PyTorch2.5.1+cpu、torchvision0.20.1+cpu、Ultralytics8.3.221。复用本机已有 Python/ROS 环境，没有重新安装依赖，没有重新训练。

| 检查 | 实际结果 |
|---|---|
| Python AST、Shell语法、YAML、git diff空白检查 | 通过 |
| MobileNet checkpoint | SHA256 与训练成品一致；CPU NMS 扩展可执行 |
| 640×640 零输入前向 | `[1,116,8400]`，原型`[1,32,160,160]`，张量有限；仅接口检查 |
| 实机 launch 参数 | 正确解析；路径相对于仓库根目录 |
| live入口 CPU启动/关闭 | 三个语义节点均ready，退出码0；未提供相机或TF，不作为实机运行证明 |
| ARM安装保护分支 | 以模拟aarch64架构标识检查，x86安装器在安装前退出；未冒充ARM硬件测试 |
| 空预测/严格验证/故障区分 | 3项单元测试通过；只构造计数元数据，不产生识别图或地图 |

20秒、0.5倍速官方Indoor bag无图形回归，仅执行一轮：

- 使用仓库内的真实训练权重，经 `run_mobilenet.sh` 启动，整体退出码0。
- 同步71帧，选择并处理23帧，20次检测，待处理队列覆盖0，推理失败0。
- 投影23组深度，累计58696个语义点，投影错误0；最终5956体素，TF丢弃1组。
- 12次实际点坐标TF重算，最大分量误差 `4.7683e-7 m`。
- 几何数据库2节点，SQLite quick_check=`ok`，没有节点关闭错误。
- 预测依然主要为 refrigerator、bottle；这些数量不是识别正确率。

本机详细回归日志保存在独立整理目录的 `runs/20261020-135331-4200/`，未上传大体积地图或临时日志。训练与之前60秒实验的精简证据在 `experiments/mobilenet/evidence/`。

原 `Semantic-SLAM` 工作目录保持干净；此次修改来自独立Git worktree。GitHub保留原基线提交 `cfb1d8faa84851c23a48c197fba6298d6e376de0`，便于追溯。

**尚未验证：Jetson/ARM实际安装、JetPack匹配、CUDA推理、真实相机/里程计/TF接入、Jetson延迟和内存。** 后续先确认实机型号及JetPack，再按 `JETSON.md` 和 `DEPLOYMENT.md` 逐项连接。
