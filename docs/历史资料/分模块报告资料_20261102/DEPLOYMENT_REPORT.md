# YOLOv8n-seg + RTAB-Map 简要部署报告

日期：2026-10-23。结论：**原来在 VMware 上使用的 Indoor rosbag 离线语义建图流程，已能在 Jetson 上独立运行，YOLO 使用 GPU。**

## 目录与版本

- Jetson：`tjark@192.168.10.102`，唯一项目根目录 **`/home/tjark/yolo_rtabmap`**。
- 已去掉原先的外层目录；源码、Conda 环境、权重、配置、bag、日志与地图都在该项目内。
- 本报告在 Jetson 为 `~/yolo_rtabmap/DEPLOYMENT_REPORT.md`，在 VMware 为 `~/ros2/DEPLOYMENT_REPORT.md`，内容一致。
- VMware 原始项目 `~/ros2/Semantic-SLAM` 保持不变；部署源码副本和证据备份在 `~/ros2/jetson_deploy_20261023/`。
- Jetson Orin NX / ARM64，Ubuntu22.04.5、ROS2 Humble、JetPack元包6.2.1、厂商L4T36.4.3、CUDA12.6；现有BSP与内核未升级。
- 原版COCO80 `yolov8n-seg.pt`；Ultralytics8.3.221，NVIDIA CUDA PyTorch2.5，torchvision0.20，Conda Python3.10；RTAB-Map0.23.13。

## 已通过的部署验收

| 项目 | 结果 |
|---|---|
| GPU实例分割 | CUDA、真实CUDA NMS和ROS模块通过；20张RGB的P50/P95为41.95/51.93ms，约22.96FPS，有真实非空Mask |
| Indoor几何回放 | 约120秒短测通过，保存几何地图、轨迹及54节点数据库 |
| Indoor语义回放 | 约120秒短测通过；处理159帧，保存16,961个语义体素，检测/投影程序错误0 |
| 新目录复验 | 仅补做一次15秒回放，通过；16帧推理、21个检测、7,896个语义体素，数据库检查通过，进程正常停止 |
| 实机输入接口 | D435的RGB/配准深度/内参通过；LakiBeam1的真实/scan约10.28Hz，退出修复后正常 |

离线版与之前VMware使用的流程一致：播放已有Indoor bag → RTAB-Map几何建图 → GPU实例分割 → 深度投影/TF变换 → 语义地图累积。它是**已有传感器数据的离线回放**，不需要接通真实相机、雷达，也不控制机器人运动；本次没有新增Gazebo仿真。
短测未验证完整路线回环或识别精度。COCO预测可能有类别误识别；真实相机画面为空检测时也不会冒充识别成功。

## 在 Jetson 上运行离线版

```bash
cd /home/tjark/yolo_rtabmap
bash run_offline.sh
```

默认：本地Indoor bag、原版YOLO、GPU、0.5倍速、120秒、无RViz，ROS域77且仅本机通信。无需连接VMware取得数据或环境。
提前停止并保存：另开Jetson终端执行以下命令，或在启动终端按Ctrl+C。

```bash
cd /home/tjark/yolo_rtabmap
bash stop.sh
```

只运行几何部分可用 `bash run_offline.sh --geometry-only`。默认保持无图形界面，减少图形资源占用；Jetson当前已存在rviz2。

## 文件在哪里查看

项目内：

- `runs/<时间戳>/`：每次独立输出，含`rtabmap.db`、`geometry_map.png`、`trajectory.csv`、`semantic_map.ply/.npz`和分割图片、JSON结果与日志。
- `runs/20261023-110028-5003/`：已完成的120秒语义回放；含`semantic_map_preview.png`。
- `runs/20261023-123557-7177/`：目录迁移后的15秒复验结果。
- `tests/gpu-20261023-105459/`：20张图的GPU推理、Mask和性能结果。
- `tests/d435_real_camera/`、`tests/lidar_live_verified/`：真实相机样图、雷达扫描图与验证记录。
- `logs/`：实际依赖、安装/编译、迁移记录。`DEPLOYMENT_STATUS.md`为详细部署状态。
- `ARTIFACT_SHA256SUMS`：当前代码、模型、地图和主要结果的完整性清单。

VMware证据副本位于 `~/ros2/jetson_deploy_20261023/local_audit/`，其中`runs/20261023-110028-5003/`含地图预览，`lidar_live_verified/scan_preview.png`为真实雷达图。

## 改动与交付边界

- 所有项目文件集中在`~/yolo_rtabmap`；目录迁移已修正Conda前缀、生成路径与符号链接，未重新安装GPU依赖。旧验收日志中的历史绝对路径原样保留，文件实际位置以本报告为准。
- 系统改动为批准的49个新增ROS/相机依赖包（含后续手推测试批准的rtabmap-odom）及D435专用权限规则；没有升级系统包、BSP、内核或JetPack。
- 未改团队比赛代码、原Semantic-SLAM、网络或开机启动项；未运行底盘/机械臂控制。测试进程均已停止。
- 离线部署已完成。后续实机测试已通过相机、雷达和GPU分割静态检查；底盘控制、里程计与安装TF尚待确认，未执行自动运动或开展训练。

环境由启动脚本自行加载。若在已加载ROS路径的终端管理Conda，使用`env -u PYTHONPATH -u PYTHONHOME miniforge3/bin/conda ...`，避免ROS的Python库干扰Conda管理程序。

## 实机静态检查更新（2026-10-23）

D435 RGB-D、LakiBeam1 `/scan`约10.01 Hz和GPU真实实例Mask通过；用户人工移动后画面已更新，最近一帧3个Mask、模型推理32.49 ms。当前未发现底盘控制接口、`/odom`或安装TF，因此未发运动指令，也未验收实车轨迹/地图。报告见 `STATIC_TEST_REPORT.md`，实拍与日志见 `runs/static-20261023-151928/`。VMware完整副本为 `~/ros2/live_robot_test_20261023/`。

## 手推与视频测试更新（2026-10-23）

手推短测已完成并停止，未发送底盘命令。GPU分割/深度投影可运行，但转弯后视觉里程计持续失跟踪，尚不能验收完整实车建图。原速相机+分割视频、建图过程视频、2倍速左右分屏视频及CPU/识别速度统计位于`runs/handpush-20261023-161242/`；简报见`HANDPUSH_TEST_REPORT.md`。现场无需继续推车。

## 最终视频（用户选择1612版）

最终选片为`handpush-20261023-161242/combined_2x.mp4`：剪去开头54.84秒，约22秒，左右分屏2倍速。VMware位置：`~/ros2/live_robot_test_20261023/handpush-20261023-161242/`；按用户要求最终剪辑仅在虚拟机交付，不再同步Jetson。详情见本地`HANDPUSH_TEST_REPORT.md`。所有现场采集已停止，无运动指令。
