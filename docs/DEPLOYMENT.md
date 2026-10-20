# 实机接入

当前交付包括已验证的官方 Indoor 回放管线，以及接入实机已有几何 SLAM 的语义入口。实机优先考虑 Jetson；具体型号、JetPack、相机和里程计尚待确认，因此尚未宣称实机部署成功。

## 安装和模型自检

Jetson 先读 [JETSON.md](JETSON.md)。x86_64 Ubuntu22.04/Humble 使用根 README 的 `./setup.sh`。仓库可放任意目录；RTAB-Map 工作空间通过 `RTABMAP_WS` 指定。

以下从仓库根目录执行；`SEMANTIC_PYTHON` 指向同时能 import ROS、torch、torchvision、Ultralytics 的 Python3.10 环境：

```bash
export RTABMAP_WS="$HOME/ros2"
export SEMANTIC_PYTHON="${SEMANTIC_PYTHON:-$PWD/.venv/bin/python}"
"$SEMANTIC_PYTHON" scripts/check_model.py --model mobilenet --device cpu
# Jetson 的 CUDA/NMS 检查，匹配环境就绪后执行：
"$SEMANTIC_PYTHON" scripts/check_model.py --model mobilenet --device 0
```

自检只校验依赖、模型哈希、实际 NMS 算子和零输入输出形状，不表示识别精度通过。

## 相机、几何建图和 TF

先通过机器人的现有 bringup 启动相机、里程计/TF 和 RTAB-Map 几何建图。`run_live.sh` 接入这些数据，启动分割、投影、语义累积和 RViz。几何栈继续由 RTAB-Map 维护；本次整理没有为未知硬件编写相机驱动或假定里程计来源。

| 接口 | 要求 |
|---|---|
| RGB | 解码后的 `sensor_msgs/Image`，RGB/BGR 可由 cv_bridge 转换 |
| Depth | 已配准到 RGB、同尺寸同光学 frame；`16UC1` 毫米或 `32FC1` 米 |
| CameraInfo | 对应已校正 RGB；fx/fy>0，畸变系数为零 |
| TF | 采集时间可查询 `map → odom → base_link → camera_optical_frame`，具体 base/frame 名以机器人为准 |
| 时钟 | 实时相机用系统时钟 `use_sim_time=false`；回放时各节点统一仿真时钟 |
| QoS | live 默认 `sensor_data`/Best Effort；必要时改 `config/live.yaml` |

相机外参使用实测标定，不能用任意零变换补齐 TF。相机、底盘和 Jetson 应使用一致时间基准。零检测会照常发布空类别/实例 mask；深度不配准、时间戳或 TF 错误则是接口故障。

## 启动

先编辑 `config/live.yaml`。CPU 是默认路径；Jetson CUDA 自检通过后，把 `device` 改成字符串 `"0"`。以下 topic 仅为示例，需要用 `ros2 topic list` 和 `ros2 topic info -v` 核实：

```bash
./run_live.sh \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info

# 使用本次训练的实验权重：
./run_live.sh model_path:="$PWD/checkpoints/mobilenetv3-small-yolov8n-seg.pt"

# 无桌面环境：
./run_live.sh rviz:=false
```

参数 `params_file:=/path/live.yaml` 可指定机器专用配置；`output_dir:=/path/new-run` 必须是新目录，避免覆盖地图。默认生成 `runs/live-时间/`。跨机器 RViz/传感器运行时，各终端设置相同 `ROS_DOMAIN_ID`；`ROS_LOCALHOST_ONLY=0`，再检查机器人网络。live 入口不强制 localhost 或软件渲染。

RViz 默认配置 `config/semantic_indoor.rviz`：Fixed Frame=`map`，`/semantic/map` 选 Reliable + Transient Local、RGB8。几何显示的 `/map`、`/mapData` 等 topic 需要与实机 RTAB-Map 命名空间对应。语义模块每10秒保存，Ctrl+C 正常退出时再次保存 `semantic_map.ply/.npz` 和各节点统计。

## 验收顺序

1. 模型哈希和 CPU/CUDA 算子自检通过。
2. 单独确认几何地图和采集时间 TF 正常。
3. 观察 `/semantic/detector_status` 的处理帧、推理时间和检测数；检测为空可以是正常输出。
4. 有真实预测时检查 `/semantic/points_camera`、`/semantic/pointcloud`、`/semantic/map`；结合原 RGB 判断类别和 Mask 是否合理。
5. 保存运行日志、配置和统计，再在实机同条件测量性能；VMware 数据不可作为 Jetson 性能结论。

## 已知边界

实例 ID 仅表示一次观测；尚无对象跟踪、回环后语义重投影或动态物体清理。MobileNet 的室内误检会累积到地图中，不能把有色点云当作精度验证。新的 live 入口仅在当前 x86 ROS 环境进行接口检查，真实相机和 Jetson CUDA 待后续实机验证。
