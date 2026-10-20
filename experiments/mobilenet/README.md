# MobileNetV3-Small + YOLOv8n-seg

2026-10-15 完成的单架构、低成本训练验证。MobileNet 为 torchvision 标准 Small/width1.0，移除分类尾部，提取 stride8/16/32 的24/48/96通道特征，经1×1卷积适配为64/128/256，保留原 SPPF、Neck、Segment Head。

骨干用 ImageNet 权重初始化870560参数；显式按模块类型、形状和连接迁移 YOLO 的2301920参数，33152个通道适配参数随机初始化。全部可训练层联合微调。完整证据见 `evidence/initialization.json`。

## 实际结果

A100-SXM4-40GB，Ultralytics8.3.221，AdamW/lr0=0.001、batch32、AMP、640输入、30轮、3937次参数更新，训练42.6分钟。8000张 train2017 训练；val2017 分为互斥的1000张选优验证和1000张测试，最终测试覆盖79类。使用官方非 crowd 多边形标签，经原版转换，多段多边形采用标准合并。

| 指标 | 官方 YOLOv8n-seg | 本次 MobileNet |
|---|---:|---:|
| Box mAP50 / mAP50-95 | 0.5275 / 0.3748 | 0.3298 / 0.2188 |
| Mask mAP50 / mAP50-95 | 0.4968 / 0.3101 | 0.3082 / 0.1780 |
| 参数量 | 3409968 | 3205632 |
| CPU 完整推理 P50 / P95 ms | 79.45 / 82.02 | 75.01 / 78.90 |
| CPU 完整推理 FPS | 12.65 | 13.68 |
| CPU 峰值 RSS MiB | 517.01 | 539.36 |

CPU 是 VMware 8vCPU/CoreUltra9-275HX，FP32、640×640、batch1、实际2线程、相同20张Indoor RGB，预热10次，3轮交替顺序、各模型独立进程。完整推理含预处理、真实NMS/Mask解码，排除文件读取。不是Jetson速度。两模型实际检测数可能不同。

原版完整COCO预训练与本次子集训练不对等，因此不能单凭此表比较骨干的精度上限。Indoor 上有柜体误作冰箱、门墙大面积误作微波炉的现象。60秒RTAB-Map短测处理77帧、25个检测、95444个投影点、12084体素、17个几何节点，只证明运行和数据流，不证明语义正确。

`evidence/` 包含完整8项精度、有效CPU测速、逐轮Loss/验证CSV、训练证明和短测统计。原始绝对机器路径已去除；不发布数据集、rosbag、账号信息或所有临时日志。

## 复现训练（部署不需要执行）

使用独立的可用CUDA训练环境，安装 Ultralytics8.3.221，保留与CUDA相配的torch/torchvision。以下命令在本目录执行；初始化过程会下载 torchvision 官方 ImageNet 权重。GPU 训练器遇到无CUDA会直接拒绝正式训练。

```bash
mkdir -p weights results images
cp ../../models/yolov8n-seg.pt weights/yolov8n-seg.pt
python initialize.py
python prepare_data.py --metadata-only
# 以下只下载清单中的10000张图片，不下载整个COCO图像包：
python prepare_data.py
python train.py --persist "$PWD/results/recovery" --epochs 30 --batch 32 --workers 4 --gpu-hours 3
python evaluate.py --device 0
```

`evaluate.py` 使用1000张独立测试图比较两模型。把需要额外定性检查的未标注RGB放在 `images/`，它们只用于效果图，不计算正式mAP。每轮的带优化器恢复点在 `--persist` 指定目录；未来中断恢复使用 `--resume /path/resume.pt`，沿用原始总训练预算。训练完成后的成品位于本目录 `weights/best.pt`；不能将初始化文件用作成品。

上述脚本保留原训练器和算法，补齐目录创建以便从干净仓库执行。本次整理没有再次训练。

## 来源

主要结构阶段参考 [DETR-SCI-Banana](https://github.com/zgqtql666/DETR-SCI-Banana/tree/e43584575724ece8b1f2a5b696fe415507c2a82c)，另核查过 [ultralytics_pro](https://github.com/Chriz122/ultralytics_pro/tree/6d2c05a2494071d1be8723e7f6dc20d849a73112)。实际使用 torchvision 标准实现及 Ultralytics 自带 TorchVision/Index 接入，未复制两套第三方框架。

基础依据：[MobileNetV3论文](https://arxiv.org/abs/1905.02244)、[torchvision Small权重](https://docs.pytorch.org/vision/0.20/models/generated/torchvision.models.mobilenet_v3_small.html)。这些依据支持骨干选择和预训练来源，不能保证组合网络在任意CPU/GPU上的优势。
