# 模型文件

`manifest.json` 保存版本、文件大小、SHA256 和用途。先执行 `scripts/check_model.py` 再部署。

| 模型 | 获取方式 | 使用定位 |
|---|---|---|
| `models/yolov8n-seg.pt` | x86 `setup.sh` 从 Ultralytics 官方下载并校验；Jetson 按下文下载 | 默认基线 |
| `checkpoints/mobilenetv3-small-yolov8n-seg.pt` | 本仓库随附，6.41 MiB | 2026-10-15 真实训练的实验模型 |

MobileNet 权重来自本项目的 `best.pt`，保持原始字节；不是初始化权重。80 类输出，标准 Ultralytics 8.3.221 + torchvision 可直接加载。结构定义在 `config/models/`，训练代码及证据在 `experiments/mobilenet/`。

Jetson 安装好匹配的 NVIDIA torch/torchvision 和 Ultralytics 后，可单独获取基线，无需执行 x86 安装脚本：

```bash
mkdir -p models
curl -fL https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n-seg.pt -o models/yolov8n-seg.pt.part
echo 'a7cd8f929e1903d78a12a48efecab430209f18dc46cb96c3599a5980c63c423c  models/yolov8n-seg.pt.part' | sha256sum -c -
mv models/yolov8n-seg.pt.part models/yolov8n-seg.pt
```

本次 MobileNet 在 COCO 子集上的 Mask mAP50-95 为 0.1780，原版为 0.3101；训练数据/预算不等。室内存在门墙误判为微波炉等问题，语义点云非空只证明管线运行。默认部署仍使用基线；MobileNet 用于对比研究。Jetson 性能和 CUDA 路径尚未经过实机验证。
