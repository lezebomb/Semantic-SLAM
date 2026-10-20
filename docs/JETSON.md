# Jetson 部署准备

先收集型号和软件版本，再选择依赖。Ubuntu22.04 本身不能确定 JetPack/CUDA/PyTorch 的兼容组合。本次没有操作或验证 Jetson 实机。

```bash
uname -m
cat /etc/os-release
cat /etc/nv_tegra_release
dpkg-query -W nvidia-jetpack 2>/dev/null
python3 --version
```

Jetson 通常是 aarch64；根目录 `setup.sh` 仅安装 x86 CPU 依赖，在 ARM 会提前退出。请保留实机现有环境；根据 [NVIDIA PyTorch for Jetson 安装说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)选择匹配 JetPack 的 NVIDIA torch，并安装与该 torch 匹配且包含 CUDA NMS 的 torchvision。NVIDIA 官方说明这些 wheel 与指定 JetPack 版本配套，不能直接搬用 x86 的 `torch==2.5.1+cpu`。

ROS2 Humble 这条路径需要 Ubuntu22.04/Python3.10。建议在独立环境准备依赖，并通过 `--system-site-packages` 复用系统 ROS；如果 NVIDIA torch 装在另一个虚拟环境中，该选项不会自动继承它，应使用那个已经验证的环境或把匹配 wheel 安装到新环境。仓库不自动下载或升级 JetPack、CUDA、torch、torchvision。

其余 Python 依赖参考：Ultralytics8.3.221、NumPy1.x、PyYAML、Pillow、OpenCV（至少支持 cv_bridge 所需转换）。实测 x86 精确版本见根 README；ARM 的 OpenCV 可能来自系统/NVIDIA，不应为了复刻 x86 版本覆盖实机包。

建议先在隔离环境中解决依赖，再安装 Ultralytics：

```bash
# 使用已经具有 ROS、匹配 torch/torchvision 的 Python 环境。
export SEMANTIC_PYTHON=/path/to/verified/environment/bin/python
"$SEMANTIC_PYTHON" -c 'import torch, torchvision, cv2, numpy, yaml, PIL, rclpy, cv_bridge; print(torch.__version__, torchvision.__version__, torch.cuda.is_available())'
# 确认其他依赖均已具备后，防止安装器替换 NVIDIA torch：
"$SEMANTIC_PYTHON" -m pip install --no-deps ultralytics==8.3.221
"$SEMANTIC_PYTHON" -m pip check
"$SEMANTIC_PYTHON" scripts/check_model.py --model mobilenet --device 0
```

`--no-deps` 并不安装缺失依赖；`pip check` 或自检失败时应先解决对应环境，不能跳过错误继续宣称 CUDA 可用。依赖的完整列表可查安装包元数据 `importlib.metadata.requires('ultralytics')`。Torchvision 导入成功也不代表 CUDA NMS 编译成功，自检会实际运行这个算子。

先运行 [DEPLOYMENT.md](DEPLOYMENT.md) 的原生 PyTorch 路径。TensorRT engine 与 JetPack/TensorRT/GPU 有关，本仓库没有上传或声称验证通用 `.engine`；后续确认实机后再决定是否导出、量化和测试。
