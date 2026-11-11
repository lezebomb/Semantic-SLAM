# 本机主工程

- 项目固定为RTAB-Map + YOLOv8n-seg；几何建图必须实际运行RTAB-Map，不用Cartographer/GTSAM或其他小组map位姿替代。
- 此目录为本机唯一维护的本组源码。2026-11-05将车端已取回的核心、入口和配置归并到现有工程；不得再创建日期版、organized或release平行工程。
- 本机使用已有.venv，Jetson使用其已有ARM64环境。scripts/ros_env.sh按架构加载，scripts/ros_env_jetson.sh为车端环境代码；没有在本机安装ARM64包。
- 当前为RGB-D手推基线，run.sh调用scripts/run_live.py统一管理RTAB-Map视觉里程计、几何建图、YOLO及记录，stop.sh统一保存退出。默认不使用轮速或雷达，不启动手柄，不发布任何底盘控制指令。车端只用run.sh、stop.sh两个根目录shell入口。
- 不把官方数据回放、离线回放、实车静态和动态测试混写。11月5日运动时未运行RTAB-Map，不属于本路线实车验收。
- 回放统一用run.sh --replay，只播放RGB、对齐Depth、CameraInfo与相机内部静态TF。使用非零Domain、本机通信；旧轮速、动态TF、语义输出和控制命令不在回放白名单。
- RTAB-Map与语义节点只使用/rtabmap_yolo/tf及tf_static。runtime仅导入相机内部静态外参；camera_odom→camera_link由rgbd_odometry发布，map→camera_odom由rtabmap发布，不再转发轮式TF。
- 11月6日历史轮速测试有人工驾驶记录，但地图质量未通过。当前RGB-D恢复代码已通过9项本机检查、CPU软件检查及隔离短回放，尚未同步到车端或进行新的实车复测。不得将本机通过写成车端通过。未经用户明确通知不启动新的实车测试。
- 改动算法前说明依据；目录合并不授权训练、更换模型或更换建图路线。其他组源码、服务、共享TF和控制进程不属于本组整理范围。
- 原始模型与输入数据保留；旧源码/环境和部署材料在artifacts内无损归档。不要把已归档旧工程重新展开为日常工作副本。
- 车端历史测试、安装材料和旧说明转存本机指定结果目录后逐文件校验，只有确认归档一致的车端副本才删除。车端不保留历史runs、tests、offline、inspection等材料；runs和logs仅留作下一次运行输出。envs、miniforge3、models和sensor_ws保持原路径，不能为减少目录而移动环境或已编译驱动。
