# Piper 机械臂 Coffee Push 任务训练指南

## 概述

本指南说明了如何使用 Mentor 框架训练 Piper 机械臂完成 Coffee Push 任务。我们已经将 MetaWorld 的 Coffee Push 任务移植到 Piper 机械臂上，并集成了 RealSense 深度相机模块。

## 环境搭建

### 1. 依赖安装

确保已安装以下依赖：

```bash
# 基础依赖
pip install torch torchvision hydra-core numpy opencv-python

# RealSense 相机依赖
pip install pyrealsense2

# Open3D (用于点云处理)
pip install open3d

# Piper SDK
# 请按照 https://github.com/agilexrobotics/piper_sdk 的说明安装
```

### 2. 硬件要求

- Piper 6 自由度机械臂
- RealSense 深度相机 (如 D435i)
- 支持 CUDA 的 GPU (推荐)

## 代码结构

```
mentor/
├── piper/
│   ├── __init__.py
│   ├── env.py              # Piper 环境包装器
│   ├── robot.py            # Piper 机器人接口
│   └── cfgs/
│       └── config.yaml     # 训练配置
├── Camera_Module.py         # 相机模块
├── train_piper.py          # Piper 训练脚本
└── TRAIN_PIPER_GUIDE.md    # 本文件
```

## 奖励机制说明

我们完全移植了 MetaWorld Coffee Push 任务的奖励逻辑：

### 奖励组成

1. **物体接近奖励**：机械臂末端接近咖啡杯时获得奖励
2. **到位奖励**：咖啡杯接近目标位置时获得奖励
3. **抓取奖励**：机械臂成功抓取咖啡杯时获得额外奖励
4. **成功奖励**：任务成功完成（咖啡杯到达目标位置）时获得最大奖励 (10.0)

### 成功判定

当咖啡杯与目标位置的距离 ≤ 0.07 米时，判定为任务成功。

## 训练配置

配置文件位于 `piper/cfgs/config.yaml`，主要参数：

```yaml
# 环境参数
frame_stack: 3              # 帧堆叠数量
action_repeat: 2            # 动作重复次数
use_sim: true               # 使用模拟模式 (true) 或真实机械臂 (false)

# 训练参数
num_train_frames: 2100000   # 总训练帧数
num_seed_frames: 4000       # 随机探索帧数
eval_every_frames: 10000    # 评估间隔

# 智能体参数
agent:
  _target_: agents.mentor_mw.MENTORAgent
  hidden_dim: 1024
  feature_dim: 50
  # ... 其他参数
```

## 训练流程

### 1. 模拟模式训练 (推荐先运行)

首先在模拟模式下测试训练流程：

```bash
python train_piper.py
```

模拟模式特点：
- 不需要真实的 Piper 机械臂和相机
- 使用简化的物理模拟
- 可以快速验证代码逻辑

### 2. 真实机械臂训练

准备好后，修改配置文件使用真实硬件：

```yaml
use_sim: false
```

然后运行相同的训练命令：

```bash
python train_piper.py
```

## 相机适配说明

### 图像输入格式

- 相机捕获 RGB 图像
- 图像大小：256x256 像素
- 帧堆叠：3 帧连续图像
- 最终输入形状：(9, 256, 256) [通道在前]

### 相机设置

相机模块在 `Camera_Module.py` 中配置：
- 彩色图像：1280x720 (自动缩放到 256x256)
- 深度图像：640x480
- 帧率：30 FPS

### 与 Mentor 框架的适配

环境自动处理：
1. 从相机获取 RGB 图像
2. 缩放到 256x256
3. 堆叠 3 帧历史图像
4. 转换为 (通道, 高度, 宽度) 格式

## 训练监控

### 日志和可视化

- TensorBoard 日志：`./runs/` 目录
- 视频录制：评估时自动保存视频
- 模型快照：定期保存到 `./snapshots/` 目录

### 关键指标

- `episode_reward`：每集奖励
- `episode_success_rate`：成功率
- `fps`：训练速度

## 从检查点恢复训练

如果训练中断，可以从保存的检查点恢复：

```bash
# 从默认检查点恢复
python train_piper.py

# 从指定 ID 恢复
python train_piper.py load_from_id=true load_id=100000

# 从指定路径恢复
python train_piper.py snapshot_path=/path/to/snapshot.pt
```

## 仅评估模式

训练完成后，可以使用已保存的模型进行评估：

```bash
python train_piper.py eval_only=true
```

## 常见问题

### Q: 相机无法连接？

A: 确保：
1. RealSense 相机正确连接
2. 没有其他程序占用相机
3. 相机权限正确

### Q: Piper 机械臂无法连接？

A: 请参考 piper_sdk 文档确保：
1. 机械臂电源已打开
2. USB 连接正常
3. piper_sdk 正确安装

### Q: 训练不收敛？

A: 可以尝试：
1. 增加训练帧数
2. 调整学习率
3. 调整探索策略 (stddev_schedule)
4. 确保奖励设置合理

## 下一步

1. 在模拟模式下验证训练流程
2. 调整超参数以获得更好性能
3. 在真实硬件上进行测试
4. 根据实际情况调整任务参数 (目标位置、物体初始位置等)

祝你训练顺利！
