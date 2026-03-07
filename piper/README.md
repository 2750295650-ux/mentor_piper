# Piper 机械臂模块

本模块用于将 Mentor 框架与 Piper 机械臂集成，支持在真实硬件或模拟环境中训练机械臂完成 Coffee Push 任务。

## 文件说明

- `__init__.py` - 模块初始化
- `env.py` - Piper 环境包装器，与 Mentor 框架兼容
- `robot.py` - Piper 机器人接口，连接 piper_sdk 和相机模块
- `cfgs/config.yaml` - 训练配置文件

## 快速开始

### 1. 测试环境

```bash
python test_piper_env.py
```

### 2. 开始训练 (模拟模式)

```bash
python train_piper.py
```

### 3. 使用真实机械臂

修改 `piper/cfgs/config.yaml`：

```yaml
use_sim: false
```

然后运行训练：

```bash
python train_piper.py
```

## 详细文档

请查看 [TRAIN_PIPER_GUIDE.md](../TRAIN_PIPER_GUIDE.md) 获取完整的训练指南。
