# Piper 机械臂训练 - 快速开始

## 🚀 5 步开始训练

### 第 1 步：硬件连接
- [ ] 将 Piper 机械臂通过 USB-CAN 连接到电脑
- [ ] 将 RealSense 摄像头通过 USB 3.0 连接到电脑
- [ ] 打开机械臂电源

### 第 2 步：激活 CAN 接口
```bash
sudo ip link set can0 up type can bitrate 1000000
```

### 第 3 步：测试硬件连接
```bash
# 测试机械臂
python test_piper_connection.py

# 测试摄像头
python test_camera.py
```

### 第 4 步：修改配置文件
编辑 `piper/cfgs/config.yaml`：
```yaml
use_sim: false  # 使用真实机械臂
```

### 第 5 步：开始训练
```bash
python train_piper.py
```

---

## 📋 详细文档

- **完整硬件设置指南**: `SETUP_PIPER_HARDWARE.md`
- **训练指南**: `TRAIN_PIPER_GUIDE.md`
- **Piper 环境说明**: `piper/README.md`

---

## ⚠️ 安全提示

1. **先测试模拟模式**：`python test_piper_env.py`
2. **机械臂周围保持安全距离**
3. **准备好随时按示教按钮**
4. **首次使用低速运动**（在 `piper/robot.py` 中调整 `move_spd_rate_ctrl`）

---

## 🔧 测试脚本说明

| 脚本 | 用途 |
|------|------|
| `test_piper_env.py` | 测试模拟环境 |
| `verify_piper_flow.py` | 验证完整流程 |
| `test_piper_connection.py` | 测试真实机械臂连接 |
| `test_camera.py` | 测试 RealSense 摄像头 |

---

## ❓ 需要帮助？

查看详细文档或参考：
- piper_sdk 官方文档: https://github.com/agilexrobotics/piper_sdk
- Mentor 项目: https://github.com/suninghuang19/mentor
