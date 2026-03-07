# Piper 机械臂硬件设置指南

## 一、硬件连接

### 1.1 连接 Piper 机械臂

**所需设备：**
- Piper 机械臂
- USB-CAN 适配器
- 电脑（推荐 Ubuntu 20.04+）

**连接步骤：**
1. 将 USB-CAN 适配器的一端连接到机械臂
2. 将另一端连接到电脑的 USB 端口
3. 打开机械臂电源开关
4. 检查机械臂指示灯：
   - 电源指示灯亮 → 正常
   - 示教按钮指示灯灭 → 可以开始控制

### 1.2 连接 RealSense 摄像头

**所需设备：**
- Intel RealSense D435/D455 深度相机
- USB 3.0 数据线

**连接步骤：**
1. 将 RealSense 相机通过 USB 3.0 线连接到电脑
2. 确保相机安装在合适位置，能够拍摄到：
   - 机械臂末端
   - 咖啡杯（物体）
   - 目标位置
3. 固定相机位置，避免训练过程中移动

## 二、软件环境配置

### 2.1 安装 CAN 工具（Ubuntu）

```bash
# 更新软件包
sudo apt update

# 安装 CAN 工具
sudo apt install can-utils ethtool
```

### 2.2 激活 CAN 接口

**每次使用前都需要执行：**

```bash
# 激活 CAN 接口，设置波特率为 1Mbps
sudo ip link set can0 up type can bitrate 1000000

# 验证 CAN 接口是否激活
ip link show can0
```

**如果看到以下输出说明成功：**
```
3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc fq_codel state UP mode DEFAULT group default qlen 10
    link/can
```

### 2.3 安装 piper_sdk

```bash
# 克隆 piper_sdk 仓库（注意：必须是 1_0_0_beta 分支！）
git clone -b 1_0_0_beta https://github.com/agilexrobotics/piper_sdk.git
cd piper_sdk

# 安装 SDK
pip install .

# 验证安装
python3 -c "import piper_sdk; print('piper_sdk 安装成功！')"
```

### 2.4 安装 RealSense 相关依赖

```bash
# 安装 pyrealsense2
pip install pyrealsense2

# 安装 OpenCV
pip install opencv-python

# 安装 Open3D（可选，用于点云处理）
pip install open3d
```

### 2.5 验证 Camera_Module

确保 `Camera_Module.py` 在项目根目录下，并且可以正常导入：

```bash
python -c "from Camera_Module import DepthCameraModule; print('Camera_Module 导入成功！')"
```

## 三、测试硬件连接

### 3.1 测试机械臂连接

创建测试脚本 `test_piper_connection.py`：

```python
import time
from piper_sdk import *

def test_piper_connection():
    print("="*50)
    print("测试 Piper 机械臂连接")
    print("="*50)
    
    try:
        # 初始化机械臂
        print("\n1. 初始化机械臂...")
        piper = Piper("can0")
        interface = piper.init()
        piper.connect()
        time.sleep(0.1)
        print("✓ 连接成功")
        
        # 使能机械臂
        print("\n2. 使能机械臂...")
        while not piper.enable_arm():
            time.sleep(0.01)
        print("✓ 使能成功")
        
        # 设置控制模式
        print("\n3. 设置控制模式...")
        interface.ModeCtrl(0x01, 0x01, 50, 0x00)
        print("✓ 控制模式设置成功")
        
        # 获取当前关节位置
        print("\n4. 获取关节位置...")
        joint_pos = piper.get_joint_states()[0]
        print(f"✓ 当前关节位置: {joint_pos}")
        
        # 测试小幅度移动（安全起见）
        print("\n5. 测试小幅度移动...")
        print("   警告：机械臂即将移动，请确保周围安全！")
        input("   按回车继续...")
        
        # 稍微移动第一个关节
        new_pos = list(joint_pos)
        new_pos[0] += 0.1  # 小幅度移动
        piper.move_j(tuple(new_pos), 30)  # 低速
        time.sleep(2)
        
        # 回到原位
        piper.move_j(tuple(joint_pos), 30)
        time.sleep(2)
        print("✓ 移动测试成功")
        
        # 禁用机械臂
        print("\n6. 禁用机械臂...")
        piper.disable_arm()
        piper.disconnect()
        print("✓ 断开连接")
        
        print("\n" + "="*50)
        print("✓ 所有测试通过！机械臂连接正常")
        print("="*50)
        
        return True
        
    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    test_piper_connection()
```

运行测试：
```bash
python test_piper_connection.py
```

### 3.2 测试摄像头

创建测试脚本 `test_camera.py`：

```python
import cv2
import time
from Camera_Module import DepthCameraModule

def test_camera():
    print("="*50)
    print("测试 RealSense 摄像头")
    print("="*50)
    
    try:
        # 初始化摄像头
        print("\n1. 初始化摄像头...")
        camera = DepthCameraModule(
            color_width=640,
            color_height=480,
            depth_width=640,
            depth_height=480,
            fps=30
        )
        print("✓ 摄像头初始化成功")
        
        # 测试获取帧
        print("\n2. 测试获取图像...")
        for i in range(5):
            color_array_rgb, depth_array = camera.get_frame()
            if color_array_rgb is not None:
                print(f"   帧 {i+1}: {color_array_rgb.shape}")
                
                # 显示图像（按 q 退出）
                cv2.imshow('Camera Test', cv2.cvtColor(color_array_rgb, cv2.COLOR_RGB2BGR))
                if cv2.waitKey(500) & 0xFF == ord('q'):
                    break
        
        cv2.destroyAllWindows()
        print("✓ 图像获取成功")
        
        # 关闭摄像头
        print("\n3. 关闭摄像头...")
        camera.stop()
        print("✓ 摄像头关闭成功")
        
        print("\n" + "="*50)
        print("✓ 所有测试通过！摄像头工作正常")
        print("="*50)
        
        return True
        
    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    test_camera()
```

运行测试：
```bash
python test_camera.py
```

## 四、开始训练

### 4.1 修改配置文件

编辑 `piper/cfgs/config.yaml`，将 `use_sim` 改为 `false`：

```yaml
# piper specific
use_sim: false  # 使用真实机械臂
```

### 4.2 激活 CAN 接口（每次训练前）

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### 4.3 开始训练

```bash
python train_piper.py
```

## 五、安全注意事项

⚠️ **重要安全提示：**

1. **首次使用时，先在模拟模式下测试**
   ```bash
   python test_piper_env.py  # 先确保代码正常
   ```

2. **机械臂周围保持安全距离**
   - 确保机械臂运动范围内无障碍物
   - 准备好随时按示教按钮停止

3. **先测试小幅度移动**
   - 使用 `test_piper_connection.py` 测试
   - 确认机械臂运动方向正确

4. **示教按钮的使用**
   - 短按示教按钮：进入/退出示教模式
   - 遇到紧急情况：立即按示教按钮

5. **训练速度设置**
   - 在 `piper/robot.py` 中调整 `move_spd_rate_ctrl`
   - 初始值建议设为 30-50（慢速）

## 六、常见问题解决

### Q1: CAN 接口无法激活
**解决方案：**
```bash
# 检查 USB-CAN 适配器是否连接
lsusb

# 尝试重启 CAN 接口
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

### Q2: 找不到 piper_sdk
**解决方案：**
```bash
# 确认安装的是 1_0_0_beta 分支
cd piper_sdk
git branch  # 应该显示 * 1_0_0_beta

# 重新安装
pip install .
```

### Q3: 摄像头无法识别
**解决方案：**
- 确保使用 USB 3.0 端口
- 检查 `dmesg` 输出：`dmesg | grep -i realsense`
- 重新插拔摄像头

### Q4: 机械臂不动
**解决方案：**
- 检查示教按钮指示灯是否熄灭
- 如果亮着，短按示教按钮退出示教模式
- 确认 CAN 接口已激活
