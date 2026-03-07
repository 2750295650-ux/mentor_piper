import time
import sys

try:
    from piper_sdk import *
except ImportError:
    print("错误：piper_sdk 未安装！")
    print("请先执行：")
    print("  git clone -b 1_0_0_beta https://github.com/agilexrobotics/piper_sdk.git")
    print("  cd piper_sdk")
    print("  pip install .")
    sys.exit(1)


def test_piper_connection():
    print("="*50)
    print("测试 Piper 机械臂连接")
    print("="*50)
    
    try:
        # 检查 CAN 接口
        print("\n0. 检查 CAN 接口...")
        print("   请确保已执行：sudo ip link set can0 up type can bitrate 1000000")
        input("   按回车继续...")
        
        # 初始化机械臂
        print("\n1. 初始化机械臂...")
        piper = Piper("can0")
        interface = piper.init()
        piper.connect()
        time.sleep(0.1)
        print("✓ 连接成功")
        
        # 使能机械臂
        print("\n2. 使能机械臂...")
        print("   请确保示教按钮指示灯熄灭")
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
        print("   ⚠️  警告：机械臂即将移动，请确保周围安全！")
        print("   机械臂将进行小幅度移动，然后回到原位")
        response = input("   确认继续？(yes/no): ")
        if response.lower() != 'yes':
            print("   跳过移动测试")
        else:
            # 稍微移动第一个关节
            new_pos = list(joint_pos)
            new_pos[0] += 0.1  # 小幅度移动
            print(f"   移动到: {new_pos}")
            piper.move_j(tuple(new_pos), 30)  # 低速
            time.sleep(2)
            
            # 回到原位
            print(f"   回到原位: {joint_pos}")
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
        
        print("\n常见问题解决：")
        print("1. CAN 接口未激活：执行 sudo ip link set can0 up type can bitrate 1000000")
        print("2. 示教模式未退出：短按示教按钮直到指示灯熄灭")
        print("3. piper_sdk 版本错误：确保安装的是 1_0_0_beta 分支")
        
        return False

if __name__ == '__main__':
    success = test_piper_connection()
    sys.exit(0 if success else 1)
