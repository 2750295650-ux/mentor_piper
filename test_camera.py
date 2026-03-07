import cv2
import time
import sys

try:
    from Camera_Module import DepthCameraModule
except ImportError:
    print("错误：Camera_Module.py 未找到！")
    print("请确保 Camera_Module.py 在项目根目录下")
    sys.exit(1)


def test_camera():
    print("="*50)
    print("测试 RealSense 摄像头")
    print("="*50)
    
    try:
        # 初始化摄像头
        print("\n1. 初始化摄像头...")
        print("   请确保 RealSense 相机已连接到 USB 3.0 端口")
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
        print("   将显示摄像头图像，按 'q' 键退出")
        print("   请确认图像中能看到机械臂和工作区域")
        time.sleep(1)
        
        frame_count = 0
        while True:
            color_array_rgb, depth_array = camera.get_frame()
            if color_array_rgb is not None:
                frame_count += 1
                
                # 显示图像
                display_img = cv2.cvtColor(color_array_rgb, cv2.COLOR_RGB2BGR)
                
                # 添加信息
                cv2.putText(display_img, f"Frame: {frame_count}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(display_img, "Press 'q' to quit", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow('Camera Test', display_img)
                
                # 按 q 退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        cv2.destroyAllWindows()
        print(f"✓ 成功获取 {frame_count} 帧图像")
        
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
        
        print("\n常见问题解决：")
        print("1. 确保使用 USB 3.0 端口（不是 USB 2.0）")
        print("2. 检查摄像头是否被其他程序占用")
        print("3. 重新插拔摄像头")
        print("4. 运行: dmesg | grep -i realsense 查看系统日志")
        
        return False

if __name__ == '__main__':
    success = test_camera()
    sys.exit(0 if success else 1)
