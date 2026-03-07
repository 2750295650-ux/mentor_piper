import numpy as np
import time
import cv2

try:
    from piper_sdk import *
    PiperSDK = Piper
except ImportError:
    print("警告：piper_sdk 未安装，将使用模拟模式")
    PiperSDK = None

try:
    from Camera_Module import DepthCameraModule
except ImportError:
    print("警告：Camera_Module 未找到，相机功能不可用")
    DepthCameraModule = None


class PiperRobot:
    def __init__(self, use_sim=False, camera_width=256, camera_height=256):
        self.use_sim = use_sim
        self.camera_width = camera_width
        self.camera_height = camera_height
        
        self.piper = None
        self.interface = None
        
        if not use_sim and PiperSDK is not None:
            try:
                self.piper = PiperSDK("can0")
                self.interface = self.piper.init()
                self.piper.connect()
                time.sleep(0.1)
                
                while not self.piper.enable_arm():
                    time.sleep(0.01)
                
                self.interface.ModeCtrl(0x01, 0x01, 100, 0x00)
                print("机械臂连接成功")
            except Exception as e:
                print(f"警告：无法连接机械臂：{e}，将使用模拟模式")
                self.use_sim = True
                self.piper = None
                self.interface = None
        
        self.camera = None
        if not use_sim and DepthCameraModule is not None:
            try:
                self.camera = DepthCameraModule(
                    color_width=camera_width,
                    color_height=camera_height,
                    depth_width=640,
                    depth_height=480,
                    fps=30
                )
            except Exception as e:
                print(f"警告：无法初始化相机：{e}")
                self.camera = None
        
        self.current_joint_pos = np.zeros(6)
        self.current_end_effector_pos = np.zeros(3)
        self.obj_pos = np.array([0.0, 0.6, 0.0])
        self.goal_pos = np.array([0.0, 0.75, 0.0])
        self.move_spd_rate_ctrl = 50
        
        if self.use_sim:
            self._update_end_effector_pos_sim()
    
    def _update_end_effector_pos_sim(self):
        self.current_end_effector_pos = np.array([
            np.sin(self.current_joint_pos[0]) * 0.3,
            np.cos(self.current_joint_pos[1]) * 0.3,
            np.sin(self.current_joint_pos[2]) * 0.1 + 0.2
        ])
        
    def _update_obj_position_sim(self, action):
        dist_to_obj = np.linalg.norm(self.current_end_effector_pos - self.obj_pos)
        if dist_to_obj < 0.05:
            target_pos = self.goal_pos
            push_dir = target_pos[:2] - self.obj_pos[:2]
            if np.linalg.norm(push_dir) > 0.001:
                push_dir = push_dir / np.linalg.norm(push_dir)
                self.obj_pos[:2] += push_dir * 0.005
                
    def get_joint_pos(self):
        if not self.use_sim and self.piper is not None:
            joint_state = self.piper.get_joint_states()[0]
            self.current_joint_pos = np.array(joint_state)
        return self.current_joint_pos.copy()
        
    def set_joint_pos(self, joint_pos, speed=None):
        joint_pos = np.array(joint_pos)
        if not self.use_sim and self.piper is not None:
            spd = speed if speed is not None else self.move_spd_rate_ctrl
            self.piper.move_j(tuple(joint_pos), spd)
        self.current_joint_pos = joint_pos.copy()
        if self.use_sim:
            self._update_end_effector_pos_sim()
            
    def get_end_effector_pos(self):
        if self.use_sim:
            return self.current_end_effector_pos.copy()
        else:
            return self.current_end_effector_pos.copy()
            
    def get_obj_pos(self):
        return self.obj_pos.copy()
        
    def set_obj_pos(self, obj_pos):
        self.obj_pos = np.array(obj_pos)
        
    def get_goal_pos(self):
        return self.goal_pos.copy()
        
    def set_goal_pos(self, goal_pos):
        self.goal_pos = np.array(goal_pos)
        
    def get_camera_image(self):
        if self.camera is not None:
            try:
                color_array_rgb, _ = self.camera.get_frame()
                if color_array_rgb is not None:
                    resized = cv2.resize(color_array_rgb, (self.camera_width, self.camera_height))
                    return resized
            except Exception as e:
                print(f"获取相机图像错误：{e}")
        
        if self.use_sim:
            return np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
        else:
            return np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
            
    def reset(self):
        self.current_joint_pos = np.zeros(6)
        self.obj_pos = np.array([0.0, 0.6, 0.0])
        self.goal_pos = np.array([0.0, 0.75, 0.0])
        
        if not self.use_sim and self.piper is not None:
            self.piper.move_j(tuple(np.zeros(6)), self.move_spd_rate_ctrl)
            time.sleep(2.0)
            
        if self.use_sim:
            self._update_end_effector_pos_sim()
            
    def step(self, action, dt=0.01):
        action = np.clip(action, -1.0, 1.0)
        joint_delta = action * 0.1
        new_joint_pos = self.current_joint_pos + joint_delta
        self.set_joint_pos(new_joint_pos)
        
        if self.use_sim:
            self._update_obj_position_sim(action)
            
        time.sleep(dt)
        
    def close(self):
        if self.piper is not None:
            try:
                self.piper.disable_arm()
                self.piper.disconnect()
            except:
                pass
        if self.camera is not None:
            try:
                self.camera.stop()
            except:
                pass
