import numpy as np
import time
import cv2
import os

# ============================================
# 📌 Piper 机械臂配置区域 - 在这里修改初始位置和限制
# ============================================
#
# 初始位置（关节角度，单位：弧度）
# 每次环境 reset 时会回到这个位置
#
# 注意：如果需要使用示教模式来设置初始位置：
# 1. 运行 test_piper_camera.py 或使用示教按钮手动移动机械臂
# 2. 记录下满意的关节角度
# 3. 将角度转换为弧度（1度 = π/180 弧度）
# 4. 填入下方的 INITIAL_JOINT_POS
#
# ============================================

# 初始关节位置（弧度）- 每次 reset 时回到这个位置
# 示例：[0.0, 1.57, -1.57, 0.0, 0.0, 0.0] 对应 0°, 90°, -90°, 0°, 0°, 0°
INITIAL_JOINT_POS = [0.0116, 0.7366, -0.4097, -0.0418, 0.4053, -0.1915]

# ============================================
# 机械臂末端位置限制（毫米）
# 如果需要启用限制，设置 ENABLE_POSITION_LIMITS = True
# ============================================
ENABLE_POSITION_LIMITS = True

# X 轴限制（毫米）
MIN_X_MM = 85486.0   # X 轴最小值
MAX_X_MM = 200000.0  # X 轴最大值（可根据实际情况调整）

# Y 轴限制（毫米）
MIN_Y_MM = -100000.0 # Y 轴最小值（可根据实际情况调整）
MAX_Y_MM = 100000.0  # Y 轴最大值（可根据实际情况调整）

# Z 轴限制（毫米）
MIN_Z_MM = 113787.0  # Z 轴最小值
MAX_Z_MM = 300000.0  # Z 轴最大值（可根据实际情况调整）

# ============================================
# 📌 配置区域结束
# ============================================

try:
    from piper_sdk import *
    PiperSDK = C_PiperInterface_V2
except ImportError:
    print("警告：piper_sdk 未安装，将使用模拟模式")
    PiperSDK = None

try:
    from Camera_Module import DepthCameraModule
except ImportError:
    print("警告：Camera_Module 未找到，相机功能不可用")
    DepthCameraModule = None

# AprilTag 支持
try:
    from pupil_apriltags import Detector
    HAS_APRILTAGS = True
except ImportError:
    print("警告：pupil-apriltags 未安装，AprilTag 功能不可用")
    print("运行: pip install pupil-apriltags")
    HAS_APRILTAGS = False


class PiperRobot:
    def __init__(self, use_sim=False, camera_width=256, camera_height=256,
                 obj_pos=None, goal_pos=None,
                 debug_mode=False,
                 use_apriltag=False, tag_size=0.05,
                 camera_calibration_file='camera_calibration.npz',
                 hand_eye_calibration_file='hand_eye_calibration.npz'):
        self.use_sim = use_sim
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.use_apriltag = use_apriltag
        self.tag_size = tag_size
        self.debug_mode = debug_mode
        
        self.piper = None
        self.factor = 57295.7795
        self.gripper_pos = 0
        self.apriltag_visible = False
        
        if not use_sim and PiperSDK is not None:
            try:
                self.piper = PiperSDK("can0")
                self.piper.ConnectPort()
                
                while not self.piper.EnablePiper():
                    time.sleep(0.01)
                
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
                # 多次调用确保模式切换成功
                for _ in range(3):
                    self.piper.ModeCtrl(0x01, 0x01, 30, 0x00)
                    self.piper.EnableArm(7, 0x02)
                    time.sleep(0.05)
                # 清除所有关节错误并配置加速度
                self.piper.JointConfig(7, 0x00, 0x00, 500, 0xAE)
                time.sleep(0.1)
                print("✓ 机械臂连接成功")
            except Exception as e:
                print(f"警告：无法连接机械臂：{e}，将使用模拟模式")
                import traceback
                traceback.print_exc()
                self.use_sim = True
                self.piper = None
        
        self.camera = None
        if not use_sim and DepthCameraModule is not None:
            try:
                self.camera = DepthCameraModule(
                    color_width=640,
                    color_height=480,
                    depth_width=640,
                    depth_height=480,
                    fps=30
                )
            except Exception as e:
                print(f"警告：无法初始化相机：{e}")
                self.camera = None
        
        self.current_joint_pos = np.zeros(6)
        self.current_end_effector_pos = np.zeros(3)
        
        if obj_pos is None:
            self.obj_pos = np.array([0.0, 0.6, 0.0])
        else:
            self.obj_pos = np.array(obj_pos)
            
        if goal_pos is None:
            self.goal_pos = np.array([0.0, 0.75, 0.0])
        else:
            self.goal_pos = np.array(goal_pos)
            
        self.move_spd_rate_ctrl = 50
        
        # 机械臂卡住检测和限制检测
        self.last_joint_pos = None
        self.stuck_counter = 0
        self.safe_joint_pos = None  # 上一个安全的关节位置
        self.position_limit_violated = False  # 位置限制是否被触发
        self.stuck_threshold = 15  # 连续多少步认为卡住（已放宽）
        
        # AprilTag 初始化
        self.apriltag_detector = None
        self.camera_params = None
        self.T_cam2robot = None
        self.last_detected_obj_pos = None
        
        if not use_sim and use_apriltag and HAS_APRILTAGS:
            self._init_apriltag(camera_calibration_file, hand_eye_calibration_file)
        
        if self.use_sim:
            self._update_end_effector_pos_sim()
    
    def _init_apriltag(self, camera_calibration_file, hand_eye_calibration_file):
        """初始化 AprilTag 检测器"""
        try:
            # 获取脚本所在目录，支持相对路径
            script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            
            # 加载相机标定 - 尝试多个路径
            cam_calib_paths = [
                camera_calibration_file,
                os.path.join(script_dir, camera_calibration_file),
                'camera_calibration.npz',
                os.path.join(script_dir, 'camera_calibration.npz')
            ]
            
            cam_file_found = None
            for path in cam_calib_paths:
                if os.path.exists(path):
                    cam_file_found = path
                    break
            
            if cam_file_found:
                data = np.load(cam_file_found)
                self.camera_params = (
                    float(data['fx']),
                    float(data['fy']),
                    float(data['cx']),
                    float(data['cy'])
                )
                print(f"✓ 加载相机标定: {cam_file_found}")
            else:
                print(f"⚠️  未找到相机标定文件")
                print(f"   尝试路径: {cam_calib_paths}")
                print("   AprilTag 将无法估计 3D 位置")
            
            # 尝试加载手眼标定（优先简单标定）
            self.hand_eye_offset = None
            self.T_cam2robot = None
            
            # 尝试手眼标定文件 - 多个路径
            hand_eye_paths = [
                'simple_hand_eye.json',
                os.path.join(script_dir, 'simple_hand_eye.json'),
                hand_eye_calibration_file,
                os.path.join(script_dir, hand_eye_calibration_file)
            ]
            
            hand_eye_file_found = None
            hand_eye_type = None
            
            for path in hand_eye_paths:
                if os.path.exists(path):
                    hand_eye_file_found = path
                    if 'simple' in path:
                        hand_eye_type = 'simple'
                    else:
                        hand_eye_type = 'full'
                    break
            
            if hand_eye_file_found and hand_eye_type == 'simple':
                import json
                with open(hand_eye_file_found, 'r') as f:
                    calib_data = json.load(f)
                self.hand_eye_offset = np.array(calib_data['offset'])
                print(f"✓ 加载简单手眼标定: {hand_eye_file_found}")
                print(f"  偏移量: {self.hand_eye_offset}")
            elif hand_eye_file_found and hand_eye_type == 'full':
                data = np.load(hand_eye_file_found)
                self.T_cam2robot = data['T_cam2gripper']
                print(f"✓ 加载完整手眼标定: {hand_eye_file_found}")
            else:
                print(f"⚠️  未找到手眼标定文件")
                print(f"   请先运行 easy_hand_eye_calibration.py")
                print("   将使用简单坐标转换（可能不准确）")
            
            # 初始化检测器
            self.apriltag_detector = Detector(
                families='tag36h11',
                nthreads=1,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0
            )
            
            print("✓ AprilTag 检测器初始化成功")
            
        except Exception as e:
            print(f"✗ AprilTag 初始化失败: {e}")
            self.apriltag_detector = None
    
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
            try:
                joint_pose = self.piper.GetArmJointMsgs()
                self.current_joint_pos = np.array([
                    joint_pose.joint_state.joint_1 / self.factor,
                    joint_pose.joint_state.joint_2 / self.factor,
                    joint_pose.joint_state.joint_3 / self.factor,
                    joint_pose.joint_state.joint_4 / self.factor,
                    joint_pose.joint_state.joint_5 / self.factor,
                    joint_pose.joint_state.joint_6 / self.factor
                ])
            except Exception as e:
                print(f"获取关节位置错误：{e}")
        return self.current_joint_pos.copy()
        
    def set_joint_pos(self, joint_pos, gripper_pos=None, speed=None):
        joint_pos = np.array(joint_pos)
        
        if gripper_pos is not None:
            self.gripper_pos = gripper_pos
        
        # 重置位置限制触发标志
        self.position_limit_violated = False
        
        if not self.use_sim and self.piper is not None:
            try:
                spd = speed if speed is not None else 100
                
                # 关节角度限制（单位：弧度）- 已放宽以允许更多探索
                # 注意：这些限制仍然在 SDK 安全范围内
                joint_limits = [
                    (-2.6179, 2.6179),  # Joint 1: [-150°, 150°] (SDK 限制)
                    (-0.1745, 3.14),    # Joint 2: [-10° 到 180°] (原 0° 到 180°，向下扩展 10°)
                    (-3.1416, 0.1745),  # Joint 3: [-180° 到 10°] (原 -170° 到 0°，向下扩展 10°)
                    (-1.745, 1.745),    # Joint 4: [-100° 到 100°] (保持 SDK 限制)
                    (-1.22, 1.22),      # Joint 5: [-70° 到 70°] (保持 SDK 限制)
                    (-2.09439, 2.09439) # Joint 6: [-120° 到 120°] (保持 SDK 限制)
                ]
                
                # 限制关节角度在安全范围内
                joint_pos_clipped = np.clip(joint_pos, 
                                            [lim[0] for lim in joint_limits],
                                            [lim[1] for lim in joint_limits])
                
                joint_0 = round(joint_pos_clipped[0] * self.factor)
                joint_1 = round(joint_pos_clipped[1] * self.factor)
                joint_2 = round(joint_pos_clipped[2] * self.factor)
                joint_3 = round(joint_pos_clipped[3] * self.factor)
                joint_4 = round(joint_pos_clipped[4] * self.factor)
                joint_5 = round(joint_pos_clipped[5] * self.factor)
                
                gripper_cmd = round(abs(self.gripper_pos) * 1000 * 1000)
                
                if self.debug_mode:
                    print(f"[DEBUG] joint_0-5: {joint_0}, {joint_1}, {joint_2}, {joint_3}, {joint_4}, {joint_5}")
                    print(f"[DEBUG] gripper_cmd: {gripper_cmd}")
                
                # 确保机械臂处于 CAN 命令控制模式并使能电机
                self.piper.ModeCtrl(0x01, 0x01, spd, 0x00)
                self.piper.EnableArm(7, 0x02)
                # 清除错误并确保电机准备好
                self.piper.JointConfig(7, 0x00, 0x00, 500, 0xAE)
                time.sleep(0.01)  # 等待命令生效
                self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
                self.piper.GripperCtrl(gripper_cmd, 1000, 0x01, 0)
                
                # 等待移动完成
                time.sleep(0.1)
                
                # 如果启用了位置限制，检查末端位置
                if ENABLE_POSITION_LIMITS and self.safe_joint_pos is not None:
                    end_pose = self.piper.GetArmEndPoseMsgs()
                    current_x_mm = end_pose.end_pose.X_axis
                    current_y_mm = end_pose.end_pose.Y_axis
                    current_z_mm = end_pose.end_pose.Z_axis
                    
                    # 检查是否违反任何限制
                    x_violated = current_x_mm < MIN_X_MM or current_x_mm > MAX_X_MM
                    y_violated = current_y_mm < MIN_Y_MM or current_y_mm > MAX_Y_MM
                    z_violated = current_z_mm < MIN_Z_MM or current_z_mm > MAX_Z_MM
                    
                    if x_violated or y_violated or z_violated:
                        # 违反了限制，回退到上一个安全位置
                        self.position_limit_violated = True
                        if self.debug_mode:
                            if x_violated:
                                print(f"[DEBUG] X 轴位置 {current_x_mm} mm 超出限制 [{MIN_X_MM}, {MAX_X_MM}]")
                            if y_violated:
                                print(f"[DEBUG] Y 轴位置 {current_y_mm} mm 超出限制 [{MIN_Y_MM}, {MAX_Y_MM}]")
                            if z_violated:
                                print(f"[DEBUG] Z 轴位置 {current_z_mm} mm 超出限制 [{MIN_Z_MM}, {MAX_Z_MM}]")
                            print(f"[DEBUG] 违反限制，回退到上一个安全位置")
                        
                        # 回退到上一个安全位置
                        safe_joint_0 = round(self.safe_joint_pos[0] * self.factor)
                        safe_joint_1 = round(self.safe_joint_pos[1] * self.factor)
                        safe_joint_2 = round(self.safe_joint_pos[2] * self.factor)
                        safe_joint_3 = round(self.safe_joint_pos[3] * self.factor)
                        safe_joint_4 = round(self.safe_joint_pos[4] * self.factor)
                        safe_joint_5 = round(self.safe_joint_pos[5] * self.factor)
                        
                        self.piper.JointCtrl(safe_joint_0, safe_joint_1, safe_joint_2, 
                                            safe_joint_3, safe_joint_4, safe_joint_5)
                        time.sleep(0.2)
                    else:
                        # 没有违反限制，更新安全位置
                        self.position_limit_violated = False
                        self.safe_joint_pos = joint_pos_clipped.copy()
                else:
                    # 没有启用限制，或者是第一次移动，更新安全位置
                    self.safe_joint_pos = joint_pos_clipped.copy()
                
                # 获取机械臂状态用于调试（仅在debug_mode下）
                if self.debug_mode:
                    arm_status = self.piper.GetArmStatus()
                    print(f"[DEBUG] Arm Status: {arm_status}")
            except Exception as e:
                print(f"设置关节位置错误：{e}")
                import traceback
                traceback.print_exc()
        
        self.current_joint_pos = joint_pos.copy()
        if self.use_sim:
            self._update_end_effector_pos_sim()
            
    def get_end_effector_pos(self):
        if not self.use_sim and self.piper is not None:
            try:
                end_pose = self.piper.GetArmEndPoseMsgs()
                self.current_end_effector_pos = np.array([
                    end_pose.end_pose.X_axis / 1000.0,
                    end_pose.end_pose.Y_axis / 1000.0,
                    end_pose.end_pose.Z_axis / 1000.0
                ])
            except Exception as e:
                print(f"获取末端位置错误：{e}")
        return self.current_end_effector_pos.copy()
            
    def get_obj_pos(self):
        if self.use_sim:
            self.apriltag_visible = True
            return self.obj_pos.copy()
        
        # 真实世界：尝试用 AprilTag 检测
        self.apriltag_visible = False
        if self.use_apriltag and self.apriltag_detector is not None and self.camera is not None:
            try:
                color_array_rgb, _ = self.camera.get_frame()
                
                if color_array_rgb is not None and self.camera_params is not None:
                    gray = cv2.cvtColor(color_array_rgb, cv2.COLOR_RGB2GRAY)
                    detections = self.apriltag_detector.detect(
                        gray,
                        estimate_tag_pose=True,
                        camera_params=self.camera_params,
                        tag_size=self.tag_size
                    )
                    
                    if detections:
                        self.apriltag_visible = True
                        tag = detections[0]
                        tag_pos_camera = tag.pose_t.flatten()
                        
                        # 转换到机械臂坐标系
                        obj_pos = self._camera_to_robot(tag_pos_camera)
                        
                        self.last_detected_obj_pos = obj_pos
                        return obj_pos
            
            except Exception as e:
                print(f"⚠️  AprilTag 检测错误: {e}")
        
        # 如果检测失败，返回最后一次检测到的位置或默认位置
        if self.last_detected_obj_pos is not None:
            return self.last_detected_obj_pos.copy()
        
        return self.obj_pos.copy()
    
    def _camera_to_robot(self, camera_pos):
        """
        将相机坐标系转换为机械臂坐标系
        
        优先级：
        1. 简单手眼标定偏移量（推荐初学者）
        2. 完整手眼标定矩阵
        3. 简单坐标转换（默认）
        """
        if self.hand_eye_offset is not None:
            # 使用简单手眼标定偏移量
            # offset 是毫米，需要转换为米
            offset_m = self.hand_eye_offset / 1000.0
            
            # 先尝试简单的转换，根据 easy_hand_eye_calibration.py 的 offset 计算方式
            # robot_pos = tag_pos + offset
            robot_pos = camera_pos + offset_m
            
            # 打印用于调试的原始值（仅在debug_mode时打印）
            if self.debug_mode:
                print(f"[DEBUG] camera_pos (tag_pos): {camera_pos}")
                print(f"[DEBUG] offset_m: {offset_m}")
                print(f"[DEBUG] robot_pos: {robot_pos}")
            
            return robot_pos
        
        if self.T_cam2robot is not None:
            # 使用完整手眼标定结果
            p_cam = np.ones(4)
            p_cam[:3] = camera_pos
            p_robot = self.T_cam2robot @ p_cam
            return p_robot[:3]
        
        # 简单坐标转换（需要根据实际情况调整）
        robot_pos = np.zeros(3)
        
        # 假设相机坐标系：
        #   x: 相机右方
        #   y: 相机下方
        #   z: 相机前方
        #
        # 转换为机械臂坐标系（示例）
        robot_pos[0] = camera_pos[0]
        robot_pos[1] = camera_pos[2]
        robot_pos[2] = -camera_pos[1]
        
        # 添加相机相对于机械臂底座的偏移（需要根据实际安装位置调整）
        robot_pos[0] += 0.0
        robot_pos[1] += 0.0
        robot_pos[2] += 0.0
        
        return robot_pos
        
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
        self.gripper_pos = 0
        # 重置卡住检测和限制检测状态
        self.last_joint_pos = None
        self.stuck_counter = 0
        self.safe_joint_pos = np.array(INITIAL_JOINT_POS).copy()
        self.position_limit_violated = False
        
        if not hasattr(self, '_initial_obj_pos'):
            self._initial_obj_pos = self.obj_pos.copy()
        if not hasattr(self, '_initial_goal_pos'):
            self._initial_goal_pos = self.goal_pos.copy()
        self.obj_pos = self._initial_obj_pos.copy()
        self.goal_pos = self._initial_goal_pos.copy()
        
        if not self.use_sim and self.piper is not None:
            # 使用配置的初始关节位置
            self.set_joint_pos(INITIAL_JOINT_POS, gripper_pos=0, speed=30)
            time.sleep(1.0)
        else:
            self.current_joint_pos = np.array(INITIAL_JOINT_POS).copy()
            
        if self.use_sim:
            self._update_end_effector_pos_sim()
            
    def update_stuck_detection(self):
        """更新卡住检测状态（仅在step中调用一次）"""
        if self.last_joint_pos is None:
            self.last_joint_pos = self.current_joint_pos.copy()
            return False
        
        # 计算关节位置变化
        joint_change = np.linalg.norm(self.current_joint_pos - self.last_joint_pos)
        if joint_change < 0.0001:  # 关节位置几乎没有变化（阈值已放宽10倍）
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        
        self.last_joint_pos = self.current_joint_pos.copy()
        
        return self.stuck_counter >= self.stuck_threshold
    
    def is_stuck(self):
        """检测机械臂是否卡住（只读，不修改状态）"""
        return self.stuck_counter >= self.stuck_threshold
            
    def step(self, action, dt=0.01):
        action = np.clip(action, -1.0, 1.0)
        joint_delta = action[:6] * 0.1
        new_joint_pos = self.current_joint_pos + joint_delta
        # 把 action[6] 从 [-1,1] 映射到 [0,1]
        new_gripper_pos = (action[6] + 1.0) / 2.0
        self.set_joint_pos(new_joint_pos, new_gripper_pos)
        
        # 更新卡住检测（每步只调用一次）
        self.update_stuck_detection()
        
        if self.use_sim:
            self._update_obj_position_sim(action)
            
        time.sleep(dt)
        
    def close(self):
        if self.piper is not None:
            try:
                self.piper.DisconnectPort()
            except:
                pass
        if self.camera is not None:
            try:
                self.camera.stop()
            except:
                pass
