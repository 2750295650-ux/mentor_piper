"""
Piper SpaceMouse Controller - 适配 Piper 机械臂的 SpaceMouse 控制
将 3Dconnexion SpaceMouse 的 6DOF 输入映射到 Piper 的关节空间
"""

import numpy as np
from typing import Optional, Tuple

try:
    from piper.pyspacemouse import SpaceNavigator, open, close, read
    SPACEMOUSE_AVAILABLE = True
except ImportError:
    print("警告：pyspacemouse 未安装，SpaceMouse 功能不可用")
    SPACEMOUSE_AVAILABLE = False


class PiperSpaceMouseController:
    """
    将 SpaceMouse 的笛卡尔控制映射到 Piper 的关节控制
    
    SpaceMouse 输出：6 DOF (x, y, z, roll, pitch, yaw)
    Piper 需求：7 维动作 (6 关节 + 1 夹爪)
    
    映射策略：
    - X/Y 平移 → 关节 1 (基座旋转)
    - Z 平移 → 关节 2, 3 (臂的升降)
    - Pitch → 关节 4, 5
    - Yaw → 关节 6
    - Roll → 不使用（Piper 不具备末端旋转）
    - 按钮 → 夹爪控制
    """
    
    def __init__(self, 
                 enable_spacemouse=True,
                 scale_factor=0.05,
                 deadzone=0.05,
                 device_name=None):
        """
        初始化 SpaceMouse 控制器
        
        Args:
            enable_spacemouse: 是否启用 SpaceMouse
            scale_factor: 动作缩放因子，控制移动速度
            deadzone: 死区，小于此值的输入会被忽略
            device_name: SpaceMouse 设备名称，None 表示自动检测
        """
        self.enable_spacemouse = enable_spacemouse and SPACEMOUSE_AVAILABLE
        self.scale_factor = scale_factor
        self.deadzone = deadzone
        
        if self.enable_spacemouse:
            try:
                self.device = open(device=device_name)
                print(f"✓ SpaceMouse 已连接: {self.device}")
            except Exception as e:
                print(f"✗ SpaceMouse 初始化失败: {e}")
                self.enable_spacemouse = False
                self.device = None
        else:
            self.device = None
    
    def read_state(self) -> Optional[SpaceNavigator]:
        """
        读取 SpaceMouse 当前状态
        
        Returns:
            SpaceNavigator 状态元组，或 None（如果未启用/未连接）
        """
        if not self.enable_spacemouse or self.device is None:
            return None
        
        state = read()
        return state
    
    def _apply_deadzone(self, value: float) -> float:
        """
        应用死区，过滤小输入
        
        Args:
            value: 输入值
            
        Returns:
            应用死区后的值
        """
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def _normalize_axis(self, value: float) -> float:
        """
        归一化轴输入到 [-1, 1] 范围
        
        Args:
            value: 原始值（通常在 [-350, 350] 范围内）
            
        Returns:
            归一化后的值 [-1, 1]
        """
        return np.clip(value / 350.0, -1.0, 1.0)
    
    def map_to_piper_action(self, 
                           spacemouse_state: Optional[SpaceNavigator]) -> np.ndarray:
        """
        将 SpaceMouse 状态映射到 Piper 的动作空间
        
        映射规则：
        - x (左右平移) → 关节 0 (基座旋转)
        - y (前后平移) → 关节 1, 2 (臂的前伸)
        - z (上下平移) → 关节 2, 3 (臂的升降)
        - pitch (俯仰) → 关节 3, 4
        - yaw (偏航) → 关节 5
        - roll (滚转) → 不使用
        - button[0] → 夹爪开/关
        
        Args:
            spacemouse_state: SpaceNavigator 状态
            
        Returns:
            7 维动作数组 [j0, j1, j2, j3, j4, j5, gripper]
        """
        if spacemouse_state is None:
            # 返回零动作
            return np.zeros(7, dtype=np.float32)
        
        # 归一化并应用死区
        x = self._apply_deadzone(self._normalize_axis(spacemouse_state.x))
        y = self._apply_deadzone(self._normalize_axis(spacemouse_state.y))
        z = self._apply_deadzone(self._normalize_axis(spacemouse_state.z))
        pitch = self._apply_deadzone(self._normalize_axis(spacemouse_state.pitch))
        yaw = self._apply_deadzone(self._normalize_axis(spacemouse_state.yaw))
        # roll 不使用
        
        # 映射到 6 个关节
        joint0 = x * self.scale_factor  # 基座旋转
        joint1 = y * self.scale_factor * 0.5  # 肩部前伸
        joint2 = (y * self.scale_factor * 0.5 + z * self.scale_factor * 0.5)  # 肘部
        joint3 = z * self.scale_factor * 0.5 + pitch * self.scale_factor * 0.3  # 腕部俯仰
        joint4 = pitch * self.scale_factor * 0.3  # 腕部偏航
        joint5 = yaw * self.scale_factor  # 腕部旋转
        
        # 夹爪控制：按钮 0 控制开/关
        if spacemouse_state.buttons and len(spacemouse_state.buttons) > 0:
            gripper = -1.0 if spacemouse_state.buttons[0] else 1.0  # 按下闭合，松开打开
        else:
            gripper = 1.0  # 默认打开
        
        action = np.array([
            joint0, joint1, joint2, joint3, joint4, joint5, gripper
        ], dtype=np.float32)
        
        return action
    
    def get_action(self) -> Tuple[np.ndarray, bool]:
        """
        获取当前动作
        
        Returns:
            (action, is_active): 
                - action: 7 维动作数组
                - is_active: 是否有用户输入（如果有任何非零输入则为 True）
        """
        state = self.read_state()
        action = self.map_to_piper_action(state)
        
        # 判断是否有有效输入
        is_active = (np.abs(action[:6]) > 0.001).any()
        
        return action, is_active
    
    def close(self):
        """关闭 SpaceMouse 连接"""
        if self.device is not None:
            close()
            self.device = None
            print("SpaceMouse 已关闭")


def test_spacemouse():
    """测试 SpaceMouse 连接和控制"""
    print("=" * 60)
    print("Piper SpaceMouse 控制器测试")
    print("=" * 60)
    
    controller = PiperSpaceMouseController()
    
    if not controller.enable_spacemouse:
        print("\n✗ SpaceMouse 不可用，请检查：")
        print("  1. SpaceMouse 是否已连接")
        print("  2. easyhid 是否已安装 (pip install easyhid)")
        return
    
    print("\n✓ SpaceMouse 已连接，开始测试...")
    print("\n控制映射：")
    print("  X 轴 (左右) → 关节 0 (基座旋转)")
    print("  Y 轴 (前后) → 关节 1, 2 (臂的前伸)")
    print("  Z 轴 (上下) → 关节 2, 3 (臂的升降)")
    print("  Pitch (俯仰) → 关节 3, 4 (腕部)")
    print("  Yaw (偏航) → 关节 5 (腕部旋转)")
    print("  按钮 0 → 夹爪开/关")
    print("\n按 Ctrl+C 退出测试\n")
    
    try:
        import time
        while True:
            action, is_active = controller.get_action()
            
            if is_active:
                print(f"动作: [{', '.join([f'{a:+.3f}' for a in action])}]")
            else:
                print(".", end="", flush=True)
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n测试结束")
    finally:
        controller.close()


if __name__ == "__main__":
    test_spacemouse()
