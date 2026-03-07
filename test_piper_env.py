import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import piper.env as piper_env


def test_env():
    print("="*50)
    print("测试 Piper 环境")
    print("="*50)
    
    try:
        print("\n1. 创建环境 (模拟模式)...")
        env = piper_env.make(
            name='piper_push',
            frame_stack=3,
            action_repeat=2,
            seed=0,
            use_sim=True
        )
        print("✓ 环境创建成功")
        
        print(f"\n2. 观察空间形状: {env.observation_spec().shape}")
        print(f"   动作空间形状: {env.action_spec().shape}")
        
        print("\n3. 测试 reset()...")
        time_step = env.reset()
        print(f"✓ Reset 成功")
        print(f"   观察形状: {time_step.observation.shape}")
        print(f"   Step type: {time_step.step_type}")
        
        print("\n4. 测试 step() 随机动作...")
        total_reward = 0
        for i in range(10):
            action = np.random.uniform(-1, 1, 6)
            time_step = env.step(action)
            total_reward += time_step.reward
            print(f"   步骤 {i+1}: 奖励 = {time_step.reward:.4f}, 成功 = {time_step.success}")
        
        print(f"\n✓ 10 步随机动作完成，总奖励 = {total_reward:.4f}")
        
        print("\n" + "="*50)
        print("✓ 所有测试通过！环境正常工作")
        print("="*50)
        
        env.close()
        
    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True


if __name__ == '__main__':
    success = test_env()
    sys.exit(0 if success else 1)
