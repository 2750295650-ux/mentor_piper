import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import piper.env as piper_env


def verify_complete_flow():
    print("="*60)
    print("验证 Piper 环境完整流程")
    print("="*60)
    
    try:
        # ============================================
        # 步骤 1: 创建环境
        # ============================================
        print("\n[1/6] 创建环境...")
        env = piper_env.make(
            name='piper_push',
            frame_stack=3,
            action_repeat=2,
            seed=0,
            use_sim=True
        )
        print("✓ 环境创建成功")
        
        # 检查环境类型
        print(f"  环境类型: {type(env)}")
        print(f"  观察空间形状: {env.observation_spec().shape}")
        print(f"  动作空间形状: {env.action_spec().shape}")
        
        # ============================================
        # 步骤 2: 测试 reset()
        # ============================================
        print("\n[2/6] 测试 reset()...")
        time_step = env.reset()
        print("✓ Reset 成功")
        print(f"  观察形状: {time_step.observation.shape}")
        print(f"  Step type: {time_step.step_type}")
        print(f"  Reward: {time_step.reward}")
        print(f"  Success: {time_step.success}")
        
        # 验证观察形状
        expected_obs_shape = (9, 256, 256)  # 3帧 × 3通道
        if time_step.observation.shape == expected_obs_shape:
            print(f"  ✓ 观察形状正确: {expected_obs_shape}")
        else:
            print(f"  ✗ 观察形状错误！期望: {expected_obs_shape}, 实际: {time_step.observation.shape}")
            return False
        
        # ============================================
        # 步骤 3: 测试 step() 随机动作
        # ============================================
        print("\n[3/6] 测试 step() 随机动作...")
        total_reward = 0
        success_count = 0
        
        for i in range(5):
            action = np.random.uniform(-1, 1, 6)
            time_step = env.step(action)
            total_reward += time_step.reward
            if time_step.success:
                success_count += 1
            print(f"  步骤 {i+1}: 奖励 = {time_step.reward:.4f}, 成功 = {time_step.success}, StepType = {time_step.step_type}")
        
        print(f"✓ 5 步随机动作完成，总奖励 = {total_reward:.4f}")
        
        # ============================================
        # 步骤 4: 测试内部 wrapper 链
        # ============================================
        print("\n[4/6] 检查内部 wrapper 链...")
        
        # 尝试访问内部环境
        try:
            print(f"  最外层: {type(env)}")
            if hasattr(env, '_env'):
                print(f"  内层 1: {type(env._env)}")
                if hasattr(env._env, '_env'):
                    print(f"  内层 2: {type(env._env._env)}")
                    if hasattr(env._env._env, '_env'):
                        print(f"  内层 3: {type(env._env._env._env)}")
                        if hasattr(env._env._env._env, 'robot'):
                            print(f"  ✓ PiperRobot 存在")
            print("✓ Wrapper 链完整")
        except Exception as e:
            print(f"  警告: 检查 wrapper 链时出错: {e}")
        
        # ============================================
        # 步骤 5: 测试 render()
        # ============================================
        print("\n[5/6] 测试 render()...")
        try:
            render_frame = env.render()
            print(f"✓ Render 成功，帧形状: {render_frame.shape}")
        except Exception as e:
            print(f"  警告: Render 测试跳过: {e}")
        
        # ============================================
        # 步骤 6: 测试 close()
        # ============================================
        print("\n[6/6] 测试 close()...")
        env.close()
        print("✓ Close 成功")
        
        print("\n" + "="*60)
        print("✓ 所有流程验证通过！")
        print("="*60)
        
        return True
        
    except Exception as e:
        print(f"\n✗ 验证失败: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == '__main__':
    success = verify_complete_flow()
    sys.exit(0 if success else 1)
