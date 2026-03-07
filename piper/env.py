import numpy as np
from gym import spaces
from dm_env import StepType, specs
from typing import Any, NamedTuple
from collections import deque
from piper.robot import PiperRobot


class NormalizeAction:
    def __init__(self, env, key="action"):
        self._env = env
        self._key = key
        space = env.act_space[key]
        self._mask = np.isfinite(space.low) & np.isfinite(space.high)
        self._low = np.where(self._mask, space.low, -1)
        self._high = np.where(self._mask, space.high, 1)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        try:
            return getattr(self._env, name)
        except AttributeError:
            raise ValueError(name)

    @property
    def act_space(self):
        low = np.where(self._mask, -np.ones_like(self._low), self._low)
        high = np.where(self._mask, np.ones_like(self._low), self._high)
        space = spaces.Box(low, high, dtype=np.float32)
        return {**self._env.act_space, self._key: space}

    def step(self, action):
        orig = (action[self._key] + 1) / 2 * (self._high - self._low) + self._low
        orig = np.where(self._mask, orig, action[self._key])
        return self._env.step({**action, self._key: orig})
    
    def reset(self):
        return self._env.reset()


class TimeLimit:
    def __init__(self, env, duration):
        self._env = env
        self._duration = duration
        self._step = None

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        try:
            return getattr(self._env, name)
        except AttributeError:
            raise ValueError(name)

    def step(self, action):
        assert self._step is not None, "Must reset environment."
        obs = self._env.step(action)
        self._step += 1
        if self._duration and self._step >= self._duration:
            obs["is_last"] = True
            obs["TimeLimit.truncated"] = True
            self._step = None
        return obs

    def reset(self):
        self._step = 0
        return self._env.reset()


class ExtendedTimeStep(NamedTuple):
    step_type: Any
    reward: Any
    discount: Any
    observation: Any
    action: Any
    success: Any

    def first(self):
        return self.step_type == StepType.FIRST

    def mid(self):
        return self.step_type == StepType.MID

    def last(self):
        return self.step_type == StepType.LAST

    def __getitem__(self, attr):
        if isinstance(attr, str):
            return getattr(self, attr)
        else:
            return tuple.__getitem__(self, attr)


class PiperEnv:
    def __init__(self, task_name, seed=None, action_repeat=1, 
                 size=(256, 256), use_sim=True):
        self.task_name = task_name
        self._size = size
        self._action_repeat = action_repeat
        
        np.random.seed(seed)
        
        self.robot = PiperRobot(use_sim=use_sim, 
                                camera_width=size[0], 
                                camera_height=size[1])
        
        self.observation_space = spaces.Box(
            low=0, high=255, 
            shape=size + (3,), 
            dtype=np.uint8
        )
        self.action_space = spaces.Box(
            low=-1, high=1, 
            shape=(6,), 
            dtype=np.float32
        )
        
        self.step_count = 0
        self.obj_init_pos = np.array([0.0, 0.6, 0.0])
    
    @property
    def obs_space(self):
        return {
            "image": self.observation_space,
            "reward": spaces.Box(-np.inf, np.inf, (), dtype=np.float32),
            "is_first": spaces.Box(0, 1, (), dtype=bool),
            "is_last": spaces.Box(0, 1, (), dtype=bool),
            "is_terminal": spaces.Box(0, 1, (), dtype=bool),
            "success": spaces.Box(0, 1, (), dtype=bool),
        }
    
    @property
    def act_space(self):
        return {"action": self.action_space}
    
    def _compute_reward(self, action, obs=None):
        tcp_pos = self.robot.get_end_effector_pos()
        obj_pos = self.robot.get_obj_pos()
        target_pos = self.robot.get_goal_pos()
        
        scale = np.array([2., 2., 1.])
        target_to_obj = (obj_pos - target_pos) * scale
        target_to_obj = np.linalg.norm(target_to_obj)
        
        obj_init_pos = self.obj_init_pos
        target_to_obj_init = (obj_init_pos - target_pos) * scale
        target_to_obj_init = np.linalg.norm(target_to_obj_init)
        
        in_place = self._tolerance(
            target_to_obj,
            bounds=(0, 0.05),
            margin=target_to_obj_init,
            sigmoid='long_tail'
        )
        
        tcp_to_obj = np.linalg.norm(obj_pos - tcp_pos)
        
        object_grasped = self._gripper_caging_reward(
            action,
            obj_pos,
            tcp_pos,
            object_reach_radius=0.04,
            obj_radius=0.02,
            pad_success_thresh=0.05,
            xz_thresh=0.05
        )
        
        reward = self._hamacher_product(object_grasped, in_place)
        
        if tcp_to_obj < 0.04:
            reward += 1.0 + 5.0 * in_place
        
        obj_to_target = np.linalg.norm(obj_pos - target_pos)
        if obj_to_target < 0.05:
            reward = 10.0
        
        success = float(obj_to_target <= 0.07)
        
        return reward, success, obj_to_target
    
    def _tolerance(self, x, bounds=(0, 0), margin=0, sigmoid='gaussian'):
        lower, upper = bounds
        in_bounds = (lower <= x) & (x <= upper)
        
        if sigmoid == 'gaussian':
            d = np.maximum(x - upper, lower - x)
            out_of_bounds = np.exp(-(d ** 2) / (2 * margin ** 2))
        elif sigmoid == 'long_tail':
            d = np.maximum(x - upper, lower - x)
            out_of_bounds = 1 / (1 + (d / margin) ** 2)
        elif sigmoid == 'tanh':
            d = np.maximum(x - upper, lower - x)
            out_of_bounds = (1 - np.tanh(d / margin)) / 2
        else:
            raise ValueError(f"Unknown sigmoid type: {sigmoid}")
        
        return np.where(in_bounds, 1.0, out_of_bounds)
    
    def _hamacher_product(self, a, b):
        return a * b / (a + b - a * b + 1e-8)
    
    def _gripper_caging_reward(self, action, obj_pos, tcp_pos,
                                object_reach_radius=0.04, obj_radius=0.02,
                                pad_success_thresh=0.05, xz_thresh=0.05):
        tcp_to_obj = np.linalg.norm(obj_pos - tcp_pos)
        
        in_object_reach = float(tcp_to_obj < object_reach_radius)
        object_reach_reward = self._tolerance(
            tcp_to_obj,
            bounds=(0, object_reach_radius),
            margin=0.1
        )
        
        return object_reach_reward
    
    def reset(self):
        self.robot.reset()
        self.step_count = 0
        
        if np.random.random() < 0.3:
            obj_x = np.random.uniform(-0.1, 0.1)
            obj_y = np.random.uniform(0.55, 0.65)
            self.robot.set_obj_pos([obj_x, obj_y, 0.0])
            
            goal_x = np.random.uniform(-0.05, 0.05)
            goal_y = np.random.uniform(0.7, 0.75)
            while np.linalg.norm([obj_x - goal_x, obj_y - goal_y]) < 0.15:
                goal_x = np.random.uniform(-0.05, 0.05)
                goal_y = np.random.uniform(0.7, 0.75)
            self.robot.set_goal_pos([goal_x, goal_y, 0.0])
        
        self.obj_init_pos = self.robot.get_obj_pos().copy()
        
        obs = self.robot.get_camera_image()
        
        return {
            "reward": 0.0,
            "is_first": True,
            "is_last": False,
            "is_terminal": False,
            "image": obs,
            "success": False
        }
    
    def step(self, action):
        assert np.isfinite(action["action"]).all(), action["action"]
        total_reward = 0.0
        success = 0.0
        
        for _ in range(self._action_repeat):
            self.robot.step(action["action"])
            reward, suc, _ = self._compute_reward(action["action"])
            success += float(suc)
            total_reward += reward or 0.0
        
        success = min(success, 1.0)
        assert success in [0.0, 1.0]
        
        obs = self.robot.get_camera_image()
        
        return {
            "reward": total_reward,
            "is_first": False,
            "is_last": False,
            "is_terminal": False,
            "image": obs,
            "success": success
        }
    
    def render(self):
        return self.robot.get_camera_image()
    
    def close(self):
        self.robot.close()


class PiperWrapper:
    def __init__(self, env, frame_stack=3):
        self._env = env
        self.frame_stack = frame_stack
        wos = env.obs_space['image']
        low = np.repeat(wos.low, self.frame_stack, axis=-1)
        high = np.repeat(wos.high, self.frame_stack, axis=-1)
        self.stackedobs = np.zeros(low.shape, low.dtype)

        self.observation_space = spaces.Box(
            low=np.transpose(low, (2, 0, 1)), 
            high=np.transpose(high, (2, 0, 1)), 
            dtype=np.uint8
        )

    def observation_spec(self):
        return specs.BoundedArray(
            self.observation_space.shape,
            np.uint8,
            0,
            255,
            name='observation'
        )

    def action_spec(self):
        return specs.BoundedArray(
            self._env.act_space['action'].shape,
            np.float32,
            self._env.act_space['action'].low,
            self._env.act_space['action'].high,
            'action'
        )

    def reset(self):
        time_step = self._env.reset()
        obs = time_step['image']
        self.stackedobs[...] = 0
        self.stackedobs[..., -obs.shape[-1]:] = obs
        return ExtendedTimeStep(
            observation=np.transpose(self.stackedobs, (2, 0, 1)),
            step_type=StepType.FIRST,
            action=np.zeros(self.action_spec().shape, dtype=self.action_spec().dtype),
            reward=0.0,
            discount=1.0,
            success=time_step['success']
        )
    
    def step(self, action):
        action = {'action': action}
        time_step = self._env.step(action)
        obs = time_step['image']
        self.stackedobs = np.roll(self.stackedobs, shift=-obs.shape[-1], axis=-1)
        self.stackedobs[..., -obs.shape[-1]:] = obs

        if time_step['is_first']:
            step_type = StepType.FIRST
        elif time_step['is_last']:
            step_type = StepType.LAST
        else:
            step_type = StepType.MID
        
        return ExtendedTimeStep(
            observation=np.transpose(self.stackedobs, (2, 0, 1)),
            step_type=step_type,
            action=action['action'],
            reward=time_step['reward'],
            discount=1.0,
            success=time_step['success']
        )
    
    def render(self):
        latest_frame = self.stackedobs[..., -3:]
        return latest_frame
    
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        try:
            return getattr(self._env, name)
        except AttributeError:
            raise ValueError(name)


def make(name, frame_stack, action_repeat, seed, use_sim=True):
    env = PiperEnv(name, seed, action_repeat, (256, 256), use_sim=use_sim)
    env = NormalizeAction(env)
    env = TimeLimit(env, 250)
    env = PiperWrapper(env, frame_stack)
    return env
