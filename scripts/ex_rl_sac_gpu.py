"""GPU 版 SAC 訓練（遠端 RTX6000 用）。

與 ex_rl_sac.py 的差異：
- 直接使用 SwingUpEnv（random_start=True），不依賴本地相對路徑
- SAC 用 zoo 風格超參：train_freq=1、gradient_steps=1、lr=1e-3
- device='cuda'
"""
import sys
sys.path.insert(0, ".")  # 讓 import 找到 ex_rl_ppo

import csv
import os

import mujoco
import gymnasium as gym
import numpy as np
from pathlib import Path
from gymnasium import spaces
from stable_baselines3 import SAC

REPO_ROOT = Path(__file__).resolve().parent.parent
from stable_baselines3.common.vec_env import SubprocVecEnv


class SwingUpEnv(gym.Env):
    def __init__(self, max_steps=500, random_start=True):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(str(REPO_ROOT / "models/pendulum_swingup.xml"))
        self.data = mujoco.MjData(self.model)
        self.max_steps = max_steps
        self.random_start = random_start
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)
        self._steps = 0

    def _obs(self):
        th, om = self.data.qpos[0], self.data.qvel[0]
        return np.array([np.cos(th), np.sin(th), np.clip(om, -8, 8) / 8.0],
                        dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        if self.random_start:
            self.data.qpos[0] = self.np_random.uniform(-np.pi, np.pi)
            self.data.qvel[0] = self.np_random.uniform(-2.0, 2.0)
        else:
            self.data.qpos[0] = self.np_random.uniform(-0.1, 0.1)
        self._steps = 0
        return self._obs(), {}

    def step(self, action):
        self.data.ctrl[0] = float(action[0])
        for _ in range(2):
            mujoco.mj_step(self.model, self.data)
        self._steps += 1
        up_cost = 1.0 + np.cos(self.data.qpos[0])
        reward = -(up_cost + 0.05 * (self.data.qvel[0] / 8) ** 2
                   + 0.01 * float(action[0]) ** 2)
        return self._obs(), reward, False, self._steps >= self.max_steps, {}


def make_env():
    return SwingUpEnv(random_start=True)


if __name__ == "__main__":
    # 平行環境數。預設 8（12 章的原始設定）；在共用主機上跑時用 N_ENVS 調低，
    # 別把整台機器的 CPU 佔滿 —— GPU 版的瓶頸在梯度更新，不在取樣。
    n_envs = int(os.environ.get("N_ENVS", "8"))
    print(f"平行環境數 = {n_envs}", flush=True)
    env = SubprocVecEnv([make_env for _ in range(n_envs)])
    model = SAC("MlpPolicy", env, learning_rate=1e-3, buffer_size=100_000,
                batch_size=256, learning_starts=500, seed=0, verbose=0,
                device="cuda")

    eval_env = SwingUpEnv(random_start=False)

    def evaluate():
        obs, _ = eval_env.reset(seed=42)
        total = 0.0
        for _ in range(500):
            a, _ = model.predict(obs, deterministic=True)
            obs, r, te, tr, _ = eval_env.step(a)
            total += r
        return total / 500

    r0 = evaluate()
    print(f"訓練前評估: {r0:.4f}", flush=True)
    curve = [(0, r0)]
    for i in range(20):
        model.learn(total_timesteps=3_000, reset_num_timesteps=False)
        r = evaluate()
        curve.append(((i + 1) * 3000, r))
        print(f"  {(i + 1) * 3000} 步後評估: {r:.4f}", flush=True)

    os.makedirs(REPO_ROOT / "runs", exist_ok=True)
    with open(REPO_ROOT / "runs/rl_sac_gpu_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["steps", "reward_per_step"])
        w.writerows(curve)
    print("runs/rl_sac_gpu_log.csv 已寫出", flush=True)

    model.save(str(REPO_ROOT / "policies/swingup_sac_gpu.zip"))
    print("已儲存 policies/swingup_sac_gpu.zip", flush=True)
