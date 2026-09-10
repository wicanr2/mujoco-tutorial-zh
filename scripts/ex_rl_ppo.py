"""RL 範例：把 MuJoCo 環境包成 Gymnasium Env，用 Stable-Baselines3 PPO 訓練單擺 swing-up。

對照 ex_rl_swingup.py（手寫 ARS）：同一個物理任務，換成標準 RL 工具鏈。
"""
import mujoco
import gymnasium as gym
import csv
import os

import numpy as np
from pathlib import Path
from gymnasium import spaces

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv

REPO_ROOT = Path(__file__).resolve().parent.parent


class SwingUpEnv(gym.Env):
    """單擺 swing-up。觀測 [cosθ, sinθ, ω/8]，動作 u ∈ [-1, 1]。"""

    metadata = {"render_modes": []}

    def __init__(self, max_steps=500, random_start=False):
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
            # 全域隨機初始化：讓 replay buffer 見過各種角度（SAC 冷啟動用）
            self.data.qpos[0] = self.np_random.uniform(-np.pi, np.pi)
            self.data.qvel[0] = self.np_random.uniform(-2.0, 2.0)
        else:
            self.data.qpos[0] = self.np_random.uniform(-0.1, 0.1)
        self._steps = 0
        return self._obs(), {}

    def step(self, action):
        self.data.ctrl[0] = float(action[0])
        for _ in range(2):  # frame skip：一個動作維持 2 個模擬步
            mujoco.mj_step(self.model, self.data)
        self._steps += 1
        up_cost = 1.0 + np.cos(self.data.qpos[0])
        reward = -(up_cost + 0.05 * (self.data.qvel[0] / 8) ** 2
                   + 0.01 * float(action[0]) ** 2)
        terminated = False
        truncated = self._steps >= self.max_steps
        return self._obs(), reward, terminated, truncated, {}


def make_env():
    return SwingUpEnv()


if __name__ == "__main__":
    # 4 個平行環境（SubprocVecEnv 用子行程繞過 GIL）
    env = SubprocVecEnv([make_env for _ in range(4)])

    model = PPO("MlpPolicy", env, n_steps=512, batch_size=256,
                learning_rate=3e-4, verbose=0, seed=0)

    eval_env = SwingUpEnv()

    def evaluate(n=1000):
        obs, _ = eval_env.reset(seed=42)
        total = 0.0
        for _ in range(n):
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, _ = eval_env.step(action)
            total += r
        return total / n

    # 分段訓練，每段評估一次 —— 只量訓練前後看不出「什麼時候學會的」，
    # 而那正是拿來跟 ARS、SAC 對照的東西（見 runs/rl_curves.png）。
    STEP = 10_000
    TOTAL = 150_000
    pre = evaluate()
    print("訓練前評估...")
    print(f"  平均回報/步: {pre:.4f}", flush=True)
    curve = [(0, pre)]
    for i in range(TOTAL // STEP):
        model.learn(total_timesteps=STEP, reset_num_timesteps=False, progress_bar=False)
        r = evaluate()
        curve.append(((i + 1) * STEP, r))
        print(f"  {(i + 1) * STEP} 步後評估: {r:.4f}", flush=True)

    print(f"訓練後平均回報/步: {curve[-1][1]:.4f}（-0.3 以上代表穩定直立）")

    os.makedirs("runs", exist_ok=True)
    with open("runs/rl_ppo_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["steps", "reward_per_step"])
        w.writerows(curve)
    print("runs/rl_ppo_log.csv 已寫出")

    model.save("policies/swingup_ppo.zip")
    print("已儲存 policies/swingup_ppo.zip")
