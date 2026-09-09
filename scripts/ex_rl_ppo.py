"""RL 範例：把 MuJoCo 環境包成 Gymnasium Env，用 Stable-Baselines3 PPO 訓練單擺 swing-up。

對照 ex_rl_swingup.py（手寫 ARS）：同一個物理任務，換成標準 RL 工具鏈。
"""
import mujoco
import gymnasium as gym
import numpy as np
from pathlib import Path
from gymnasium import spaces

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv

REPO_ROOT = Path(__file__).resolve().parent.parent


class SwingUpEnv(gym.Env):
    """單擺 swing-up。觀測 [cosθ, sinθ, ω/8]，動作 u ∈ [-1, 1]。"""

    metadata = {"render_modes": []}

    def __init__(self, max_steps=500):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(str(REPO_ROOT / "models/pendulum_swingup.xml"))
        self.data = mujoco.MjData(self.model)
        self.max_steps = max_steps
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

    print("訓練前評估...")
    eval_env = SwingUpEnv()
    obs, _ = eval_env.reset(seed=42)
    pre = 0.0
    for _ in range(1000):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, term, trunc, _ = eval_env.step(action)
        pre += r
    print(f"  平均回報/步: {pre / 1000:.4f}")

    model.learn(total_timesteps=150_000, progress_bar=False)

    obs, _ = eval_env.reset(seed=42)
    post = 0.0
    for _ in range(1000):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, term, trunc, _ = eval_env.step(action)
        post += r
    print(f"訓練後平均回報/步: {post / 1000:.4f}（-0.3 以上代表穩定直立）")

    model.save("models/swingup_ppo.zip")
    print("已儲存 models/swingup_ppo.zip")
