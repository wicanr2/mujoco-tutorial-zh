"""RL 範例：同一個 swing-up 任務改用 SAC（off-policy）。

對照 ex_rl_ppo.py：環境直接重用 SwingUpEnv，只換演算法。
SAC 是 off-policy，樣本效率高，通常比 PPO 更快學會。
"""
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv

from ex_rl_ppo import SwingUpEnv


def make_env():
    return SwingUpEnv()


if __name__ == "__main__":
    env = SubprocVecEnv([make_env for _ in range(4)])

    model = SAC("MlpPolicy", env, learning_rate=3e-4,
                buffer_size=100_000, batch_size=256, verbose=0, seed=0)

    eval_env = SwingUpEnv()

    def evaluate():
        obs, _ = eval_env.reset(seed=42)
        total = 0.0
        for _ in range(500):
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, _ = eval_env.step(action)
            total += r
        return total / 500

    print(f"訓練前評估: {evaluate():.4f}")
    model.learn(total_timesteps=30_000, progress_bar=False)
    print(f"訓練後評估: {evaluate():.4f}（-0.3 以上代表穩定直立）")

    model.save("models/swingup_sac.zip")
    print("已儲存 models/swingup_sac.zip")
