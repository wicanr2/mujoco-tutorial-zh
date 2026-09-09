"""RL 範例：同一個 swing-up 任務改用 SAC（off-policy）。

對照 ex_rl_ppo.py：環境直接重用 SwingUpEnv，只換演算法。
SAC 是 off-policy，樣本效率高，通常比 PPO 更快學會。
"""
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv

from ex_rl_ppo import SwingUpEnv


def make_env():
    # 隨機初始狀態：讓 replay buffer 收錄靠近直立的經驗，解決冷啟動探索問題
    return SwingUpEnv(random_start=True)


if __name__ == "__main__":
    env = SubprocVecEnv([make_env for _ in range(4)])

    model = SAC("MlpPolicy", env, learning_rate=3e-4,
                buffer_size=50_000, batch_size=256, verbose=0, seed=0,
                train_freq=(4, "step"), gradient_steps=1)  # 每 4 步才更新，CPU 友善

    eval_env = SwingUpEnv()

    def evaluate():
        obs, _ = eval_env.reset(seed=42)
        total = 0.0
        for _ in range(500):
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, _ = eval_env.step(action)
            total += r
        return total / 500

    print(f"訓練前評估: {evaluate():.4f}", flush=True)
    # 分階段訓練，每 3000 步報告一次（SAC 每步都做梯度更新，CPU 上較慢）
    for i in range(20):
        model.learn(total_timesteps=3_000, reset_num_timesteps=False)
        print(f"  {(i + 1) * 3000} 步後評估: {evaluate():.4f}", flush=True)
    print("（-0.3 以上代表穩定直立）", flush=True)

    model.save("models/swingup_sac.zip")
    print("已儲存 models/swingup_sac.zip")
