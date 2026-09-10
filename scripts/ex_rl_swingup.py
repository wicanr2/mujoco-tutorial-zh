"""RL 範例：用 Augmented Random Search（ARS）訓練單擺 swing-up。

策略：線性策略 u = w · [cosθ, sinθ, ω]，直接以 numpy 實作，
利用「一個 mjModel + 多個 mjData」做平行 episode rollout。
不依賴任何 RL 套件 — 重點是展示 MuJoCo 在訓練迴圈中的角色。
"""
import csv
import os

import mujoco
import numpy as np
from concurrent.futures import ThreadPoolExecutor

MODEL = mujoco.MjModel.from_xml_path("models/pendulum_swingup.xml")
EPISODE_STEPS = 1000          # 3 秒（timestep 0.002）
N_DIR = 8                     # 每輪探索方向數
POOL = ThreadPoolExecutor(max_workers=8)

def features(data):
    th, om = data.qpos[0], data.qvel[0]
    return np.array([np.cos(th), np.sin(th), np.clip(om, -8, 8) / 8.0])

def episode(w, seed):
    """跑一個 episode，回傳總回報。目標：擺到直立（θ=π）並停穩。"""
    data = mujoco.MjData(MODEL)
    rng = np.random.default_rng(seed)
    data.qpos[0] = rng.uniform(-0.1, 0.1)   # 從垂下附近出發
    total = 0.0
    for _ in range(EPISODE_STEPS):
        data.ctrl[0] = np.tanh(w @ features(data))
        mujoco.mj_step(MODEL, data)
        up_cost = 1.0 + np.cos(data.qpos[0])        # 0 = 直立
        total += -(up_cost + 0.05 * (data.qvel[0] / 8) ** 2
                   + 0.01 * float(data.ctrl[0]) ** 2)
    return total / EPISODE_STEPS

def evaluate(w, n=4):
    return float(np.mean([episode(w, 1000 + i) for i in range(n)]))

rng = np.random.default_rng(0)
w = np.zeros(3)
r0 = evaluate(w)
print(f"初始策略回報: {r0:.4f}")

# 每輪都評估（不是每 10 輪）—— 學習曲線要有足夠的點才看得出「什麼時候學會的」，
# 那是跟 PPO、SAC 對照的重點。每輪多跑 4 個 episode，總時間增加約四分之一。
curve = [(0, r0)]

for it in range(1, 31):
    noises = rng.normal(size=(N_DIR, 3)) * 0.5
    seeds = rng.integers(0, 10_000, size=2 * N_DIR)
    cand = [w + n for n in noises] + [w - n for n in noises]
    rewards = list(POOL.map(lambda a: episode(a[0], a[1]),
                            zip(cand, seeds)))
    rewards = np.array(rewards)
    order = np.argsort(rewards)[::-1][:N_DIR]        # 取表現較好的一半
    signs = np.where(order < N_DIR, 1.0, -1.0)[:, None]
    top = signs * noises[order % N_DIR]
    w += 0.5 * top.mean(axis=0)
    r = evaluate(w)
    curve.append((it, r))
    if it % 10 == 0:
        print(f"第 {it:2d} 輪: 評估回報 = {r:.4f}")

print(f"最終策略 w = {np.round(w, 3)}")
print(f"最終評估回報: {evaluate(w, n=8):.4f}")

os.makedirs("runs", exist_ok=True)
with open("runs/rl_ars_log.csv", "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["iteration", "reward_per_step"])
    wr.writerows(curve)
print("runs/rl_ars_log.csv 已寫出")

np.save("policies/swingup_policy.npy", w)
POOL.shutdown()
