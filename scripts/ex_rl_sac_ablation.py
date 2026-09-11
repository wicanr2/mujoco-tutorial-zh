"""SAC 在 CPU 上學不起來的真正原因：掃 train_freq × learning_rate。

12 章原本把「CPU 版沒收斂」歸給 CPU 太慢 —— 因為每步都做梯度更新太貴，所以把
`train_freq` 從 1 砍到 4。這支腳本把兩個參數各取兩個值跑滿 60k 步，看哪一個才是關鍵。
結論是 **`train_freq` 才是主因**，而且四組全部在 CPU 上、每組 90 秒內跑完 ——
「CPU 跑不動」這個前提本身就不成立。

平行環境數用 `N_ENVS` 調（預設 4）。在共用主機上別把核心佔滿。
`WITH_GPU=1` 會多跑一組 GPU 版做速度對照（需要 CUDA），寫進另一個 CSV —— 主檔固定
四列，這樣同一支腳本在有沒有 GPU 的機器上跑出來的 `rl_sac_ablation.csv` 都一樣。

輸出：runs/rl_sac_ablation.csv（＋ `WITH_GPU=1` 時的 runs/rl_sac_ablation_gpu.csv）
"""
import csv
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import torch
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv

from ex_rl_sac_gpu import SwingUpEnv

REPO_ROOT = Path(__file__).resolve().parent.parent
N_ENVS = int(os.environ.get("N_ENVS", "4"))
STEPS = 60_000


def make_env():
    return SwingUpEnv(random_start=True)


def run(train_freq, lr, device):
    """跑滿 60k 步，回傳 (耗時秒數, 最終評估值)。"""
    env = SubprocVecEnv([make_env for _ in range(N_ENVS)])
    model = SAC("MlpPolicy", env, learning_rate=lr, buffer_size=100_000,
                batch_size=256, learning_starts=500, seed=0, verbose=0,
                device=device, train_freq=(train_freq, "step"), gradient_steps=1)
    eval_env = SwingUpEnv(random_start=False)

    def evaluate():
        obs, _ = eval_env.reset(seed=42)
        total = 0.0
        for _ in range(500):
            a, _ = model.predict(obs, deterministic=True)
            obs, r, terminated, truncated, _ = eval_env.step(a)
            total += r
        return total / 500

    t0 = time.perf_counter()
    model.learn(total_timesteps=STEPS)
    elapsed = time.perf_counter() - t0
    reward = evaluate()
    env.close()
    return elapsed, reward


if __name__ == "__main__":
    # 牆鐘時間對負載極度敏感 —— 量之前先把 load 記下來，數字才有得比。
    load1 = os.getloadavg()[0]
    print(f"平行環境數 = {N_ENVS}；開跑 load average = {load1:.2f}", flush=True)

    cases = [
        ("cpu", 4, 3e-4, "12 章原設定"),
        ("cpu", 4, 1e-3, "只調高 lr"),
        ("cpu", 1, 3e-4, "只放開 train_freq"),
        ("cpu", 1, 1e-3, "zoo 設定（兩個都改）"),
    ]
    if os.environ.get("WITH_GPU") == "1":
        if not torch.cuda.is_available():
            sys.exit("WITH_GPU=1 但這台沒有可用的 CUDA")
        cases.append(("cuda", 1, 1e-3, "zoo 設定（GPU 對照）"))

    rows, gpu_rows = [], []
    for device, tf, lr, label in cases:
        elapsed, reward = run(tf, lr, device)
        row = [device, tf, lr, round(elapsed, 1), round(reward, 4)]
        (gpu_rows if device != "cpu" else rows).append(row)
        print(f"  {label:22s} device={device:4s} train_freq={tf} lr={lr:<7g}"
              f" → {elapsed:5.1f} 秒、最終 {reward:.4f}", flush=True)

    os.makedirs(REPO_ROOT / "runs", exist_ok=True)
    header = ["device", "train_freq", "learning_rate", "seconds", "final_reward"]
    with open(REPO_ROOT / "runs/rl_sac_ablation.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print("runs/rl_sac_ablation.csv 已寫出", flush=True)
    if gpu_rows:
        with open(REPO_ROOT / "runs/rl_sac_ablation_gpu.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(gpu_rows)
        print("runs/rl_sac_ablation_gpu.csv 已寫出", flush=True)
    print(f"結束 load average = {os.getloadavg()[0]:.2f}", flush=True)

    # 驗收對準「真正想證明的事」：放開 train_freq 就收斂，砍掉它就躺平 —— 兩者都在 CPU 上。
    # key 要帶 lr：只用 (device, train_freq) 會被同 train_freq 的另一列蓋掉，
    # 驗收就變成在檢查別組的數字。
    by_key = {(d, tf, lr): r for d, tf, lr, s, r in rows}
    assert len(by_key) == len(rows), "驗收用的 key 撞號了"
    assert by_key[("cpu", 4, 3e-4)] < -1.0, "原設定應該躺平（≈ -2），這次卻收斂了"
    assert by_key[("cpu", 1, 3e-4)] > -0.5, "只放開 train_freq 就該收斂到 -0.5 以上"
    assert by_key[("cpu", 1, 1e-3)] > -0.5, "zoo 設定在 CPU 上也該收斂"
    print("結果：CPU 上放開 train_freq 就學得起來，瓶頸不在算力 ✓")
