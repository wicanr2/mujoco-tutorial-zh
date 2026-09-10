# 訓練好的策略（policies/）

RL 章節訓練出來的權重，可以直接載入試玩，不必重新訓練。

| 檔案 | 演算法 | 最終回報 | 訓練環境 | 章節 |
| --- | --- | --- | --- | --- |
| `swingup_policy.npy` | ARS（手寫 numpy） | -0.30 | CPU，21 秒 | [08](../docs/02-programming/03-rl-swingup.md) |
| `swingup_ppo.zip` | PPO（SB3） | -0.10 | CPU，150k 步（空閒機器約 6 分鐘） | [09](../docs/02-programming/04-ppo-gymnasium.md) |
| `swingup_sac_gpu.zip` | SAC（SB3，zoo 超參） | -0.21 | RTX Pro 6000，4 分鐘、60k 步 | [12](../docs/02-programming/07-sac.md) |

三者是同一個單擺 swing-up 任務（`models/pendulum_swingup.xml`），觀測為
`[cosθ, sinθ, ω/8]`、動作為 `u ∈ [-1, 1]`，回報可以直接互相比較。

## 載入

ARS 的權重是三維線性策略，用同一組 `features()` 算控制量：

```python
import numpy as np
w = np.load("policies/swingup_policy.npy")
u = float(np.clip(w @ features(theta, omega), -1, 1))
```

SB3 的策略用 `load()`，SAC 的 GPU 權重在 CPU 上也能回放：

```python
from stable_baselines3 import PPO, SAC
model = PPO.load("policies/swingup_ppo.zip")
model = SAC.load("policies/swingup_sac_gpu.zip", device="cpu")
action, _ = model.predict(obs, deterministic=True)
```

CPU 版 SAC 沒有收斂，權重沒有保留 — 重跑 `scripts/ex_rl_sac.py` 會產生
`swingup_sac.zip`，那是未收斂的結果，用途只有對照。
