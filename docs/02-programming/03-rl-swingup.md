# 08｜RL 訓練範例：單擺 Swing-up（ARS 隨機搜索）

> 參考來源：[MuJoCo Overview — Performance / multi-threading](https://mujoco.readthedocs.io/en/stable/overview.html)、[API Reference](https://mujoco.readthedocs.io/en/stable/APIreference/)；ARS 演算法出自論文 [Simple random search of static linear policies (Mania et al., 2018)](https://arxiv.org/abs/1803.07055)
> 擷取日期：2026-09-09
> 測試環境：Linux、MuJoCo 3.12.0、Python 3.12.3；已實測，約 21 秒完成 30 輪訓練。

## 學習目標

- 理解 MuJoCo 在 RL 訓練迴圈中的角色（環境 / 動力學）
- 用純 numpy 實作一個可運作的 RL 訓練迴圈
- 實際把單擺從垂下盪到直立並穩住

## 前置知識

- [03｜程式設計入門](01-simulation-loop.md)、[07｜更多 Python 範例](02-more-examples.md)（keyframe 重置、平行 rollout）

## 為什麼用 ARS 而不是 PPO/DQN？

本章的目標是展示 **MuJoCo 與訓練迴圈的整合方式**，不是 RL 演算法本身。Augmented Random Search（ARS）：

- 只需「跑 episode、記回報」這一個介面，十幾行就能實作；
- 不需要梯度、不需要神經網路套件；
- 對低維線性策略意外地有效。

同樣的迴圈結構可直接替換成 PPO/SAC 等演算法（例如搭配 Gymnasium 介面）。

## 任務設計

模型 `models/pendulum_swingup.xml`：單擺 + `gear=3` 的 motor（力矩要夠大才盪得起來）。

- **狀態特徵**：`[cosθ, sinθ, ω/8]`（用 cos/sin 避免角度在 ±π 處的不連續）
- **策略**：線性策略 `u = tanh(w · 特徵)`，只有 3 個參數
- **回報**：`-(1+cosθ)`（直立為 0）`- 0.05·(ω/8)² - 0.01·u²`，鼓勵「上去而且停穩、省力」
- **初始狀態**：垂下附近加小隨機擾動

## 訓練迴圈（`scripts/ex_rl_swingup.py`）

```python
def episode(w, seed):
    data = mujoco.MjData(MODEL)        # 每個 episode 自己的 mjData
    rng = np.random.default_rng(seed)
    data.qpos[0] = rng.uniform(-0.1, 0.1)
    total = 0.0
    for _ in range(EPISODE_STEPS):
        data.ctrl[0] = np.tanh(w @ features(data))
        mujoco.mj_step(MODEL, data)
        total += reward(data)
    return total / EPISODE_STEPS
```

每輪對 `w` 加 8 個隨機正/負方向擾動，平行跑 16 個 episode，取表現較好的一半方向平均後更新 `w`：

```python
for it in range(30):
    noises = rng.normal(size=(N_DIR, 3)) * 0.5
    cand = [w + n for n in noises] + [w - n for n in noises]
    rewards = list(POOL.map(episode, cand))   # 多執行緒 rollout
    order = np.argsort(rewards)[::-1][:N_DIR]
    w += 0.5 * (signs * noises[order % N_DIR]).mean(axis=0)
```

實測輸出：

```
初始策略回報: -1.9998          ← 完全躺平
第 10 輪: 評估回報 = -0.3816
第 20 輪: 評估回報 = -0.3219
第 30 輪: 評估回報 = -0.2958
最終策略 w = [ 0.975  2.993 -1.328]
最終評估回報: -0.3040          ← 大部分時間維持直立附近
```

約 21 秒完成訓練；訓練好的權重存為 `models/swingup_policy.npy`。

## 除錯紀錄（真實發生在本章開發過程）

1. **第一版完全沒學習**（回報卡在 -2.0）：探索噪音 0.1、學習率 0.05 太小，30 輪只動了 0.03。調大到噪音 0.5、步長 0.5 後立刻收斂 — **隨機搜索類演算法對探索尺度很敏感**。
2. **廣播錯誤**：`np.where` 的布林條件要 reshape 成 `(N,1)` 才能乘上向量。
3. **力矩不足**：`gear=2` 時最大力矩約 1 N·m，接近單擺所需的重力矩，學習很慢；調到 `gear=3` 才順利 swing-up — RL 學不起來時先檢查**硬體能力上限**。

## 常見錯誤與除錯

- **回報不動**：先印出 episode 中的狀態範圍，確認策略真的影響了系統（ctrl 是否被 range 限死）。
- **多執行緒沒有加速**：Python 有 GIL，速度取決於 `mj_step` 釋放 GIL 的程度；更大規模請改用 `multiprocessing` 或 GPU 後端（MJX / MuJoCo Warp）。
- **每次結果不同**：固定 `seed` 可重現；評估時用一組獨立種子避免過擬合訓練種子。
- **要上真機**：把 `models/swingup_policy.npy` 載進部署程式，用同一個 `features()` 函式計算控制量即可。

## 延伸閱讀

- [Simple random search of static linear policies（ARS 論文）](https://arxiv.org/abs/1803.07055)
- [MuJoCo Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)
- 下一步：把 `episode()` 包成 Gymnasium `Env`，接 Stable-Baselines3 的 PPO
