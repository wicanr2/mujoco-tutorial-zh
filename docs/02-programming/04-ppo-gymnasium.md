# 09｜接軌標準 RL 工具鏈：Gymnasium + Stable-Baselines3 PPO

> 參考來源：[Gymnasium 文件](https://gymnasium.farama.org/)、[Stable-Baselines3 文件](https://stable-baselines3.readthedocs.io/)、[PPO 論文（Schulman et al., 2017）](https://arxiv.org/abs/1707.06347)
>
> 擷取日期：2026-09-09
>
> 相依套件：`pip install gymnasium stable-baselines3`（實測 sb3 2.9.0、gymnasium 1.3.0、torch 2.14.0，Linux）

## 學習目標

- 把 MuJoCo 環境包成 Gymnasium 標準 `Env` 介面
- 用 Stable-Baselines3 的 PPO 訓練同一個單擺 swing-up 任務
- 理解 frame skip、向量化環境等實務技巧

## 前置知識

- [08｜RL 訓練範例：單擺 Swing-up（ARS）](03-rl-swingup.md) — 本章是它的「換工具」版：物理任務、回報函數完全相同

## 為什麼要包 Gymnasium Env？

手寫 ARS 能解決簡單任務，但當任務變難（雙擺、四足行走），你會想要 PPO / SAC 等現代演算法。Gymnasium 是 Python RL 生態系的標準環境介面：

```python
obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(action)
```

只要實作這個介面，Stable-Baselines3、Ray RLlib、CleanRL 等所有工具都能直接用。

## 實作（`scripts/ex_rl_ppo.py`）

```python
class SwingUpEnv(gym.Env):
    def __init__(self, max_steps=500):
        self.model = mujoco.MjModel.from_xml_path("models/pendulum_swingup.xml")
        self.data = mujoco.MjData(self.model)
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)

    def step(self, action):
        self.data.ctrl[0] = float(action[0])
        for _ in range(2):                    # frame skip：一個動作維持 2 個模擬步
            mujoco.mj_step(self.model, self.data)
        up_cost = 1.0 + np.cos(self.data.qpos[0])
        reward = -(up_cost + 0.05 * (self.data.qvel[0] / 8) ** 2
                   + 0.01 * float(action[0]) ** 2)
        truncated = self._steps >= self.max_steps
        return self._obs(), reward, False, truncated, {}
```

重點：

- **frame skip**：RL 慣例。控制頻率通常比物理頻率低（這裡 500 Hz 物理、250 Hz 控制），也讓每個動作的影響更明顯、加速學習。
- **觀測用 cos/sin**：同 ARS 章，避免角度不連續。
- **`terminated` vs `truncated`**：任務沒有失敗條件，所以永遠 `terminated=False`，時間上限走 `truncated` — 兩者在 bootstrapping 上的處理不同，不要混用。

## 訓練

```python
env = SubprocVecEnv([make_env for _ in range(4)])   # 4 個子行程平行跑環境
model = PPO("MlpPolicy", env, n_steps=512, batch_size=256,
            learning_rate=3e-4, verbose=0, seed=0)
model.learn(total_timesteps=150_000)
```

- **SubprocVecEnv**：用子行程而非執行緒，繞過 GIL 限制（對照 [07 章](02-more-examples.md) ThreadPoolExecutor 的討論）。
- 訓練後 `model.save("policies/swingup_ppo.zip")` 儲存策略，`PPO.load()` 可在部署端使用。

## 實測結果

> 測試環境：Linux、Python 3.12.3、MuJoCo 3.12.0、sb3 2.9.0、torch 2.14.0。

```
訓練前平均回報/步: -1.9997        ← 完全躺平
  10000 步後評估: -1.9804
  ...
 140000 步後評估: -0.1150
訓練後平均回報/步: -0.1035        ← 穩定直立（比 ARS 的 -0.30 更好）
```

150k timesteps、4 個平行環境。策略存為 `policies/swingup_ppo.zip`（已收錄於 repo，可直接
`PPO.load()` 載入試玩），每 10k 步的評估寫進
[runs/rl_ppo_log.csv](../../runs/rl_ppo_log.csv)。

訓練時間在空閒的機器上約 6 分鐘；這次重跑時本機 load average 超過 20（有別的工作在跑），
同樣的 150k 步花了 30 分鐘。**牆鐘時間這種數字要連量測當下的機器狀態一起看**，否則會把
鄰居的負載當成演算法的特性。

[![三種演算法的學習曲線](../../runs/rl_curves.png)](../../runs/rl_curves.png)

右圖（前 80k 步）看得出 PPO 的特徵：**曲線是震盪著往上爬的**，20k 步衝到 -1.19、40k 步
又掉回 -1.65。這是 on-policy 的正常樣貌 —— 每次更新都用剛蒐集的那批資料，策略一變，
下一批資料的分布跟著變，中間出現倒退很常見。SAC 那條（off-policy，有 replay buffer）
就平順得多。

拉長來看三者的取捨：PPO 最終成績最好（-0.10），但用了 150k 步；SAC 只要 60k 步就到
-0.20，而且要 GPU 才跑得動；ARS 十幾秒跑完卻吃掉 48 萬步。**時間、樣本、硬體，三者
挑兩個。**

## 除錯紀錄

1. **60k timesteps 完全不夠**：PPO 在 swing-up 上第一版只練 60k 步，回報幾乎沒動（-2.0 → -1.82）。PPO 是 on-policy 演算法，樣本效率低；本章最終使用 150k 步 + frame skip 2。
2. **不要省略評估**：訓練前後都用固定 seed 的單一環境、`deterministic=True` 評估，才能公平比較。

## ARS vs PPO 對照

| | ARS（08 章） | PPO（本章） |
| --- | --- | --- |
| 策略 | 線性，3 參數 | 神經網路（MlpPolicy） |
| 需要梯度 | 否 | 是 |
| 樣本效率 | 低 | 低（on-policy） |
| 實作/除錯成本 | 極低 | 中 |
| 可擴展到高維任務 | 有限 | 好 |

## 延伸閱讀

- [Stable-Baselines3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html)
- [Gymnasium 自訂環境教學](https://gymnasium.farama.org/tutorials/gymnasium_basics/environment_creation/)
- 下一步：雙擺 swing-up、加入 domain randomization、或改用 SAC（off-policy，樣本效率高得多）
