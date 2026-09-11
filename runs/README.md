# 實驗紀錄（runs/）

模擬實驗的輸出：影片（mp4）、軌跡記錄（CSV）與圖表（png）。所有檔案都可由 `scripts/`
對應腳本重新產生（腳本會自動建立 `runs/`）。

## 檔案清單

| 檔案 | 產生腳本 | 對應教學 | 內容 |
| --- | --- | --- | --- |
| `mission.mp4` / `mission_log.csv` / `mission_traj.png` | `scripts/ex_yreach_mission.py` | [22](../docs/06-amr/10-yreach-mission.md) | 取放任務 19 秒：對位→插入→抬起→搬運→reach 外伸→放置→退出（571 幀、951 筆） |
| `fork_cyclic.mp4` / `fork_cyclic_log.csv` / `fork_cyclic_traj.png` | `scripts/ex_fork_cyclic.py` | [23](../docs/06-amr/11-fork-cyclic.md) | 連續動作 27.7 秒：載棧板升降 ×3 + 側移 ×3（831 幀），量循環中的漂移 |
| `reach_xz.mp4` / `reach_xz_log.csv` | `scripts/ex_reach_xz.py` | [24](../docs/06-amr/12-steer-reach-xz.md) | 舵輪底盤從貨架層板取貨、退出、升高、搬運 2.10 m、放到地面（漂移峰值 6.7 cm） |
| `reach_xyz.mp4` / `reach_xyz_log.csv` | `scripts/ex_reach_xyz.py` | [25](../docs/06-amr/13-reach-xyz.md) | 同流程加 y 向 reach 側移對位 0.49 m 再放置（漂移峰值 12.1 cm） |
| `gripper.mp4` / `gripper_log.csv` | `scripts/ex_gripper_ab.py` | [26](../docs/06-amr/14-gripper-ab.md) | 搬運車三軸手臂 + 二指夾爪，A 桌取箱 B 桌放下 |
| `loop.mp4` / `loop_log.csv` / `loop_traj.png` | `scripts/ex_loop_steer.py` | [27](../docs/06-amr/15-steer-loop.md) | 舵輪繞圈 waypoint 追蹤，2×2 m 正方形兩圈（4500 幀） |
| `tilt_boundary_log.csv` / `tilt_kp_sweep.csv` / `tilt_boundary.png` | `scripts/ex_tilt_boundary.py` ＋ `scripts/make_curves.py` | [17](../docs/06-amr/05-tilt-boundary.md) | 門架前傾掃描三案例，兩種座標系的滑移對照，以及 kp 對追隨誤差的影響 |
| `rerun_materials_log.csv` / `rerun_materials.png` | `scripts/ex_rerun_materials.py` ＋ `scripts/make_curves.py` | [20](../docs/06-amr/08-rerun-real-model.md) | 橫移強度 vy 掃描：兩種材質的滑移與是否被甩落 |
| `cartpole_log.csv` / `cartpole_traj.png` | `scripts/ex_cartpole_swingup.py` ＋ `scripts/make_curves.py` | [10](../docs/02-programming/05-cartpole-swingup.md) | 車桿 swing-up 16 秒：能量整形盪起、1.20 s 切 LQR 後穩定 |
| `rl_ars_log.csv` / `rl_ppo_log.csv` / `rl_sac_gpu_log.csv` / `rl_curves.png` | `scripts/ex_rl_swingup.py`、`ex_rl_ppo.py`、`ex_rl_sac_gpu.py` ＋ `scripts/make_curves.py` | [08](../docs/02-programming/03-rl-swingup.md)、[09](../docs/02-programming/04-ppo-gymnasium.md)、[12](../docs/02-programming/07-sac.md) | 三種演算法在同一個單擺 swing-up 任務上的學習曲線 |
| `pd_control_log.csv` / `pd_kp_sweep.csv` / `pd_control.png` | `scripts/pd_control.py` ＋ `scripts/make_curves.py` | [03](../docs/02-programming/01-simulation-loop.md) | PD 控制的響應曲線與 KP 掃描（穩態誤差 14.6° → 0.3°，永遠不歸零） |
| `sensors_log.csv` / `servo_log.csv` / `parallel_rollout_log.csv` / `more_examples.png` | `scripts/ex_sensors.py`、`ex_position_servo.py`、`ex_parallel_rollout.py` ＋ `scripts/make_curves.py` | [07](../docs/02-programming/02-more-examples.md) | 觸覺感測器的接觸尖峰、伺服追隨正弦的相位落後、16 條平行 rollout 的發散 |
| `mjx_throughput.csv` / `mjx_throughput.png` | 遠端主機的 `scripts/ex_mjx_throughput.py` 輸出（見下方說明）＋ `scripts/make_curves.py` | [28](../docs/02-programming/08-mjx-gpu.md) | 三種做法的模擬吞吐對照 |

影片一律 30 fps。時間序列的 CSV 一律 50 Hz 取樣、第一欄為模擬時間 `time`（秒）；
學習曲線的 CSV 以訓練進度為橫軸（`iteration` 或 `steps`），不是時間。

`mjx_throughput.csv` 不是每次重跑的產物：它記的是一次效能量測的結果（2026-09-11，
遠端 8 核主機、load average 0.39），而效能數字每跑一次就不一樣。腳本
`ex_mjx_throughput.py` 只印到 stdout、不寫這個檔，否則 CI 的「重跑後 `runs/` 不得變動」
會每次都失敗。

`rl_sac_gpu_log.csv` 的資料來自 2026-09-11 在遠端 RTX Pro 6000 上的那次訓練
（`N_ENVS=4`，見 [12 章](../docs/02-programming/07-sac.md)），不是本機跑出來的 —— 本機沒有
可用的 CUDA 裝置。腳本現在會自己寫出這個檔案，在有 GPU 的機器上重跑就會覆蓋。

## CSV 欄位說明

### mission_log.csv

| 欄位 | 說明 |
| --- | --- |
| `base_x, base_y, base_z` | 底盤世界座標（平面模型，z 固定 0.11） |
| `base_roll, base_pitch, base_yaw` | 底盤姿態（平面模型僅 yaw 變化，rad） |
| `pallet_x, pallet_y, pallet_z` | 木頭棧板世界座標 |
| `pallet_roll, pallet_pitch, pallet_yaw` | 棧板姿態（freejoint 四元數轉 RPY，rad） |

### fork_cyclic_log.csv

| 欄位 | 說明 |
| --- | --- |
| `base_x, base_y, base_yaw` | 底盤位置與朝向 |
| `lift1` | 門架第一級高度（m） |
| `reach_y` | reach 滑台側移量（m） |
| `pallet_x/y/z, pallet_roll/pitch/yaw` | 棧板 6DOF |
| `rel_x, rel_y, rel_z` | 棧板在 **yreach（牙叉）座標系**中的相對位置 — 滑動 / 漂移以此欄位量測，基準取「取貨完成時」的值 |

### reach_xz_log.csv、reach_xyz_log.csv

兩支欄位相同（舵輪車型 `models/mr1533_steer.xml`）：

| 欄位 | 說明 |
| --- | --- |
| `bx, by, bz` | 底盤世界座標（freejoint） |
| `qw, qx, qy, qz` | 底盤姿態四元數（MuJoCo 慣例 w 在前） |
| `drive, steer` | 後舵輪的驅動角速度與轉向角 |
| `stage` | 門架前後移動（reach X，m，負值為往前伸） |
| `lift1, lift2` | 升降兩級高度（m） |
| `tilt` | 牙叉前傾角（rad） |
| `reach` | y 向 reach 側移量（m，僅 reach_xyz 用到） |
| `px, py, pz` | 棧板世界座標 |
| `rel_x, rel_y, rel_z` | 棧板在 **yreach（牙叉）座標系**中的位置 — 漂移量以此欄位量測，基準取「微升離開層板」那一刻 |

### gripper_log.csv

| 欄位 | 說明 |
| --- | --- |
| `bx, by, byaw` | 搬運車底盤位置與朝向 |
| `q3, q4, q5` | 手臂三個關節角（shoulder / elbow / wrist，rad） |
| `px, py, pz` | 被搬運箱子的世界座標 |

### loop_log.csv

| 欄位 | 說明 |
| --- | --- |
| `x, y, z` | 底盤世界座標 |
| `yaw` | 底盤朝向（由 freejoint 四元數換算，rad） |
| `steer, drive` | 舵輪轉向角（rad）與驅動角速度（rad/s） |
| `wp_i` | 目前追蹤到第幾個 waypoint |

## 重新產生

```bash
# 需先有 .venv（pip install -r requirements.txt）
MUJOCO_GL=osmesa .venv/bin/python scripts/ex_yreach_mission.py
MUJOCO_GL=osmesa .venv/bin/python scripts/ex_fork_cyclic.py
MUJOCO_GL=osmesa .venv/bin/python scripts/ex_reach_xz.py
MUJOCO_GL=osmesa .venv/bin/python scripts/ex_reach_xyz.py
MUJOCO_GL=osmesa .venv/bin/python scripts/ex_gripper_ab.py
MUJOCO_GL=osmesa .venv/bin/python scripts/ex_loop_steer.py
```

無顯示器環境必須設 `MUJOCO_GL=osmesa`（或 `egl`），否則渲染會失敗。重跑會覆蓋這裡的檔案，
需要保留舊版時先複製到 `workspace/backup/`。

## 過程圖條

六支錄影各有一張過程圖條（`docs/assets/strip_<實驗名>.png`），在對應章節裡就看得到，
不必下載影片。用 `scripts/make_video_strips.py` 從這裡的 `.mp4` 抽幀產生 —— 改動實驗
腳本、重錄影片之後要一起重產：

```bash
python scripts/make_video_strips.py                    # 全部
python scripts/make_video_strips.py reach_xz           # 單一支
python scripts/make_video_strips.py --suggest reach_xz # 重挑時間點時看候選
```

### tilt_boundary_log.csv

| 欄位 | 說明 |
| --- | --- |
| `case` | 案例名稱（木頭 / 塑膠 / 塑膠+重心前移） |
| `target_deg` | 送進 position 致動器的目標角（`ctrl`） |
| `tilt_deg` | 門架**實際**傾角（從 `qpos` 讀）— 載重下與目標差很多 |
| `slip_cm` | 棧板在**叉齒座標系**中的位移，這是真實滑移 |
| `slip_world_cm` | 同一件事在世界座標量到的值，含門架轉動帶走的部分（會高估） |
| `pallet_vs_fork_deg` | 棧板相對叉齒的姿態偏離，用來判翻倒 |

### cartpole_log.csv

| 欄位 | 說明 |
| --- | --- |
| `x`, `xdot` | 台車位置（m）與速度 |
| `theta`, `thetadot` | 桿角（rad，從垂下 π 出發，直立為 0）與角速度；未 wrap 到 [-π, π] |
| `ctrl` | 施加在台車上的力（N，限幅 ±10） |
| `mode` | 0 = 能量整形盪起，1 = 已切換到 LQR |

### rl_*_log.csv

| 欄位 | 說明 |
| --- | --- |
| `iteration`（ARS）/ `steps`（PPO、SAC） | 訓練進度。ARS 每輪跑 16 個 episode × 1000 步 = 16,000 環境步 |
| `reward_per_step` | 以固定 seed 評估的平均回報／步，越接近 0 越好 |

### pd_control_log.csv / pd_kp_sweep.csv

| 欄位 | 說明 |
| --- | --- |
| `theta`, `omega` | 擺角（rad，垂下為 0）與角速度 |
| `ctrl` | PD 控制器輸出的力矩（N·m） |
| `kp`, `theta_final`, `error_rad`, `error_deg` | KP 掃描：各增益下的穩態角與殘餘誤差 |

### sensors_log.csv / servo_log.csv / parallel_rollout_log.csv

| 欄位 | 說明 |
| --- | --- |
| `z`, `touch_N`, `acc_z` | 球的世界高度、觸覺感測器讀值（N）、加速度計 z 分量。200 Hz 取樣 — 接觸力是只持續幾個 timestep 的尖峰，50 Hz 會整個錯過 |
| `target`, `actual`, `error` | 伺服的目標角、實際角與誤差（rad） |
| `run`, `q0`, `q1` | 平行 rollout 的編號與兩個關節角；16 條軌跡疊在同一個檔案裡 |

### rerun_materials_log.csv

| 欄位 | 說明 |
| --- | --- |
| `vy` | 橫移速度命令（m/s），±vy 來回三次 |
| `wood_slip_cm` / `plastic_slip_cm` | 棧板相對車體的最終位移 |
| `wood_fell` / `plastic_fell` | 1 = 棧板被甩落到地面 |
| `gap_cm` | 兩種材質的差距；只有兩者都沒掉落時才有意義 |
