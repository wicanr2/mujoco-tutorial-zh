# 實驗紀錄（runs/）

模擬實驗的輸出：影片（mp4）、軌跡記錄（CSV）與圖表（png）。所有檔案都可由 `scripts/`
對應腳本重新產生（腳本會自動建立 `runs/`）。

## 檔案清單

| 檔案 | 產生腳本 | 對應教學 | 內容 |
| --- | --- | --- | --- |
| `mission.mp4` / `mission_log.csv` / `mission_traj.png` | `scripts/ex_yreach_mission.py` | [22](../docs/06-amr/10-yreach-mission.md) | 取放任務 19 秒：對位→插入→抬起→搬運→reach 外伸→放置→退出（571 幀、951 筆） |
| `fork_cyclic.mp4` / `fork_cyclic_log.csv` / `fork_cyclic_traj.png` | `scripts/ex_fork_cyclic.py` | [23](../docs/06-amr/11-fork-cyclic.md) | 連續動作 27.7 秒：載棧板升降 ×3 + 側移 ×3（831 幀），量循環中的漂移 |
| `reach_xz.mp4` / `reach_xz_log.csv` | `scripts/ex_reach_xz.py` | [24](../docs/06-amr/12-steer-reach-xz.md) | 舵輪底盤從貨架層板取貨，reach X（stage）+ Z（lift）。搬運段貨會滑落，見該章說明 |
| `reach_xyz.mp4` / `reach_xyz_log.csv` | `scripts/ex_reach_xyz.py` | [25](../docs/06-amr/13-reach-xyz.md) | 同場景加 y 向 reach，三軸機構動作。放置段未完成，見該章說明 |
| `gripper.mp4` / `gripper_log.csv` | `scripts/ex_gripper_ab.py` | [26](../docs/06-amr/14-gripper-ab.md) | 搬運車三軸手臂 + 二指夾爪，A 桌取箱 B 桌放下 |
| `loop.mp4` / `loop_log.csv` / `loop_traj.png` | `scripts/ex_loop_steer.py` | [27](../docs/06-amr/15-steer-loop.md) | 舵輪繞圈 waypoint 追蹤，2×2 m 正方形兩圈（4500 幀） |

影片一律 30 fps，CSV 一律 50 Hz 取樣、第一欄為模擬時間 `time`（秒）。

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
