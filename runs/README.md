# 實驗紀錄（runs/）

本目錄存放模擬實驗的輸出：影片（mp4）、軌跡記錄（CSV）與圖表（png）。
所有檔案都可由 `scripts/` 對應腳本重新產生（腳本會自動 `os.makedirs("runs")`）。

## 檔案清單

| 檔案 | 產生腳本 | 對應教學 | 內容 |
| --- | --- | --- | --- |
| `mission.mp4` | `scripts/ex_yreach_mission.py` | [22](../docs/06-amr/10-yreach-mission.md) | 取放任務 19 秒：對位→插入→抬起→搬運→reach 外伸→放置→退出（571 幀） |
| `mission_log.csv` | 同上 | 22 | 50 Hz，951 筆，底盤與棧板 6DOF |
| `mission_traj.png` | 同上 | 22 | base vs pallet 6 子圖軌跡 |
| `fork_cyclic.mp4` | `scripts/ex_fork_cyclic.py` | [23](../docs/06-amr/11-fork-cyclic.md) | 連續動作 27.7 秒：載棧板升降 ×3 + 側移 ×3（831 幀） |
| `fork_cyclic_log.csv` | 同上 | 23 | 50 Hz，底盤/lift/reach + 棧板 6DOF + 相對牙叉位置 |
| `fork_cyclic_traj.png` | 同上 | 23 | lift / reach / 棧板 z 與漂移 |

## CSV 欄位說明

### mission_log.csv

| 欄位 | 說明 |
| --- | --- |
| `time` | 模擬時間（秒） |
| `base_x, base_y, base_z` | 底盤世界座標（平面模型，z 固定 0.11） |
| `base_roll, base_pitch, base_yaw` | 底盤姿態（平面模型僅 yaw 變化，rad） |
| `pallet_x, pallet_y, pallet_z` | 木頭棧板世界座標 |
| `pallet_roll, pallet_pitch, pallet_yaw` | 棧板姿態（freejoint 四元數轉 RPY，rad） |

### fork_cyclic_log.csv

| 欄位 | 說明 |
| --- | --- |
| `time` | 模擬時間（秒） |
| `base_x, base_y, base_yaw` | 底盤位置與朝向 |
| `lift1` | 門架第一級高度（m） |
| `reach_y` | reach 滑台側移量（m） |
| `pallet_x/y/z, pallet_roll/pitch/yaw` | 棧板 6DOF |
| `rel_x, rel_y, rel_z` | 棧板在 **yreach（牙叉）座標系**中的相對位置 — 滑動/漂移以此欄位量測 |

## 重新產生

```bash
# 需先有 .venv（pip install mujoco numpy imageio imageio-ffmpeg matplotlib pillow）
MUJOCO_GL=osmesa .venv/bin/python scripts/ex_yreach_mission.py
MUJOCO_GL=osmesa .venv/bin/python scripts/ex_fork_cyclic.py
```

> 無顯示器環境必須設 `MUJOCO_GL=osmesa`（或 `egl`），否則渲染會失敗。
