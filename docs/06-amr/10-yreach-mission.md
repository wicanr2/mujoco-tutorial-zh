# 22｜y-reach 取放任務：錄影 + 6DOF 軌跡記錄

> 擷取日期：2026-09-10
> 測試環境：Linux、MuJoCo 3.12.0、Python 3.12.3、imageio/matplotlib；範例已實測通過。

## 學習目標

- 串聯完整 pick-and-place：對位 → 插入 → 抬起 → 搬運 → 放置 → 退出
- 錄製模擬影片（`mujoco.Renderer` + imageio → mp4）
- 記錄底盤與棧板的 x/y/z/roll/pitch/yaw（CSV + 軌跡圖）
- 閉迴圈 P 控制取代開環時間控制

## 前置知識

- [21｜y-reach 車型](09-yreach.md)、[11｜Viewer 與離屏渲染](../02-programming/06-viewer-rendering.md)

## 場景與任務

兩塊並排棧板（木頭 y=+0.45、塑膠 y=-0.45，各 20 kg）。任務：取木頭棧板，搬到塑膠棧板旁的空位放下。

```
靜置 → drive_to(0, 0.45) 對位 → drive_to(-0.95, 0.45) 後退插入
→ 抬起 2s → drive_to(0.4, 0.45) 退出貨架 → drive_to(0.4, -0.45) 橫移
→ reach 外伸 -0.45 → 放下 → drive_to(1.2, -0.45) 退出
```

底盤控制用閉迴圈 P 控制（`drive_to(tx, ty)`）— 誤差小自動減速，不會像開環時間控制那樣衝過頭。

## 實測結果

```
任務結束：木頭棧板位置 x=0.06 y=-0.55 z=-0.000   ← 從 (-1.35, 0.45) 搬來
runs/mission_log.csv: 951 筆（50 Hz）
runs/mission.mp4: 571 幀（30 fps，19 秒）
結果：取放任務完成 ✓
```

![6DOF 軌跡](../../runs/mission_traj.png)

軌跡解讀：棧板 z 在 t≈5s 衝到 0.53（抬起）、x 從 -1.35 跟到 0.06、y 從 0.45 跟到 -0.55（reach 外伸時一度到 -1.05）；roll/pitch 在插入與放置瞬間有小尖峰（接觸衝擊），其餘時間平穩。

影片：[runs/mission.mp4](../../runs/mission.mp4)

## 錄影與記錄的做法

```python
renderer = mujoco.Renderer(model, 360, 640)
frames = []
while simulating:
    mujoco.mj_step(model, data)
    if int(data.time * 30) > len(frames) - 1:      # 30 fps
        renderer.update_scene(data, camera=cam, scene_option=opt)
        frames.append(renderer.render().copy())     # .copy()！render() 回傳的是內部 buffer
imageio.mimsave("runs/mission.mp4", frames, fps=30)
```

6DOF 記錄：底盤直接讀 `qpos`（平面模型 x/y/yaw）；棧板是 freejoint，四元數轉 RPY：

```python
roll  = atan2(2(wx+yz), 1-2(x²+y²))
pitch = asin(2(wy-zx))
yaw   = atan2(2(wz+xy), 1-2(y²+z²))
```

## 除錯紀錄（本章血淚最多）

1. **開環對位衝過頭 0.13 m**：`vy=0.8 × 0.9s` 以為會到 0.45，實際到 0.576（速度伺服的加減速過程）— 叉齒撞到棧板枕木把貨推走。解法：閉迴圈 `drive_to`。
2. **叉齒間距對不上棧板枕木通道**（±0.315 vs 通道中心 ±0.21）：把棧板**整個鏟起來翻掉**。解法：回 Blender 把叉齒改到 ±0.21、縮窄到 0.1 m 寬 — 程序化建模改參數只要 30 秒。
3. **叉齒太高頂到板面**（prong top 0.10 vs deck bottom 0.09，1 cm 干涉）：把安裝高度降 0.015。
4. **freejoint 的 qpos 索引**：`mj_name2id(JOINT, body名)` 不一定對，用 `model.body_jntadr[body_id]` 最穩。
5. **錄影共用 buffer**：`renderer.render()` 回傳內部陣列，不 `.copy()` 的話所有 frame 都是同一張。
6. **matplotlib 中文字型**：headless 環境預設 DejaVu Sans 沒有 CJK，圖表標題用英文（或額外裝字型）。

## 延伸閱讀

- [11｜離屏渲染](../02-programming/06-viewer-rendering.md)
- 下一步：雙棧板都搬（連續任務）、加入感測器回饋（觸覺確認插入深度）、或把軌跡與真機 odometry 對照
