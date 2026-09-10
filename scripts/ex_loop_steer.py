"""實驗 1：MR1533 舵輪底盤繞圈（loop）— 航向控制 + 錄影 + 6DOF 記錄。

控制：waypoint 追蹤 — 舵輪角度 ∝ 航向誤差（類 pure pursuit 簡化版），
繞 2×2 m 正方形兩圈。環氧樹脂地板（μ=0.7）。

輸出：runs/loop_log.csv、runs/loop.mp4、runs/loop_traj.png
"""
import mujoco
import numpy as np
import os
import csv

os.makedirs("runs", exist_ok=True)

model = mujoco.MjModel.from_xml_path("models/mr1533_steer.xml")
data = mujoco.MjData(model)

renderer = mujoco.Renderer(model, 480, 640)
cam = mujoco.MjvCamera()
cam.azimuth, cam.elevation, cam.distance = 90, -45, 6.5
cam.lookat = [1.0, 1.0, 0]
opt = mujoco.MjvOption()
opt.geomgroup[3] = 0

WAYPOINTS = [(0, 0), (2, 0), (2, 2), (0, 2)] * 2   # 兩圈
V_TARGET = 0.6   # m/s
WHEEL_R = 0.115

log_rows, frames = [], []
wp_i = 0


def yaw():
    w, x, y, z = data.qpos[3:7]
    return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi


def run(seconds):
    global wp_i
    for _ in range(int(seconds / model.opt.timestep)):
        x, y = data.qpos[0], data.qpos[1]
        tx, ty = WAYPOINTS[wp_i]
        if np.hypot(tx - x, ty - y) < 0.35:
            wp_i = min(wp_i + 1, len(WAYPOINTS) - 1)
            tx, ty = WAYPOINTS[wp_i]
        heading_err = wrap(np.arctan2(ty - y, tx - x) - yaw())
        steer = np.clip(1.5 * heading_err, -1.4, 1.4)
        drive = np.clip(V_TARGET / WHEEL_R, -12, 12) * max(0.4, 1 - 0.7 * abs(heading_err))
        data.ctrl[:] = [drive, steer, 0, 0, 0, 0, 0]
        mujoco.mj_step(model, data)
        if not log_rows or data.time - log_rows[-1][0] >= 0.0199:
            log_rows.append([data.time, x, y, data.qpos[2], yaw(), steer, drive, wp_i])
            if int(data.time * 30) > len(frames) - 1:
                renderer.update_scene(data, camera=cam, scene_option=opt)
                frames.append(renderer.render().copy())


print("繞圈開始（2×2 m 正方形 × 2 圈）...")
run(150.0)
print(f"結束：wp_i = {wp_i}（到達過 {wp_i} 個 waypoint）, x={data.qpos[0]:.2f}, y={data.qpos[1]:.2f}")

with open("runs/loop_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "x", "y", "z", "yaw", "steer", "drive", "wp_i"])
    w.writerows(log_rows)

import imageio
imageio.mimsave("runs/loop.mp4", frames, fps=30)
print(f"runs/loop.mp4: {len(frames)} 幀")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

log = np.array(log_rows)
fig, axes = plt.subplots(1, 3, figsize=(13, 4))
axes[0].plot(log[:, 1], log[:, 2])
wps = np.array(WAYPOINTS)
axes[0].plot(wps[:, 0], wps[:, 1], "ro--", alpha=0.4)
axes[0].set_aspect("equal"); axes[0].set_title("xy track"); axes[0].grid(True)
# CSV 欄位順序：0 time, 1 x, 2 y, 3 z, 4 yaw, 5 steer, 6 drive, 7 wp_i
axes[1].plot(log[:, 0], np.rad2deg(log[:, 4])); axes[1].set_title("yaw (deg)"); axes[1].grid(True)
axes[2].plot(log[:, 0], log[:, 5], label="steer (rad)")
axes[2].plot(log[:, 0], log[:, 6], label="drive (rad/s)")
axes[2].legend(); axes[2].set_title("ctrl"); axes[2].grid(True)
fig.tight_layout()
fig.savefig("runs/loop_traj.png", dpi=110)
print("runs/loop_traj.png")

assert wp_i >= 4, "沒有走完一圈"
print("結果：loop 實驗通過 ✓")
