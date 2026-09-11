"""03 章：用 PD 控制器把單擺穩定在水平。

順便掃幾組 KP：純 PD 沒有重力補償，穩態誤差不會歸零，只會隨 KP 變小。
輸出 runs/pd_control_log.csv（KP=4 的完整響應）與 runs/pd_kp_sweep.csv（各 KP 的穩態）。
"""
import csv
import os

import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/pendulum_actuated.xml")
data = mujoco.MjData(model)

TARGET = np.pi / 2   # 目標：水平（垂下為 0）
KP, KD = 4.0, 0.8    # PD 增益

rows = []
while data.time < 5.0:
    theta = data.qpos[0]
    omega = data.qvel[0]
    data.ctrl[0] = KP * (TARGET - theta) - KD * omega
    mujoco.mj_step(model, data)
    if not rows or data.time - rows[-1][0] >= 0.0199:      # 50 Hz
        rows.append([round(data.time, 4), round(float(data.qpos[0]), 5),
                     round(float(data.qvel[0]), 5), round(float(data.ctrl[0]), 5)])

print("最終角度 theta =", round(data.qpos[0], 3), "rad（目標", round(TARGET, 3), "）")
print("最終角速度     =", round(data.qvel[0], 3))
print("感測器讀值     =", np.round(data.sensordata, 3))

# 穩態誤差從哪來：重力在水平位置的力矩要靠 KP·e 才撐得住，e 必然不為零。
# KP 越大誤差越小，但永遠不會是 0 —— 要歸零得加積分項或重力前饋。
sweep = []
for kp in (2.0, 4.0, 10.0, 30.0, 100.0):
    d2 = mujoco.MjData(model)
    while d2.time < 5.0:
        d2.ctrl[0] = kp * (TARGET - d2.qpos[0]) - KD * d2.qvel[0]
        mujoco.mj_step(model, d2)
    err = TARGET - float(d2.qpos[0])
    sweep.append([kp, round(float(d2.qpos[0]), 5), round(err, 5), round(np.degrees(err), 3)])
    print(f"  KP={kp:6.1f} → 穩態 {d2.qpos[0]:.3f} rad，誤差 {err:+.3f} rad（{np.degrees(err):+.1f}°）")

os.makedirs("runs", exist_ok=True)
with open("runs/pd_control_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "theta", "omega", "ctrl"])
    w.writerows(rows)
with open("runs/pd_kp_sweep.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["kp", "theta_final", "error_rad", "error_deg"])
    w.writerows(sweep)
print(f"runs/pd_control_log.csv（{len(rows)} 列）、runs/pd_kp_sweep.csv 已寫出")
