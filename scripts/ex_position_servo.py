"""範例：position 伺服追蹤正弦軌跡（不必自己寫 PD）"""
import csv
import os

import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/servo_pendulum.xml")
data = mujoco.MjData(model)

errors = []
rows = []
while data.time < 3.0:
    data.ctrl[0] = np.sin(data.time)          # 目標角度：±1 rad 的正弦
    mujoco.mj_step(model, data)
    errors.append(data.ctrl[0] - data.qpos[0])
    if not rows or data.time - rows[-1][0] >= 0.0049:      # 200 Hz
        rows.append([round(data.time, 5), round(float(data.ctrl[0]), 5),
                     round(float(data.qpos[0]), 5), round(float(errors[-1]), 5)])

print(f"追蹤誤差 RMS = {np.sqrt(np.mean(np.square(errors))):.4f} rad")
print(f"最後 100 步誤差 RMS = {np.sqrt(np.mean(np.square(errors[-100:]))):.4f} rad")

os.makedirs("runs", exist_ok=True)
with open("runs/servo_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "target", "actual", "error"])
    w.writerows(rows)
print(f"runs/servo_log.csv（{len(rows)} 列）已寫出")
