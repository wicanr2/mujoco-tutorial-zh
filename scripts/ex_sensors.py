"""範例：觸覺感測器（touch）與加速度計（accelerometer）"""
import csv
import os

import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/touch_ball.xml")
data = mujoco.MjData(model)

touched = False
rows = []
for _ in range(2000):  # 球從 z=0.5 自由落下，直到碰地
    mujoco.mj_step(model, data)
    touch, ax, ay, az = data.sensordata
    if not rows or data.time - rows[-1][0] >= 0.0049:      # 200 Hz：碰撞尖峰很窄
        rows.append([round(data.time, 5), round(0.5 + float(data.qpos[0]), 5),
                     round(float(touch), 5), round(float(az), 4)])
    if touch > 0 and not touched:
        touched = True
        print(f"碰地！時間 t = {data.time:.3f} s")
        print(f"  touch 讀值       = {touch:.4f} N")
        print(f"  accelerometer    = [{ax:.2f}, {ay:.2f}, {az:.2f}] m/s²")

z_world = 0.5 + data.qpos[0]  # qpos 是 slide 關節座標，世界高度 = 初始位置 + qpos
print(f"最終世界高度 z = {z_world:.4f} m（球半徑 0.1，應停在附近）")

os.makedirs("runs", exist_ok=True)
with open("runs/sensors_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "z", "touch_N", "acc_z"])
    w.writerows(rows)
print(f"runs/sensors_log.csv（{len(rows)} 列）已寫出")
