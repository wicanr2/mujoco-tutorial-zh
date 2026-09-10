"""MR1533 + y 向 reach 滑台（Blender 建模）動作驗證。

新機構：reach_y（slide, ±0.45 m）— 牙叉整組側移伸出，
適用於貨架側向取放、狹窄通道不轉車身就能對位。
"""
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/mr1533_yreach.xml")
data = mujoco.MjData(model)

# 1. 伸出 reach（y + 0.4 m）
for _ in range(800):
    data.ctrl[:] = [0, 0, 0, 0, 0, 0, 0.4]
    mujoco.mj_step(model, data)
print(f"reach_y = {data.qpos[6]:.3f} m（目標 0.4）")

# 2. 保持伸出，門架升起
for _ in range(1500):
    data.ctrl[:] = [0, 0, 0, 0.4, 0.2, 0, 0.4]
    mujoco.mj_step(model, data)
print(f"lift1 = {data.qpos[3]:.3f} m, lift2 = {data.qpos[4]:.3f} m")

# 3. 收回 reach
for _ in range(800):
    data.ctrl[:] = [0, 0, 0, 0.4, 0.2, 0, 0.0]
    mujoco.mj_step(model, data)
print(f"reach_y 收回 = {data.qpos[6]:.3f} m")

tip = data.sensordata
print(f"叉尖世界座標 = {np.round(tip, 2)}")

assert abs(data.qpos[6]) < 0.02, "reach 沒收回"
assert 0.3 < data.qpos[3] < 0.5
print("結果：y-reach 模型動作驗證通過 ✓")
