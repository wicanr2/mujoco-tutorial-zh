"""MR1533 真實 mesh 叉車（TB3 實驗資產）在 MuJoCo 中的動作驗證。

模型 models/mr1533_forklift.xml：視覺用 Blender 匯出的 OBJ mesh，
碰撞用簡化盒體；關節配置依 TB3 的 mr1533_light.urdf。
"""
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/mr1533_forklift.xml")
data = mujoco.MjData(model)

# 1. 前進
for _ in range(500):
    data.ctrl[:] = [0.8, 0, 0, 0, 0, 0]
    mujoco.mj_step(model, data)
print(f"前進後 x = {data.qpos[0]:.2f} m")

# 2. 兩段門架升起（lift1 → 1.0 m, lift2 → 0.5 m）
for _ in range(1500):
    data.ctrl[:] = [0, 0, 0, 1.0, 0.5, 0]
    mujoco.mj_step(model, data)
print(f"升起後 lift1 = {data.qpos[3]:.3f} m, lift2 = {data.qpos[4]:.3f} m")

# 3. 牙叉前傾
for _ in range(500):
    data.ctrl[:] = [0, 0, 0, 1.0, 0.5, 0.2]
    mujoco.mj_step(model, data)
print(f"前傾後 tilt = {np.rad2deg(data.qpos[5]):.1f}°，叉尖世界座標 = {np.round(data.sensordata, 2)}")

assert 0.9 < data.qpos[3] < 1.1 and 0.4 < data.qpos[4] < 0.6
assert data.qpos[5] > 0.1
print("結果：MR1533 mesh 模型動作驗證通過 ✓")
