import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/double_pendulum.xml")
data = mujoco.MjData(model)

# 初始姿態：把兩個關節各拉開 1 rad
data.qpos[:] = [1.0, 1.0]

for _ in range(500):  # 500 步 × 0.002s = 1 秒
    mujoco.mj_step(model, data)

print("關節角度 qpos =", np.round(data.qpos, 3))
print("關節角速度 qvel =", np.round(data.qvel, 3))
