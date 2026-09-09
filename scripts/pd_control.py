import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/pendulum_actuated.xml")
data = mujoco.MjData(model)

TARGET = np.pi / 2   # 目標：水平（垂下為 0）
KP, KD = 4.0, 0.8    # PD 增益

while data.time < 5.0:
    theta = data.qpos[0]
    omega = data.qvel[0]
    data.ctrl[0] = KP * (TARGET - theta) - KD * omega
    mujoco.mj_step(model, data)

print("最終角度 theta =", round(data.qpos[0], 3), "rad（目標", round(TARGET, 3), "）")
print("最終角速度     =", round(data.qvel[0], 3))
print("感測器讀值     =", np.round(data.sensordata, 3))
