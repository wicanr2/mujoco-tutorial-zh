"""範例：position 伺服追蹤正弦軌跡（不必自己寫 PD）"""
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/servo_pendulum.xml")
data = mujoco.MjData(model)

errors = []
while data.time < 3.0:
    data.ctrl[0] = np.sin(data.time)          # 目標角度：±1 rad 的正弦
    mujoco.mj_step(model, data)
    errors.append(data.ctrl[0] - data.qpos[0])

print(f"追蹤誤差 RMS = {np.sqrt(np.mean(np.square(errors))):.4f} rad")
print(f"最後 100 步誤差 RMS = {np.sqrt(np.mean(np.square(errors[-100:]))):.4f} rad")
