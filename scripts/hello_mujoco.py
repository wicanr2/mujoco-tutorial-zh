import mujoco

model = mujoco.MjModel.from_xml_path("models/hello.xml")
data = mujoco.MjData(model)

for _ in range(200):  # 模擬 200 步（預設 timestep 0.002s → 0.4 秒）
    mujoco.mj_step(model, data)

print("球的高度 z =", data.qpos[2])
