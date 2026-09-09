import mujoco
import numpy as np

# 直接載入 URDF：MuJoCo 內建支援
model = mujoco.MjModel.from_xml_path("models/two_link_arm.urdf")
data = mujoco.MjData(model)

print("自由度 nv =", model.nv, "| 關節數 njnt =", model.njnt)
print("關節名稱  =", [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                      for i in range(model.njnt)])

# 拉離平衡點（垂下為 0），在重力下自然擺動 1 秒
data.qpos[:] = [1.0, 0.5]
for _ in range(500):
    mujoco.mj_step(model, data)
print("1 秒後 qpos =", np.round(data.qpos, 3))

# 將載入的模型轉存成 MJCF，之後可直接以 MJCF 編輯維護
mujoco.mj_saveLastXML("models/two_link_arm_converted.xml", model)
print("已轉存 MJCF → models/two_link_arm_converted.xml")
