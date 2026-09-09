"""範例：keyframe 儲存/重置狀態，以及 mj_resetDataKeyframe 的用法"""
import mujoco
import numpy as np

XML = """
<mujoco>
  <worldbody>
    <body pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.4" size="0.02"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="raised" qpos="1.0"/>
  </keyframe>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "raised")

# 從 keyframe 出發模擬 0.5 秒
mujoco.mj_resetDataKeyframe(model, data, key_id)
for _ in range(250):
    mujoco.mj_step(model, data)
print(f"從 keyframe 擺 0.5s 後 qpos = {data.qpos[0]:.3f}")

# 隨機擾動後再重置回 keyframe — 強化學習 episode 重置的標準做法
data.qpos[0] = -2.0
mujoco.mj_resetDataKeyframe(model, data, key_id)
print(f"重置回 keyframe：qpos = {data.qpos[0]:.3f}（應為 1.0）, time = {data.time}")
