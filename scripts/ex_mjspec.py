"""範例：mjSpec 程序化建模 — 不用寫 XML 也能建立模型"""
import mujoco
import numpy as np

spec = mujoco.MjSpec()
world = spec.worldbody

# 動態產生一排 N 個擺
N = 4
for i in range(N):
    body = world.add_body(name=f"pend{i}", pos=[i * 0.3, 0, 1.0])
    body.add_joint(name=f"hinge{i}", type=mujoco.mjtJoint.mjJNT_HINGE,
                   axis=[0, 1, 0], damping=0.05)
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                  fromto=[0, 0, 0, 0, 0, -0.3 - 0.1 * i],
                  size=[0.02], rgba=[0.2 * i % 1, 0.5, 1 - 0.2 * i, 1])

model = spec.compile()
data = mujoco.MjData(model)
data.qpos[:] = 0.8

for _ in range(500):
    mujoco.mj_step(model, data)

print(f"以 mjSpec 建立了 {N} 個長度不同的擺，nv = {model.nv}")
print(f"1 秒後 qpos = {np.round(data.qpos, 3)}")

# spec 也可以再存成 XML
xml = spec.to_xml()
with open("models/spec_generated.xml", "w") as f:
    f.write(xml)
print("已輸出 models/spec_generated.xml")
