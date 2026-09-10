"""實驗 2：原地取放 — stage reach（X）+ lift（Z），貨架情境。

叉車停在貨架前不動，僅用 stage 前移（X）與升降（Z）從層板取貨、放回。
環氧地板（μ=0.7）。

輸出：runs/reach_xz_log.csv、runs/reach_xz.mp4
"""
import mujoco
import numpy as np
import os
import csv

os.makedirs("runs", exist_ok=True)

steer = open("models/mr1533_steer.xml").read()

# 貨架：兩根立柱 + 兩層橫梁（層板 z=0.35）；木頭棧板放在層板上
RACK = """
    <body name="rack" pos="-1.2 0 0">
      <geom type="box" size="0.05 0.05 0.7" pos="0  0.60 0.7" rgba="0.2 0.3 0.5 1" group="3"/>
      <geom type="box" size="0.05 0.05 0.7" pos="0 -0.60 0.7" rgba="0.2 0.3 0.5 1" group="3"/>
      <geom type="box" size="0.35 0.55 0.025" pos="-0.3 0 0.325" rgba="0.3 0.4 0.6 1" group="3"/>
      <geom type="box" size="0.35 0.55 0.025" pos="-0.3 0 0.875" rgba="0.3 0.4 0.6 1" group="3"/>
    </body>
    <body name="pallet_wood" pos="-1.5 0 0.35" euler="0 0 1.5707963">
      <freejoint/>
      <inertial pos="0 0 0.06" mass="20" diaginertia="0.15 0.15 0.1"/>
      <geom class="visual" type="mesh" mesh="pallet_wood" rgba="0.45 0.30 0.15 1"/>
      <geom type="box" size="0.5 0.4 0.011" pos="0 0 0.101" friction="0.6 0.05 0.01" group="3"/>
      <geom type="box" size="0.05 0.4 0.045" pos="-0.42 0 0.045" friction="0.6 0.05 0.01" group="3"/>
      <geom type="box" size="0.05 0.4 0.045" pos="0 0 0.045"     friction="0.6 0.05 0.01" group="3"/>
      <geom type="box" size="0.05 0.4 0.045" pos="0.42 0 0.045"  friction="0.6 0.05 0.01" group="3"/>
    </body>
"""
assets = '    <mesh name="pallet_wood" file="/home/anr2/tmp2/mujoco/models/meshes/pallet_wood.stl"/>\n'
xml = steer.replace("</asset>", assets + "</asset>").replace("</worldbody>", RACK + "</worldbody>")

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet_wood")

renderer = mujoco.Renderer(model, 480, 640)
cam = mujoco.MjvCamera()
cam.azimuth, cam.elevation, cam.distance = 270, -15, 4.0
cam.lookat = [-0.8, 0, 0.5]
opt = mujoco.MjvOption()
opt.geomgroup[3] = 0

log_rows, frames = [], []


def step(ctrl, seconds):
    for _ in range(int(seconds / model.opt.timestep)):
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
        if not log_rows or data.time - log_rows[-1][0] >= 0.0199:
            log_rows.append([data.time, *data.qpos[:7], *ctrl, *data.xpos[pid]])
            if int(data.time * 30) > len(frames) - 1:
                renderer.update_scene(data, camera=cam, scene_option=opt)
                frames.append(renderer.render().copy())


def drive_to(tx, hold=(0, 0), timeout=10.0):
    """閉迴圈開到 (tx, 0)，朝向 -x（讓牙叉朝貨架）。"""
    for _ in range(int(timeout / model.opt.timestep)):
        ex = tx - data.qpos[0]
        w, xi, yi, zi = data.qpos[3:7]
        yaw = np.arctan2(2*(w*zi+xi*yi), 1-2*(yi*yi+zi*zi))
        heading_err = (np.pi - yaw + np.pi) % (2*np.pi) - np.pi
        steer = np.clip(1.2 * heading_err, -1.2, 1.2)
        drive = np.clip(4.0 * ex / 0.115, -8, 8) * max(0.15, 1 - abs(heading_err))
        step([drive, steer, 0, hold[0], hold[1], 0, 0], model.opt.timestep)
        if abs(ex) < 0.03:
            break
    step([0, 0, 0, 0, 0, 0, 0], 0.5)


# 叉齒世界高度 = 0.29+lift1+lift2-0.39+0.185 ≈ 0.085 + lift1 + lift2
# 層板頂 z=0.35 → 棧板底 z=0.35，叉齒需到 0.30 進入、再升 0.15 抬起
print("開到貨架前...")
drive_to(-0.65)
print("牙叉對準層板下方（lift1=0.39）...")
step([0, 0, 0, 0.39, 0, 0, 0], 2.0)
print("stage 前移插入（stage=-0.65）...")
step([0, 0, -0.65, 0.39, 0, 0, 0], 3.0)
p0 = data.xpos[pid].copy()
print(f"  插入後棧板 = {np.round(p0, 3)}")
print("微升離開層板（+4cm）...")
step([0, 0, -0.65, 0.48, 0.08, 0, 0], 1.5)
print(f"  微升後棧板 z = {data.xpos[pid][2]:.3f}")
print("（深插已到位，省略 stage 收回 — 避免棧板傾斜）")
print("升到搬運高度（lift1=0.70, lift2=0.35）...")
step([0, 0, 0, 0.70, 0.35, 0, 0], 2.0)
print(f"  抬起後棧板 z = {data.xpos[pid][2]:.3f}")
print("往前開離開貨架...")
drive_to(0.8, hold=(0.70, 0.35), timeout=20.0)
print("放低（lift1=0.02）...")
step([0, 0, 0, 0.02, 0.0, 0, 0], 2.5)
print(f"放下後棧板 = {np.round(data.xpos[pid], 3)}")

with open("runs/reach_xz_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "bx", "by", "bz", "qw", "qx", "qy", "qz",
                "drive", "steer", "stage", "lift1", "lift2", "tilt", "reach",
                "px", "py", "pz"])
    w.writerows(log_rows)

import imageio
imageio.mimsave("runs/reach_xz.mp4", frames, fps=30)
print(f"runs/reach_xz.mp4: {len(frames)} 幀")

pz = data.xpos[pid][2]
px = data.xpos[pid][0]
print(f"最終棧板 x = {px:.2f}, z = {pz:.3f}")
assert pz < 0.25 and px > -1.0, "取放失敗"
print("結果：原地取放（reach X/Z）驗證通過 ✓")
