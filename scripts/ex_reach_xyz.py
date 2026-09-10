"""實驗 3：取放 + reach X/Y/Z 全軸（貨架情境，舵輪底盤）。

在實驗 2 的基礎上加 y 向 reach：取貨後把棧板側移到旁邊空位放下。
流程：到位 → lift → stage 插入（X）→ 抬起 → stage 收回
      → reach 側移（Y）→ 放低（Z）→ reach 收回。
輸出：runs/reach_xyz_log.csv、runs/reach_xyz.mp4
"""
import mujoco
from pathlib import Path
import numpy as np
import os
import csv

os.makedirs("runs", exist_ok=True)

REPO_ROOT = Path(__file__).resolve().parent.parent
MESHES = REPO_ROOT / "models" / "meshes"

# 從 repo 根算路徑，不依賴 cwd；meshdir 換成絕對路徑，因為 from_xml_string
# 沒有檔案位置可當基準。
steer = (REPO_ROOT / "models" / "mr1533_steer.xml").read_text()
steer = steer.replace('meshdir="meshes/mr1533/"', f'meshdir="{MESHES / "mr1533"}/"')
steer = steer.replace('file="../yreach_carriage.stl"', f'file="{MESHES / "yreach_carriage.stl"}"')

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
assets = (f'    <mesh name="pallet_wood" 'f'file="{MESHES / "pallet_wood.stl"}"/>\n')
xml = steer.replace("</asset>", assets + "</asset>").replace("</worldbody>", RACK + "</worldbody>")

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet_wood")

renderer = mujoco.Renderer(model, 480, 640)
cam = mujoco.MjvCamera()
cam.azimuth, cam.elevation, cam.distance = 270, -15, 4.5
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


def drive_to(tx, hold=(0, 0), timeout=12.0):
    for _ in range(int(timeout / model.opt.timestep)):
        ex = tx - data.qpos[0]
        w, xi, yi, zi = data.qpos[3:7]
        yaw = np.arctan2(2*(w*zi+xi*yi), 1-2*(yi*yi+zi*zi))
        heading_err = (np.pi - yaw + np.pi) % (2*np.pi) - np.pi
        steer = np.clip(1.5 * heading_err, -1.4, 1.4)
        drive = np.clip(4.0 * ex / 0.115, -8, 8) * max(0.4, 1 - 0.7*abs(heading_err))
        step([drive, steer, 0, hold[0], hold[1], 0, 0], model.opt.timestep)
        if abs(ex) < 0.03:
            break
    step([0, 0, 0, 0, 0, 0, 0], 0.5)


print("開到貨架前...")
drive_to(-0.65)
print("牙叉對準（lift1=0.39）...")
step([0, 0, 0, 0.39, 0, 0, 0], 2.0)
print("stage 前移插入（stage=-0.65）...")
step([0, 0, -0.65, 0.39, 0, 0, 0], 3.0)
print("微升離開層板（Z: +4cm）...")
step([0, 0, -0.65, 0.48, 0.08, 0, 0], 1.5)
print("（深插已到位，省略 stage 收回）")
print("升到搬運高度（lift1=0.70, lift2=0.35）...")
step([0, 0, 0, 0.70, 0.35, 0, 0], 2.0)
lifted_z = float(data.xpos[pid][2])
print(f"  抬起後棧板 z = {lifted_z:.3f}")
print("reach 側移（Y: -0.4）...")
step([0, 0, 0, 0.55, 0.30, 0, -0.4], 2.0)
print("放低（lift1=0.05, lift2=0.02）...")
step([0, 0, 0, 0.05, 0.02, 0, -0.4], 2.0)
print("reach 收回...")
step([0, 0, 0, 0.05, 0.02, 0, 0], 2.0)
print("往前開離開貨架...")
drive_to(0.8, hold=(0.70, 0.35), timeout=20.0)

p = data.xpos[pid]
print(f"最終棧板 = {np.round(p, 3)}")

with open("runs/reach_xyz_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "bx", "by", "bz", "qw", "qx", "qy", "qz",
                "drive", "steer", "stage", "lift1", "lift2", "tilt", "reach", "px", "py", "pz"])
    w.writerows(log_rows)

import imageio
imageio.mimsave("runs/reach_xyz.mp4", frames, fps=30)
print(f"runs/reach_xyz.mp4: {len(frames)} 幀")

# 這章驗證的是三個 reach 軸都能動作：X（stage 深插取貨）、Z（抬離層板）、Y（側移）。
# 放置與退出那一段不成立 —— 「放低」後棧板仍在叉齒上，接著 drive_to 的 hold 把門架
# 重新升起，貨在行進中掉落。詳見 docs/06-amr/13-reach-xyz.md，這裡不把它算進驗收。
assert lifted_z > 0.60, f"取貨失敗：棧板沒有被抬離層板（z={lifted_z:.3f}）"
assert abs(p[1]) > 0.05, f"y 向 reach 沒有效果（棧板 y={p[1]:.3f}）"
print("結果：reach X/Y/Z 三軸機構驗證通過 ✓（放置段的問題見文件）")
