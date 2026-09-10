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
fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "yreach")


def rel_to_fork():
    """棧板在牙叉座標系中的位置。滑動要在這個參考系量（20 章的教訓）。"""
    R = data.xmat[fid].reshape(3, 3)
    return R.T @ (data.xpos[pid] - data.xpos[fid])

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
            log_rows.append([data.time, *data.qpos[:7], *ctrl,
                             *data.xpos[pid], *rel_to_fork()])
            if int(data.time * 30) > len(frames) - 1:
                renderer.update_scene(data, camera=cam, scene_option=opt)
                frames.append(renderer.render().copy())


def drive_to(tx, hold=(0, 0), tilt=0.0, stage=0.0, reach=0.0,
             timeout=12.0, gain=1.0, cap=2.0):
    """閉迴圈開到 x=tx，全程保持航向 0（車體 -x 側是牙叉，本來就朝著貨架）。

    行進中要保持的通道全部要傳進來（`hold`／`tilt`／`stage`／`reach`），不傳就被歸零。
    載重靠摩擦坐在叉齒上，速度上限刻意壓得很低。細節見 24 章的同名函式。
    """
    for _ in range(int(timeout / model.opt.timestep)):
        ex = tx - data.qpos[0]
        w, xi, yi, zi = data.qpos[3:7]
        yaw = np.arctan2(2*(w*zi+xi*yi), 1-2*(yi*yi+zi*zi))
        heading_err = (0.0 - yaw + np.pi) % (2*np.pi) - np.pi
        steer = np.clip(1.2 * heading_err, -1.2, 1.2)
        drive = np.clip(gain * ex / 0.115, -cap, cap) * max(0.15, 1 - abs(heading_err))
        step([drive, steer, stage, hold[0], hold[1], tilt, reach], model.opt.timestep)
        if abs(ex) < 0.03:
            break
    step([0, 0, stage, hold[0], hold[1], tilt, reach], 0.5)


STAGE = -0.65      # 深插到位後全程保持，不收回
TILT = 0.08        # 後傾，讓貨靠上滑架背板

print("開到貨架前...")
drive_to(-0.65)
print("牙叉對準（lift1=0.39）...")
step([0, 0, 0, 0.39, 0, 0, 0], 2.0)
print("stage 前移插入（X: stage=-0.65）...")
step([0, 0, STAGE, 0.39, 0, 0, 0], 3.0)
print("微升離開層板（Z: +4cm）並後傾...")
step([0, 0, STAGE, 0.48, 0.08, TILT, 0], 1.5)
lifted_z = float(data.xpos[pid][2])
rel_home = rel_to_fork()
print(f"  微升後棧板 z = {lifted_z:.3f}")

# 在貨架內升高會讓棧板頂部逼近上層層板（只剩 8 mm），先退出來再升。
print("保持低位退出貨架...")
drive_to(0.3, hold=(0.48, 0.08), tilt=TILT, stage=STAGE, timeout=25.0)
print("離架後升到搬運高度（Z: lift1=0.70, lift2=0.35）...")
step([0, 0, STAGE, 0.70, 0.35, TILT, 0], 2.0)
carry_z = float(data.xpos[pid][2])
print(f"  搬運高度棧板 z = {carry_z:.3f}")
print("搬到目的地...")
drive_to(1.5, hold=(0.70, 0.35), tilt=TILT, stage=STAGE, timeout=20.0)

print("reach 側移對位（Y: -0.4）...")
step([0, 0, STAGE, 0.70, 0.35, TILT, -0.4], 3.0)
y_after_reach = float(data.xpos[pid][1])
print(f"  側移後棧板 y = {y_after_reach:.3f}")
print("放低到地面（Z: lift1=0.02）...")
step([0, 0, STAGE, 0.02, 0.0, TILT, -0.4], 2.5)
step([0, 0, STAGE, 0.02, 0.0, 0, -0.4], 1.5)
print(f"  放下後棧板 z = {data.xpos[pid][2]:.3f}")
print("reach 收回，退開...")
step([0, 0, STAGE, 0.02, 0.0, 0, 0], 2.0)

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

drift = float(np.linalg.norm(rel_to_fork() - rel_home))
moved = float(p[0]) + 1.5      # 起點 x = -1.5
print(f"棧板被搬運了 {moved:.2f} m；全程在牙叉座標系中漂移 {drift*100:.1f} cm")

# 三個軸各驗一件事，再加上「貨全程跟著車」與「真的放到地面」。
assert lifted_z > 0.42, f"X/Z 取貨失敗：棧板沒有離開層板（z={lifted_z:.3f}）"
assert abs(y_after_reach) > 0.05, f"Y 向 reach 沒有效果（棧板 y={y_after_reach:.3f}）"
assert drift < 0.15, f"搬運失敗：貨在叉齒上漂移 {drift*100:.1f} cm"
assert moved > 1.5, f"搬運失敗：棧板只移動 {moved:.2f} m"
assert p[2] < 0.10, f"放置失敗：棧板沒有降到地面（z={p[2]:.3f}）"
print("結果：reach X/Y/Z 取貨 → 搬運 → 側移對位 → 放置 全程驗證通過 ✓")
