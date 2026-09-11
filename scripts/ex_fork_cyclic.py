"""連續實驗：載著棧板，牙叉上上下下 ×3 + 左右左右 ×3（錄影 + 6DOF 記錄）。

流程：對位插入木頭棧板 → 抬起 → 連續動作：
  升降 0.10↔0.45 m ×3、reach ±0.35 m ×3，最後放下。
全程量測棧板相對牙叉的滑動，驗證棧板不掉落。

輸出：runs/fork_cyclic.mp4、runs/fork_cyclic_log.csv、runs/fork_cyclic_traj.png
"""
import mujoco
import numpy as np
import os
import csv

os.makedirs("runs", exist_ok=True)

# ===== 場景（同 mission：y-reach 車 + 木頭棧板）=====
base_xml = (open("models/mr1533_yreach.xml").read()
            .replace('meshdir="meshes/mr1533/"', 'meshdir="models/meshes/mr1533/"'))
pallet = """
    <body name="pallet_wood" pos="-1.35 0.45 0" euler="0 0 1.5707963">
      <freejoint/>
      <inertial pos="0 0 0.06" mass="20" diaginertia="0.15 0.15 0.1"/>
      <geom class="visual" type="mesh" mesh="pallet_wood" rgba="0.45 0.30 0.15 1"/>
      <geom type="box" size="0.5 0.4 0.011" pos="0 0 0.101" friction="0.6 0.05 0.01" group="3"/>
      <geom type="box" size="0.05 0.4 0.045" pos="-0.42 0 0.045" friction="0.6 0.05 0.01" group="3"/>
      <geom type="box" size="0.05 0.4 0.045" pos="0 0 0.045"     friction="0.6 0.05 0.01" group="3"/>
      <geom type="box" size="0.05 0.4 0.045" pos="0.42 0 0.045"  friction="0.6 0.05 0.01" group="3"/>
    </body>
"""
assets = '    <mesh name="pallet_wood" file="../pallet_wood.stl"/>\n'
xml = base_xml.replace("</asset>", assets + "</asset>").replace("</worldbody>", pallet + "</worldbody>")

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet_wood")
fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "yreach")
qadr = model.jnt_qposadr[model.body_jntadr[pid]]

renderer = mujoco.Renderer(model, 360, 640)
cam = mujoco.MjvCamera()
cam.azimuth, cam.elevation, cam.distance = 300, -16, 4.2
cam.lookat = [-0.6, 0.3, 0.5]
opt = mujoco.MjvOption()
opt.geomgroup[3] = 0

log_rows, frames = [], []


def quat2rpy(q):
    w, x, y, z = q
    return (np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
            np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def step(ctrl, seconds):
    for _ in range(int(seconds / model.opt.timestep)):
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
        if not log_rows or data.time - log_rows[-1][0] >= 0.0199:
            r, p, y = quat2rpy(data.qpos[qadr+3:qadr+7])
            # 棧板相對牙叉（yreach 座標系）
            mujoco.mj_fwdPosition(model, data)
            rel = data.xmat[fid].reshape(3, 3).T @ (data.xpos[pid] - data.xpos[fid])
            log_rows.append([data.time, *data.qpos[:3], data.qpos[3], data.qpos[6],
                             *data.qpos[qadr:qadr+3], r, p, y, *rel])
            # 漂移要取整段的峰值：貨在循環中滑出去、下一次落底又被推回來，
            # 只看最後一幀會低估（24 章踩過這個坑，這裡是同一件事）。
            # 但只算「貨還在叉齒上」那段 —— 放下之後叉齒繼續下降，相對位置本來就會
            # 變（實測 rel_z 從 0.095 跳到 0.275），那是卸貨動作不是漂移。
            if DRIFT["home"] is not None and data.ctrl[3] > 0.10:
                DRIFT["peak"] = max(DRIFT["peak"],
                                    float(np.linalg.norm(rel - DRIFT["home"])))
            if int(data.time * 30) > len(frames) - 1:
                renderer.update_scene(data, camera=cam, scene_option=opt)
                frames.append(renderer.render().copy())


DRIFT = {"home": None, "peak": 0.0}


def drive_to(tx, ty, lift=(0, 0), reach=0.0, timeout=6.0):
    for _ in range(int(timeout / model.opt.timestep)):
        ex, ey = tx - data.qpos[0], ty - data.qpos[1]
        step([np.clip(1.5*ex, -0.8, 0.8), np.clip(1.5*ey, -0.8, 0.8), 0,
              lift[0], lift[1], 0, reach], model.opt.timestep)
        if abs(ex) < 0.02 and abs(ey) < 0.02:
            break


# ===== 取貨 =====
print("取貨...")
step([0, 0, 0, 0, 0, 0, 0], 0.5)
drive_to(0.0, 0.45)
drive_to(-1.25, 0.45)
print(f"插入後: truck=({data.qpos[0]:.2f},{data.qpos[1]:.2f}) pallet=({data.xpos[pid][0]:.2f},{data.xpos[pid][1]:.2f}) 叉尖z={data.sensordata[2]:.3f} ncon={data.ncon}")
step([0, 0, 0, 0.35, 0.2, 0, 0], 2.0)
print(f"抬起後: lift1={data.qpos[3]:.3f} lift2={data.qpos[4]:.3f}")
drive_to(-0.3, 0.45, lift=(0.45, 0.3))
z0 = data.xpos[pid][2]
rel_home = np.array(log_rows[-1][-3:])   # 取貨完成時的相對位置（漂移基準）
DRIFT["home"] = rel_home
print(f"抬起 z = {z0:.3f} m，開始連續動作")

# ===== 連續：上上下下 ×3 =====
for i in range(3):
    print(f"升降 第 {i+1} 次")
    step([0, 0, 0, 0.15, 0.10, 0.08, 0], 1.5)   # 下（後傾 0.08 rad 固定棧板）
    step([0, 0, 0, 0.45, 0.30, 0.08, 0], 1.5)   # 上

# ===== 連續：左右左右 ×3 =====
for i in range(3):
    print(f"側移 第 {i+1} 次")
    step([0, 0, 0, 0.45, 0.30, 0.08,  0.35], 1.2)   # 右
    step([0, 0, 0, 0.45, 0.30, 0.08, -0.35], 1.2)   # 左
step([0, 0, 0, 0.45, 0.30, 0.08, 0], 1.0)

# ===== 放下 =====
print("放下...")
step([0, 0, 0, 0.0, 0.0, 0, 0], 2.0)

p = data.xpos[pid]
rel_final = np.array(log_rows[-1][-3:])
drift = np.linalg.norm(rel_final[:2] - rel_home[:2])
drift_peak = DRIFT["peak"]
print(f"最終棧板 z = {p[2]:.3f} m（放下應接近 0）")
print(f"連續動作期間的漂移：終點 {drift*100:.1f} cm、峰值 {drift_peak*100:.1f} cm"
      f"（基準：取貨完成時）")

# ===== 輸出 =====
with open("runs/fork_cyclic_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "base_x", "base_y", "base_yaw", "lift1", "reach_y",
                "pallet_x", "pallet_y", "pallet_z", "pallet_roll", "pallet_pitch", "pallet_yaw",
                "rel_x", "rel_y", "rel_z"])
    w.writerows(log_rows)

import imageio
imageio.mimsave("runs/fork_cyclic.mp4", frames, fps=30)
print(f"runs/fork_cyclic.mp4: {len(frames)} 幀")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

log = np.array(log_rows)
fig, axes = plt.subplots(1, 3, figsize=(13, 3.5))
axes[0].plot(log[:, 0], log[:, 4]); axes[0].set_ylabel("lift1 (m)"); axes[0].set_title("fork up/down")
axes[1].plot(log[:, 0], log[:, 5]); axes[1].set_ylabel("reach_y (m)"); axes[1].set_title("fork left/right")
axes[2].plot(log[:, 0], log[:, 8], label="pallet z")
axes[2].plot(log[:, 0], log[:, 12]*10, label="rel x (x10)")
axes[2].plot(log[:, 0], log[:, 13]*10, label="rel y (x10)")
axes[2].legend(fontsize=8); axes[2].set_title("pallet z & slip")
for ax in axes:
    ax.grid(True); ax.set_xlabel("t (s)")
fig.tight_layout()
fig.savefig("runs/fork_cyclic_traj.png", dpi=110)
print("runs/fork_cyclic_traj.png")

assert p[2] < 0.05, f"棧板沒有放下（z={p[2]:.3f}）"
assert drift < 0.10, f"終點漂移 {drift*100:.1f} cm 超過 10 cm"
assert drift_peak < 0.25, f"過程漂移峰值 {drift_peak*100:.1f} cm 超過 25 cm"
print("結果：連續上上下下左右左右驗證通過 ✓")
