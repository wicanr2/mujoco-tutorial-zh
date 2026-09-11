"""y-reach 取放任務：兩塊並排棧板選擇性取放 + 錄影 + 6DOF 記錄。

場景：貨架上並排兩塊棧板（木頭在 y=+0.45、塑膠在 y=-0.45）。
任務：叉車對位木頭棧板 → reach 微調 → 插入 → 抬起 → 後退 → 放到空位 → 退出。

輸出（runs/）：
  mission_log.csv   — 時間、底盤與棧板的 x/y/z/roll/pitch/yaw
  mission.mp4       — 錄影（30 fps）
  mission_traj.png  — 6DOF 軌跡圖
"""
import mujoco
import numpy as np
import os
import csv

os.makedirs("runs", exist_ok=True)

# ===== 場景：y-reach 車 + 兩塊並排棧板 =====
base_xml = (open("models/mr1533_yreach.xml").read()
            .replace('meshdir="meshes/mr1533/"', 'meshdir="models/meshes/mr1533/"'))
pallet_tpl = """
    <body name="PALLET_NAME" pos="PX PY 0" euler="0 0 1.5707963">
      <freejoint/>
      <inertial pos="0 0 0.06" mass="20" diaginertia="0.15 0.15 0.1"/>
      <geom class="visual" type="mesh" mesh="PALLET_MESH" rgba="PALLET_RGBA"/>
      <geom type="box" size="0.5 0.4 0.011" pos="0 0 0.101" friction="FRIC 0.05 0.01" group="3" rgba="PALLET_RGBA"/>
      <geom type="box" size="0.05 0.4 0.045" pos="-0.42 0 0.045" friction="FRIC 0.05 0.01" group="3" rgba="PALLET_RGBA"/>
      <geom type="box" size="0.05 0.4 0.045" pos="0 0 0.045"     friction="FRIC 0.05 0.01" group="3" rgba="PALLET_RGBA"/>
      <geom type="box" size="0.05 0.4 0.045" pos="0.42 0 0.045"  friction="FRIC 0.05 0.01" group="3" rgba="PALLET_RGBA"/>
    </body>
"""
assets = '''    <mesh name="pallet_wood"    file="../pallet_wood.stl"/>
    <mesh name="pallet_plastic" file="../pallet_plastic.stl"/>
'''

def pallet_xml(name, mesh, y, rgba, fric, x=-1.35):
    return (pallet_tpl.replace("PALLET_NAME", name).replace("PX", str(x)).replace("PY", str(y))
            .replace("PALLET_MESH", mesh).replace("PALLET_RGBA", rgba).replace("FRIC", fric))

xml = base_xml.replace("</asset>", assets + "</asset>").replace("</worldbody>",
    pallet_xml("pallet_wood", "pallet_wood", 0.45, "0.45 0.30 0.15 1", "0.6")
    + pallet_xml("pallet_plastic", "pallet_plastic", -0.45, "0.15 0.35 0.75 1", "0.35")
    + "</worldbody>")

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
print(f"模型載入：nv={model.nv}, nu={model.nu}, nbody={model.nbody}")

pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet_wood")
fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "yreach")


def rel_to_fork():
    """棧板在牙叉座標系中的位置。載具會轉動與側移，滑動要在這個參考系量。"""
    R = data.xmat[fid].reshape(3, 3)
    return R.T @ (data.xpos[pid] - data.xpos[fid])


DRIFT = {"home": None, "peak": 0.0}


def track_drift():
    """貨在叉齒上這段期間的漂移峰值。

    只看最後一幀會低估：貨可能中途滑出去又被帶回來。判定要看整段軌跡。
    """
    if DRIFT["home"] is None or data.ctrl[3] <= 0.10:
        return
    DRIFT["peak"] = max(DRIFT["peak"],
                        float(np.linalg.norm(rel_to_fork() - DRIFT["home"])))
qadr = model.jnt_qposadr[model.body_jntadr[pid]]

# ===== 記錄器 =====
renderer = mujoco.Renderer(model, 360, 640)
cam = mujoco.MjvCamera()
cam.azimuth, cam.elevation, cam.distance = 300, -18, 4.5
cam.lookat = [-0.7, 0, 0.5]
opt = mujoco.MjvOption()
opt.geomgroup[3] = 0

log_rows, frames = [], []


def quat2rpy(q):
    w, x, y, z = q
    roll = np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = np.arcsin(np.clip(2*(w*y-z*x), -1, 1))
    yaw = np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll, pitch, yaw


def step(ctrl, seconds):
    n = int(seconds / model.opt.timestep)
    for _ in range(n):
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
        # 50 Hz 記錄 + 30 fps 錄影
        if len(log_rows) == 0 or data.time - log_rows[-1][0] >= 0.0199:
            bx, by, byaw = data.qpos[0], data.qpos[1], data.qpos[2]
            pq = data.qpos[qadr+3:qadr+7]
            pr, pp, py = quat2rpy(pq)
            log_rows.append([data.time, bx, by, 0.11, 0, 0, byaw,
                             *data.qpos[qadr:qadr+3], pr, pp, py])
            track_drift()
            if int(data.time * 30) > len(frames) - 1:
                renderer.update_scene(data, camera=cam, scene_option=opt)
                frames.append(renderer.render().copy())


# ===== 任務序列 =====
print("開始任務...")
def drive_to(tx, ty, reach=0.0, lift=(0, 0), timeout=6.0):
    """閉迴圈 P 控制開到 (tx, ty)。"""
    for _ in range(int(timeout / model.opt.timestep)):
        ex, ey = tx - data.qpos[0], ty - data.qpos[1]
        vx = np.clip(1.5 * ex, -0.8, 0.8)
        vy = np.clip(1.5 * ey, -0.8, 0.8)
        step([vx, vy, 0, lift[0], lift[1], 0, reach], model.opt.timestep)
        if abs(ex) < 0.02 and abs(ey) < 0.02:
            break
    step([0, 0, 0, lift[0], lift[1], 0, reach], 0.5)  # 停穩


step([0, 0, 0, 0, 0, 0, 0], 0.5)                       # 靜置
p_start = data.xpos[pid].copy()
drive_to(0.0, 0.45)                                    # 對準木頭棧板中心
drive_to(-0.95, 0.45)                                  # 後退插入（叉尖到棧板中心）
step([0, 0, 0, 0.35, 0.2, 0, 0], 2.0)                  # 抬起
z_lifted = float(data.xpos[pid][2])
DRIFT["home"] = rel_to_fork()                          # 漂移的基準：貨剛離地、還沒開始搬
drive_to(0.4, 0.45, lift=(0.35, 0.2))                  # 前進退出貨架
drive_to(0.4, -0.45, lift=(0.35, 0.2))                 # 橫移到塑膠棧板後方空位
step([0, 0, 0, 0.35, 0.2, 0, -0.45], 1.0)              # reach 外伸：把棧板送得更遠
step([0, 0, 0, 0.0, 0.0, 0, -0.45], 1.5)               # 放下
drive_to(1.2, -0.45)                                   # 前進退出、reach 收回

p = data.xpos[pid]
print(f"任務結束：木頭棧板位置 x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.3f}")

# ===== 輸出 =====
with open("runs/mission_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "base_x", "base_y", "base_z", "base_roll", "base_pitch", "base_yaw",
                "pallet_x", "pallet_y", "pallet_z", "pallet_roll", "pallet_pitch", "pallet_yaw"])
    w.writerows(log_rows)
print(f"runs/mission_log.csv: {len(log_rows)} 筆")

import imageio
imageio.mimsave("runs/mission.mp4", frames, fps=30)
print(f"runs/mission.mp4: {len(frames)} 幀")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

log = np.array(log_rows)
fig, axes = plt.subplots(2, 3, figsize=(13, 6))
labels = ["x (m)", "y (m)", "z (m)", "roll (rad)", "pitch (rad)", "yaw (rad)"]
for i, ax in enumerate(axes.flat):
    ax.plot(log[:, 0], log[:, 1+i], label="base")
    ax.plot(log[:, 0], log[:, 7+i], label="pallet")
    ax.set_ylabel(labels[i]); ax.grid(True); ax.legend(fontsize=8)
    ax.set_xlabel("t (s)")
fig.suptitle("y-reach pick-and-place mission: base vs pallet 6DOF")
fig.tight_layout()
fig.savefig("runs/mission_traj.png", dpi=110)
print("runs/mission_traj.png")

# 判定要對準「真正想證明的事」：貨被抬起來、全程跟著車、搬到了目的地、最後放到地面。
# 只檢查最終高度的話，貨掉在半路也會被算成「放下去了」（24 章踩過這個坑）。
moved = float(np.linalg.norm(np.asarray(p)[:2] - p_start[:2]))
drift = DRIFT["peak"]
print(f"棧板被搬運了 {moved:.2f} m；搬運全程在牙叉座標系中的漂移峰值 {drift*100:.1f} cm")
assert z_lifted > 0.25, f"取貨失敗：棧板沒有被抬離地面（z={z_lifted:.3f}）"
assert drift < 0.15, f"搬運失敗：貨在叉齒上的漂移峰值 {drift*100:.1f} cm"
assert moved > 0.8, f"搬運失敗：棧板只移動 {moved:.2f} m"
assert p[2] < 0.1, f"放置失敗：棧板沒有降到地面（z={p[2]:.3f}）"
print("結果：取貨 → 搬運 → 放置 全程驗證通過 ✓")
