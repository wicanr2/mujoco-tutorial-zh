"""實驗 2：原地取放 — stage reach（X）+ lift（Z），貨架情境。

叉車停在貨架前不動，僅用 stage 前移（X）與升降（Z）從層板取貨、放回。
環氧地板（μ=0.7）。

輸出：runs/reach_xz_log.csv、runs/reach_xz.mp4
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


DRIFT = {"home": None, "peak": 0.0}


def track_drift():
    """追蹤貨還在叉齒上這段期間的漂移峰值。

    只看最後一幀會低估：貨在行進中滑出去、後傾又把它帶回來，終點值會比過程中的
    峰值小。判定實驗成敗要看整段軌跡。
    """
    if DRIFT["home"] is None or data.ctrl[3] <= 0.10:
        return
    d = float(np.linalg.norm(rel_to_fork() - DRIFT["home"]))
    DRIFT["peak"] = max(DRIFT["peak"], d)

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
            log_rows.append([data.time, *data.qpos[:7], *ctrl,
                             *data.xpos[pid], *rel_to_fork()])
            track_drift()
            if int(data.time * 30) > len(frames) - 1:
                renderer.update_scene(data, camera=cam, scene_option=opt)
                frames.append(renderer.render().copy())


def drive_to(tx, hold=(0, 0), tilt=0.0, stage=0.0, timeout=10.0, gain=1.0, cap=2.0):
    """閉迴圈開到 x=tx，全程保持航向 0（車體 -x 側是牙叉，本來就朝著貨架）。

    `hold` / `tilt` / `stage` 是「行進中要保持的通道值」。不傳就會被歸零 —— 那正是
    第 22、23、25 章都踩過的坑：共用控制函式會偷改你以為不變的通道。載著貨行進時
    stage 要維持深插、tilt 要維持後傾，否則貨會被甩出叉齒。

    `gain` / `cap` 預設得很保守（線速度約 0.23 m/s）。載重 20 kg 靠摩擦坐在叉齒上，
    加速度大了就會相對滑動 —— 慢，是這裡的正確答案。
    """
    for _ in range(int(timeout / model.opt.timestep)):
        ex = tx - data.qpos[0]
        w, xi, yi, zi = data.qpos[3:7]
        yaw = np.arctan2(2*(w*zi+xi*yi), 1-2*(yi*yi+zi*zi))
        heading_err = (0.0 - yaw + np.pi) % (2*np.pi) - np.pi
        steer = np.clip(1.2 * heading_err, -1.2, 1.2)
        drive = np.clip(gain * ex / 0.115, -cap, cap) * max(0.15, 1 - abs(heading_err))
        step([drive, steer, stage, hold[0], hold[1], tilt, 0], model.opt.timestep)
        if abs(ex) < 0.03:
            break
    step([0, 0, stage, hold[0], hold[1], tilt, 0], 0.5)


# 叉齒世界高度 = 0.29+lift1+lift2-0.39+0.185 ≈ 0.085 + lift1 + lift2
# 層板頂 z=0.35 → 棧板底 z=0.35，叉齒需到 0.30 進入、再升 0.15 抬起
STAGE = -0.65      # 深插到位後全程保持，不收回
TILT = 0.08        # 後傾，讓貨靠上滑架背板（23 章的做法）

print("開到貨架前...")
drive_to(-0.65)
print("牙叉對準層板下方（lift1=0.39）...")
step([0, 0, 0, 0.39, 0, 0, 0], 2.0)
print("stage 前移插入（stage=-0.65）...")
step([0, 0, STAGE, 0.39, 0, 0, 0], 3.0)
p0 = data.xpos[pid].copy()
print(f"  插入後棧板 = {np.round(p0, 3)}")
print("微升離開層板（+4cm）並後傾...")
step([0, 0, STAGE, 0.48, 0.08, TILT, 0], 1.5)
lifted_z = float(data.xpos[pid][2])
print(f"  微升後棧板 z = {lifted_z:.3f}")
rel_home = rel_to_fork()
DRIFT["home"] = rel_home          # 漂移的基準時刻：貨剛離開層板、還沒開始移動

# 在貨架內升到搬運高度會讓棧板頂部逼近上層層板（實測只剩 8 mm 餘裕），擦到就翻。
# 真實叉車也是先退出來再升高。
print("保持低位退出貨架...")
drive_to(0.3, hold=(0.48, 0.08), tilt=TILT, stage=STAGE, timeout=25.0)
print(f"  退出後棧板 z = {data.xpos[pid][2]:.3f}")
print("離架後才升到搬運高度（lift1=0.70, lift2=0.35）...")
step([0, 0, STAGE, 0.70, 0.35, TILT, 0], 2.0)
carry_z = float(data.xpos[pid][2])
print(f"  搬運高度棧板 z = {carry_z:.3f}")
print("搬到目的地...")
drive_to(1.5, hold=(0.70, 0.35), tilt=TILT, stage=STAGE, timeout=20.0)
print("放低（lift1=0.02）...")
step([0, 0, STAGE, 0.02, 0.0, TILT, 0], 2.5)
step([0, 0, STAGE, 0.02, 0.0, 0, 0], 1.5)
print(f"放下後棧板 = {np.round(data.xpos[pid], 3)}")

with open("runs/reach_xz_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "bx", "by", "bz", "qw", "qx", "qy", "qz",
                "drive", "steer", "stage", "lift1", "lift2", "tilt", "reach",
                "px", "py", "pz", "rel_x", "rel_y", "rel_z"])
    w.writerows(log_rows)

import imageio
imageio.mimsave("runs/reach_xz.mp4", frames, fps=30)
print(f"runs/reach_xz.mp4: {len(frames)} 幀")

pz = float(data.xpos[pid][2])
px = float(data.xpos[pid][0])
drift = DRIFT["peak"]      # 搬運全程的峰值，不是終點值
moved = px - float(p0[0])
print(f"最終棧板 x = {px:.2f}, z = {pz:.3f}")
print(f"棧板被搬運了 {moved:.2f} m；搬運全程在牙叉座標系中的漂移峰值 {drift*100:.1f} cm")

# 驗收要對準「真正想證明的事」：貨被抬離層板、跟著車走完全程、最後放到地面。
# 只檢查終點位置會把「掉下去」算成「放下去」（見 REPORT 第七節第三輪）。
assert lifted_z > 0.42, f"取貨失敗：棧板沒有離開層板（z={lifted_z:.3f}）"
assert drift < 0.10, f"搬運失敗：貨在叉齒上的漂移峰值 {drift*100:.1f} cm，超過 10 cm"
assert moved > 1.5, f"搬運失敗：棧板只移動 {moved:.2f} m"
assert pz < 0.10, f"放置失敗：棧板沒有降到地面（z={pz:.3f}）"
print("結果：取貨 → 搬運 → 放置 全程驗證通過 ✓")
