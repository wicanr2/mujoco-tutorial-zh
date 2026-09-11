"""實驗 4：搬運車夾爪 A 取 B 放（錄影 + 記錄）。

流程：開到 A 桌旁 → IK 對準箱子 → 夾爪閉合 → 抬起 → 開到 B 桌旁 → 放下夾爪。
輸出：runs/gripper_log.csv、runs/gripper.mp4
"""
import mujoco
import numpy as np
import os
import csv

os.makedirs("runs", exist_ok=True)

model = mujoco.MjModel.from_xml_path("models/mm_gripper.xml")
data = mujoco.MjData(model)

ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee")
box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_target")
ARM = [3, 4, 5]
ACT_ARM = [3, 4, 5]
ACT_GRIP = [6, 7]

renderer = mujoco.Renderer(model, 480, 640)
cam = mujoco.MjvCamera()
cam.azimuth, cam.elevation, cam.distance = 210, -20, 2.8
cam.lookat = [0.6, 0, 0.4]
opt = mujoco.MjvOption()
opt.geomgroup[3] = 0

log_rows, frames = [], []


def step(ctrl, seconds):
    for _ in range(int(seconds / model.opt.timestep)):
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
        if not log_rows or data.time - log_rows[-1][0] >= 0.0199:
            log_rows.append([data.time, *data.qpos[:6], *data.xpos[box_id]])
            if int(data.time * 30) > len(frames) - 1:
                renderer.update_scene(data, camera=cam, scene_option=opt)
                frames.append(renderer.render().copy())


def drive_to(tx, ty, arm=(0, 0, 0), grip=0.035, timeout=8.0):
    for _ in range(int(timeout / model.opt.timestep)):
        ex, ey = tx - data.qpos[0], ty - data.qpos[1]
        c = [np.clip(1.5*ex, -0.6, 0.6), np.clip(1.5*ey, -0.6, 0.6), 0,
             arm[0], arm[1], arm[2], grip, grip]
        step(c, model.opt.timestep)
        if abs(ex) < 0.02 and abs(ey) < 0.02:
            break


def ik(target, max_iter=2000, tol=5e-3, lam=0.05):
    jacp = np.zeros((3, model.nv))
    for _ in range(max_iter):
        mujoco.mj_fwdPosition(model, data)
        err = target - data.site_xpos[ee_id]
        if np.linalg.norm(err) < tol:
            return True
        mujoco.mj_jacSite(model, data, jacp, None, ee_id)
        J = jacp[:, ARM]
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), err)
        data.qpos[ARM] += 0.3 * dq
    return False


# ===== A 取 =====
print("開到 A 桌旁...")
drive_to(0.45, 0.4)
print("IK 先到箱子上方...")
ok = ik(data.xpos[box_id] + np.array([-0.075, 0, 0.17]))
q_above = data.qpos[ARM].copy()
step([0, 0, 0, *q_above, 0.035, 0.035], 2.0)
print("上方到位 box =", np.round(data.xpos[box_id], 3))
print("垂直放下到夾取高度...")
ok2 = ik(data.xpos[box_id] + np.array([-0.075, 0, 0.01]))
q_goal = data.qpos[ARM].copy()
print(f"IK: {ok}/{ok2}, q = {np.round(q_goal, 3)}")
step([0, 0, 0, *q_goal, 0.035, 0.035], 1.5)
print(f"末端誤差 = {np.linalg.norm(data.site_xpos[ee_id] - np.array([0.85, 0.4, 0.46]))*100:.1f} cm")

print("夾爪閉合前 box =", np.round(data.xpos[box_id], 3), " ee =", np.round(data.site_xpos[ee_id], 3))
step([0, 0, 0, *q_goal, 0.008, 0.008], 1.5)
names = {i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)}
pairs = set()
for k in range(data.ncon):
    c = data.contact[k]
    b1, b2 = names[model.geom_bodyid[c.geom1]], names[model.geom_bodyid[c.geom2]]
    if "box_target" in (b1, b2):
        pairs.add((b1, b2))
gl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "grip_l")
gr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "grip_r")
print("grip qpos:", round(data.qpos[model.jnt_qposadr[gl]],4), round(data.qpos[model.jnt_qposadr[gr]],4))
fl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "finger_l")
fr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "finger_r")
print("finger y:", round(data.xpos[fl][1],3), round(data.xpos[fr][1],3), " box y:", round(data.xpos[box_id][1],3))
print("夾爪閉合後 box 接觸:", pairs, " box =", np.round(data.xpos[box_id], 3))
# weld 抓取：以「目前相對位姿」寫入約束資料，啟動時不會把箱子吸走
fl_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "finger_l")
R_fl = data.xmat[fl_id].reshape(3, 3)
rel_pos = R_fl.T @ (data.xpos[box_id] - data.xpos[fl_id])
q_fl = np.zeros(4); mujoco.mju_mat2Quat(q_fl, data.xmat[fl_id])
q_bx = np.zeros(4); mujoco.mju_mat2Quat(q_bx, data.xmat[box_id])
q_conj = np.zeros(4); mujoco.mju_negQuat(q_conj, q_fl)
rel_quat = np.zeros(4); mujoco.mju_mulQuat(rel_quat, q_conj, q_bx)
model.eq_data[0, 3:6] = rel_pos      # relpos（body1 座標系中 body2 原點）
model.eq_data[0, 6:10] = rel_quat    # 相對姿態
data.eq_active[0] = 1
print(f"weld 抓取啟動（rel_pos={np.round(rel_pos, 3)}）")
print("抬起手臂（IK 到箱子上方 0.3 m）...")
ik(data.xpos[box_id] + np.array([0, 0, 0.3]))
q_lift = data.qpos[ARM].copy()
step([0, 0, 0, *q_lift, 0.008, 0.008], 2.0)
print(f"箱子 z = {data.xpos[box_id][2]:.3f}")

# ===== B 放 =====
print("開到 B 桌旁...")
drive_to(0.0, 0.4, arm=tuple(q_lift), grip=0.008, timeout=15.0)   # 先退出 A 桌
drive_to(0.0, -0.5, arm=tuple(q_lift), grip=0.008, timeout=15.0)   # 直行到 B 旁
drive_to(0.45, -0.5, arm=tuple(q_lift), grip=0.008, timeout=15.0)  # 進入 B 桌
target_b = np.array([0.85, -0.5, 0.44])
print("到位: base =", np.round(data.qpos[:2], 2))
ok = ik(target_b)
q_place = data.qpos[ARM].copy()
print(f"place IK: {ok}, ee = {np.round(data.site_xpos[ee_id], 2)}")
step([0, 0, 0, *q_place, 0.008, 0.008], 1.5)
print("放開夾爪（先放鬆 weld、抬高、再張開）...")
data.eq_active[0] = 0
step([0, 0, 0, *q_place, 0.008, 0.008], 0.5)      # 靜置讓箱子落穩
ik(data.xpos[box_id] + np.array([-0.05, 0, 0.25]))
q_up = data.qpos[ARM].copy()
step([0, 0, 0, *q_up, 0.035, 0.035], 1.5)          # 抬高並張開
drive_to(0.2, -0.5, arm=tuple(q_up), grip=0.035, timeout=10.0)
print(f"箱子最終位置 = {np.round(data.xpos[box_id], 3)}")

with open("runs/gripper_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "bx", "by", "byaw", "q3", "q4", "q5", "px", "py", "pz"])
    w.writerows(log_rows)

import imageio
imageio.mimsave("runs/gripper.mp4", frames, fps=30)
print(f"runs/gripper.mp4: {len(frames)} 幀")

p = data.xpos[box_id]

# 只檢查終點的話，箱子被「推」到 B 桌也會算通過。加一條過程檢查：它必須真的被舉高過。
# log 的第 9 欄（索引 9）是箱子的 z。
pz = np.array([r[9] for r in log_rows])
z_max = float(pz.max())
z_rise = z_max - float(pz[0])
print(f"箱子最高到過 z={z_max:.3f}（起點 {pz[0]:.3f}，抬升 {z_rise*100:.1f} cm）")

assert z_rise > 0.10, f"抓取失敗：箱子只被抬高 {z_rise*100:.1f} cm，可能是被推過去的"
assert abs(p[0] - 0.9) < 0.15 and abs(p[1] + 0.5) < 0.15 and p[2] < 0.5, "A取B放失敗"
print("結果：A 取 B 放驗證通過 ✓")
