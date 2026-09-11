"""新模型重跑 16 章：材質摩擦實驗（MR1533 + Blender 棧板，20 kg）。

與 16 章差異：真實比例車（2000 kg 底盤）、真實 mesh 棧板、
橫移由底盤 vy 急打方向提供（MR1533 無側移機構）。
流程：插取 → 抬起 → 急左/急右 ×3 → 量相對滑動，並掃 vy 找可分辨材質的區間。

輸出 runs/rerun_materials_log.csv（vy 掃描）。
"""
import csv
import os

import mujoco
import numpy as np

TEMPLATE = open("models/mr1533_pallet_template.xml").read()

MATERIALS = {
    "木頭": {"obj": "pallet_wood.stl",    "friction": "0.6",  "rgba": "0.45 0.30 0.15 1"},
    "塑膠": {"obj": "pallet_plastic.stl", "friction": "0.35", "rgba": "0.15 0.35 0.75 1"},
}


def experiment(name, cfg, vy=0.2, cycles=3):
    xml = (TEMPLATE.replace("PALLET_OBJ", cfg["obj"])
                   .replace("PALLET_FRICTION", cfg["friction"])
                   .replace("PALLET_RGBA", cfg["rgba"]))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet")

    def phase(ctrl, seconds):
        for _ in range(int(seconds / model.opt.timestep)):
            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)

    def rel():
        return data.xpos[pid][:2] - data.qpos[:2]

    phase([0, 0, 0, 0, 0, 0], 0.5)
    phase([-0.5, 0, 0, 0, 0, 0], 1.5)          # 倒退插入
    phase([0, 0, 0, 0.35, 0.2, 0], 2.0)        # 抬起
    home = rel()
    z_lifted = data.xpos[pid][2]

    # 只做橫移，不做升降。升降那段會讓棧板脫離叉齒（降到 lift 總和 0.25 以下就碰地，
    # 再升起來是「重新插取」而不是「載著升降」），量到的是插取偏移，不是材質滑動。
    for _ in range(cycles):
        phase([0,  vy, 0, 0.35, 0.2, 0], 0.6)
        phase([0, -vy, 0, 0.35, 0.2, 0], 0.6)
    phase([0, 0, 0, 0.35, 0.2, 0], 1.5)        # 停穩

    p = data.xpos[pid]
    slip = np.linalg.norm(rel() - home)
    fell = p[2] < 0.05
    return slip, fell, z_lifted


def tilt_unload(name="木頭"):
    """實驗二：前傾卸貨（重跑 17 章）。

    量測要在**牙叉座標系**：tilt 會轉動牙叉，用底盤座標量會把幾何位移誤判成滑動。
    """
    cfg = MATERIALS[name]
    xml = (TEMPLATE.replace("PALLET_OBJ", cfg["obj"])
                   .replace("PALLET_FRICTION", cfg["friction"])
                   .replace("PALLET_RGBA", cfg["rgba"]))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet")
    fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "fork")
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "tilt")
    qa = model.jnt_qposadr[tid]

    def phase(ctrl, seconds):
        for _ in range(int(seconds / model.opt.timestep)):
            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)

    def in_fork():
        Rf = data.xmat[fid].reshape(3, 3)
        pos = Rf.T @ (data.xpos[pid] - data.xpos[fid])
        Rrel = Rf.T @ data.xmat[pid].reshape(3, 3)
        return pos, np.degrees(np.arccos(np.clip(Rrel[2, 2], -1, 1)))

    phase([0, 0, 0, 0, 0, 0], 0.5)
    phase([-0.5, 0, 0, 0, 0, 0], 1.5)
    phase([0, 0, 0, 0.35, 0.2, 0], 2.0)
    home, _ = in_fork()

    print(f"\n=== 實驗二：前傾卸貨（{name}棧板）===")
    steps = int(6.0 / model.opt.timestep)
    marks, next_mark = [], -3.0
    for i in range(steps):
        target = -0.5 * i / steps                      # 0 → -0.5 rad（-28.6°）
        data.ctrl[:] = [0, 0, 0, 0.35, 0.2, target]
        mujoco.mj_step(model, data)
        deg = np.degrees(data.qpos[qa])
        if deg <= next_mark:
            pos, vs_fork = in_fork()
            pitch = np.degrees(np.arccos(np.clip(
                data.xmat[pid].reshape(3, 3)[2, 2], -1, 1)))
            print(f"tilt={deg:6.1f}°  棧板 z={data.xpos[pid][2]:.3f}  俯仰={pitch:4.1f}°  "
                  f"牙叉座標系滑移={np.linalg.norm(pos - home)*100:5.1f} cm  "
                  f"相對牙叉偏轉={vs_fork:4.1f}°")
            marks.append(deg)
            next_mark -= 3.0
    pos, vs_fork = in_fork()
    print(f"最終：棧板 z={data.xpos[pid][2]:.3f}，牙叉座標系滑移 "
          f"{np.linalg.norm(pos - home)*100:.1f} cm，相對牙叉偏轉 {vs_fork:.1f}°")
    return float(data.xpos[pid][2])


print("=== 新模型（MR1533 + Blender 棧板）重跑材質實驗 ===\n")

# 橫移強度掃描：材質差異只在「摩擦剛好不夠」的區間看得到。
# vy 太小兩者都不滑、太大兩者都滑到飽和甚至甩落 —— 16 章那組數字就落在中間帶。
print("實驗一：橫移強度掃描（插取 → 抬起 → 急左右 ×3）")
print(f"{'vy (m/s)':>9}  {'木頭滑移':>9} {'':>4}  {'塑膠滑移':>9} {'':>4}   差距")
rows = []
for vy in (0.2, 0.35, 0.5, 0.7, 0.9, 1.2):
    out = {n: experiment(n, c, vy=vy) for n, c in MATERIALS.items()}
    w, p = out["木頭"], out["塑膠"]
    rows.append([vy, round(w[0]*100, 1), int(w[1]), round(p[0]*100, 1), int(p[1]),
                 round(abs(w[0]-p[0])*100, 1)])
    print(f"{vy:9.2f}  {w[0]*100:7.1f} cm {'掉落' if w[1] else '留住'}  "
          f"{p[0]*100:7.1f} cm {'掉落' if p[1] else '留住'}  {abs(w[0]-p[0])*100:6.1f} cm")

os.makedirs("runs", exist_ok=True)
with open("runs/rerun_materials_log.csv", "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["vy", "wood_slip_cm", "wood_fell", "plastic_slip_cm", "plastic_fell", "gap_cm"])
    wr.writerows(rows)
print("runs/rerun_materials_log.csv 已寫出")

best = min(rows, key=lambda r: -r[5] if not (r[2] or r[4]) else 1e9)
print(f"\n兩種材質都留在叉齒上、差距最大的是 vy={best[0]} m/s："
      f"木頭 {best[1]} cm、塑膠 {best[3]} cm（差 {best[5]} cm）")
assert best[5] > 10, "材質差異應該在低速區顯現"

final_z = tilt_unload()
print("\n驗證通過 ✓")
