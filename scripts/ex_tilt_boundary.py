"""門架前傾實驗：找棧板掉落的臨界角度（木頭 vs 塑膠、有無重心偏移）。

門架緩慢前傾，記錄棧板「明顯滑動」與「掉落」的角度。
理論：滑動臨界角 θ = atan(μ)；重心偏移會讓翻倒提早發生。
"""
import csv
import os

import mujoco
import numpy as np

TEMPLATE = open("models/amr_tilt_template.xml").read()

CASES = {
    "木頭棧板":           {"friction": "0.6",  "rgba": "0.55 0.38 0.2 1",  "cg": "0"},
    "塑膠棧板":           {"friction": "0.35", "rgba": "0.2  0.45 0.8 1",  "cg": "0"},
    "塑膠棧板+重心前移10cm": {"friction": "0.35", "rgba": "0.2  0.45 0.8 1", "cg": "0.10"},
}


ROWS = []      # (case, time, tilt_deg, slip_cm, pallet_tilt_deg)


def run_case(name, cfg, max_tilt_deg=45):
    xml = (TEMPLATE.replace("PALLET_FRICTION", cfg["friction"])
                   .replace("PALLET_RGBA", cfg["rgba"])
                   .replace("CG_OFFSET", cfg["cg"]))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet")
    fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "forks")
    tilt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "tilt")
    qadr = model.jnt_qposadr[tilt_id]

    def in_fork_frame():
        """棧板在叉齒座標系中的位置與姿態偏離。

        門架會旋轉，所以不能像 16 章那樣「減掉牙叉的平移」就當成相對滑動 —— 那會
        把「棧板跟著門架轉過去」算成滑動。姿態同理：拿棧板對**世界**的傾角判翻倒，
        棧板只要乖乖貼在傾斜的叉齒上就會被誤判成翻倒。
        """
        Rf = data.xmat[fid].reshape(3, 3)
        pos = Rf.T @ (data.xpos[pid] - data.xpos[fid])
        Rrel = Rf.T @ data.xmat[pid].reshape(3, 3)
        return pos, np.degrees(np.arccos(np.clip(Rrel[2, 2], -1, 1)))

    # 靜置穩定。ctrl 還是 0，但 kp=500 撐不住 20 kg 棧板的力矩，門架會先被重力
    # 拉下來一段 —— 所以掃描的起點不是 0°。
    for _ in range(250):
        mujoco.mj_step(model, data)
    p0 = data.xpos[pid].copy()
    fork0, _ = in_fork_frame()
    start_deg = np.degrees(data.qpos[qadr])

    slip_deg = fall_deg = None
    max_vs_fork = 0.0          # 追峰值，不是看最後一幀（24 章的教訓）
    steps = int(12 / model.opt.timestep)          # 12 秒慢慢傾到 max_tilt
    t0 = data.time
    for i in range(steps):
        target = np.deg2rad(max_tilt_deg) * i / steps
        data.ctrl[:] = [target, 0.1]              # 傾斜掃描，牙叉略升避免地板接觸
        mujoco.mj_step(model, data)

        p = data.xpos[pid]
        tilt_deg = np.rad2deg(data.qpos[qadr])
        target_deg = np.degrees(target)
        fork_pos, pallet_vs_fork = in_fork_frame()
        rel = float(np.linalg.norm(fork_pos - fork0))          # 叉齒座標系的真實滑移
        rel_world = float(np.linalg.norm(p[:2] - p0[:2]))      # 世界座標，含門架帶動的平移
        if slip_deg is None and rel > 0.03:        # 滑動 > 3 cm
            slip_deg = tilt_deg
        # 掉落判定：棧板相對叉齒翻轉超過 30°，或在叉齒座標系滑出 25 cm
        tilt_of_pallet = pallet_vs_fork
        max_vs_fork = max(max_vs_fork, pallet_vs_fork)
        # 50 Hz 取樣寫進 CSV：只印三個臨界角度的話，看不出「怎麼滑過去的」
        if not ROWS or ROWS[-1][0] != name or (data.time - t0) - ROWS[-1][1] >= 0.0199:
            ROWS.append([name, round(data.time - t0, 4), round(target_deg, 3),
                         round(tilt_deg, 3), round(rel * 100, 4),
                         round(rel_world * 100, 4), round(tilt_of_pallet, 3)])
        if tilt_of_pallet > 30 or rel > 0.25:
            fall_deg = tilt_deg
            break

    print(f"{name}: 門架 {start_deg:.1f}° → {tilt_deg:.1f}°（目標掃到 {max_tilt_deg}°，"
          f"kp=500 追不上）\n"
          f"    叉齒座標系滑移 {rel*100:.1f} cm"
          f"（世界座標量會是 {rel_world*100:.1f} cm）"
          f"，棧板相對叉齒最大偏轉 {max_vs_fork:.1f}°\n"
          f"    開始滑動 @ {slip_deg if slip_deg is None else str(round(slip_deg,1)) + '°'}"
          f"，掉落 @ {fall_deg if fall_deg is None else str(round(fall_deg,1)) + '°'}"
          f"（理論滑動角 atan(μ) = {np.rad2deg(np.arctan(float(cfg['friction']))):.1f}°）")
    return slip_deg, fall_deg


print("門架緩慢前傾（0→45°），棧板載重 20 kg\n")
for name, cfg in CASES.items():
    run_case(name, cfg)

# 門架為什麼傾不到目標角：position 伺服的增益不足以撐住 20 kg 棧板的力矩。
# 掃一遍 kp 看它能追到哪 —— 這是「ctrl 是目標值不是實際值」最直接的證據。
print("\nkp 掃描（木頭棧板，目標同樣掃到 45°）：")
KP_ROWS = []
for kp in (500, 1500, 3000, 8000):
    xml = (TEMPLATE.replace("PALLET_FRICTION", "0.6")
                   .replace("PALLET_RGBA", "0.55 0.38 0.2 1")
                   .replace("CG_OFFSET", "0")
                   .replace('kp="500" kv="50"', f'kp="{kp}" kv="{kp // 10}"'))
    m2 = mujoco.MjModel.from_xml_string(xml)
    d2 = mujoco.MjData(m2)
    pid2 = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_BODY, "pallet")
    fid2 = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_BODY, "forks")
    tid2 = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_JOINT, "tilt")
    qa2 = m2.jnt_qposadr[tid2]
    for _ in range(250):
        mujoco.mj_step(m2, d2)
    Rf = d2.xmat[fid2].reshape(3, 3)
    home = Rf.T @ (d2.xpos[pid2] - d2.xpos[fid2])
    start2 = np.degrees(d2.qpos[qa2])
    n = int(12 / m2.opt.timestep)
    for i in range(n):
        d2.ctrl[:] = [np.deg2rad(45) * i / n, 0.1]
        mujoco.mj_step(m2, d2)
    Rf = d2.xmat[fid2].reshape(3, 3)
    slip2 = float(np.linalg.norm(Rf.T @ (d2.xpos[pid2] - d2.xpos[fid2]) - home)) * 100
    end2 = np.degrees(d2.qpos[qa2])
    KP_ROWS.append([kp, round(start2, 2), round(end2, 2), round(slip2, 2)])
    print(f"  kp={kp:5d}  靜置起點 {start2:5.2f}°  掃描終點 {end2:5.1f}°  滑移 {slip2:.1f} cm")

os.makedirs("runs", exist_ok=True)
with open("runs/tilt_kp_sweep.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["kp", "rest_deg", "final_deg", "slip_cm"])
    w.writerows(KP_ROWS)
with open("runs/tilt_boundary_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    # slip_cm 是叉齒座標系的真實滑移；slip_world_cm 是世界座標，含門架帶動的平移
    w.writerow(["case", "time", "target_deg", "tilt_deg", "slip_cm",
                "slip_world_cm", "pallet_vs_fork_deg"])
    w.writerows(ROWS)
print(f"\nruns/tilt_boundary_log.csv 已寫出（{len(ROWS)} 列）")
