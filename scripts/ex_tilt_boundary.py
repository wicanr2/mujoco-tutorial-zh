"""門架前傾實驗：找棧板掉落的臨界角度（木頭 vs 塑膠、有無重心偏移）。

門架緩慢前傾，記錄棧板「明顯滑動」與「掉落」的角度。
理論：滑動臨界角 θ = atan(μ)；重心偏移會讓翻倒提早發生。
"""
import mujoco
import numpy as np

TEMPLATE = open("models/amr_tilt_template.xml").read()

CASES = {
    "木頭棧板":           {"friction": "0.6",  "rgba": "0.55 0.38 0.2 1",  "cg": "0"},
    "塑膠棧板":           {"friction": "0.35", "rgba": "0.2  0.45 0.8 1",  "cg": "0"},
    "塑膠棧板+重心前移10cm": {"friction": "0.35", "rgba": "0.2  0.45 0.8 1", "cg": "0.10"},
}


def run_case(name, cfg, max_tilt_deg=45):
    xml = (TEMPLATE.replace("PALLET_FRICTION", cfg["friction"])
                   .replace("PALLET_RGBA", cfg["rgba"])
                   .replace("CG_OFFSET", cfg["cg"]))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet")
    tilt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "tilt")
    qadr = model.jnt_qposadr[tilt_id]

    # 靜置穩定
    for _ in range(250):
        mujoco.mj_step(model, data)
    p0 = data.xpos[pid].copy()

    slip_deg = fall_deg = None
    steps = int(12 / model.opt.timestep)          # 12 秒慢慢傾到 max_tilt
    for i in range(steps):
        target = np.deg2rad(max_tilt_deg) * i / steps
        data.ctrl[:] = [target, 0.1]              # 傾斜掃描，牙叉略升避免地板接觸
        mujoco.mj_step(model, data)

        p = data.xpos[pid]
        tilt_deg = np.rad2deg(data.qpos[qadr])
        rel = np.linalg.norm(p[:2] - p0[:2])
        if slip_deg is None and rel > 0.03:        # 滑動 > 3 cm
            slip_deg = tilt_deg
        # 掉落判定：棧板翻轉（自身 z 軸偏離世界 z 超過 30°）或滑出 25 cm
        R = data.xmat[pid].reshape(3, 3)
        tilt_of_pallet = np.rad2deg(np.arccos(np.clip(R[2, 2], -1, 1)))
        if tilt_of_pallet > 30 or rel > 0.25:
            fall_deg = tilt_deg
            break

    print(f"{name}: 開始滑動 @ {slip_deg if slip_deg is None else round(slip_deg,1)}°"
          f", 掉落 @ {fall_deg if fall_deg is None else round(fall_deg,1)}°"
          f"（理論滑動角 atan(μ) = {np.rad2deg(np.arctan(float(cfg['friction']))):.1f}°）")
    return slip_deg, fall_deg


print("門架緩慢前傾（0→45°），棧板載重 20 kg\n")
for name, cfg in CASES.items():
    run_case(name, cfg)
