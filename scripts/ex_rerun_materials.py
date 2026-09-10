"""新模型重跑 16 章：材質摩擦實驗（MR1533 + Blender 棧板，20 kg）。

與 16 章差異：真實比例車（2000 kg 底盤）、真實 mesh 棧板、
橫移由底盤 vy 急打方向提供（MR1533 無側移機構）。
流程：插取 → 抬起 → 升/降 → 急左/急右 ×3 → 量相對滑動。
"""
import mujoco
import numpy as np

TEMPLATE = open("models/mr1533_pallet_template.xml").read()

MATERIALS = {
    "木頭": {"obj": "pallet_wood.stl",    "friction": "0.6",  "rgba": "0.45 0.30 0.15 1"},
    "塑膠": {"obj": "pallet_plastic.stl", "friction": "0.35", "rgba": "0.15 0.35 0.75 1"},
}


def experiment(name, cfg, frames=False):
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

    # 升降兩次
    phase([0, 0, 0, 0.05, 0.05, 0], 1.2)
    phase([0, 0, 0, 0.35, 0.2, 0], 1.2)
    phase([0, 0, 0, 0.05, 0.05, 0], 1.2)
    phase([0, 0, 0, 0.35, 0.2, 0], 1.2)

    # 急打橫移三次（vy 滿幅切換）
    for _ in range(3):
        phase([0, 1.2, 0, 0.35, 0.2, 0], 0.6)
        phase([0, -1.2, 0, 0.35, 0.2, 0], 0.6)
    phase([0, 0, 0, 0.35, 0.2, 0], 1.5)        # 停穩

    p = data.xpos[pid]
    slip = np.linalg.norm(rel() - home)
    fell = p[2] < 0.05
    print(f"{name}: 抬起 z={z_lifted:.3f} m, 相對滑動 = {slip*100:.1f} cm, 掉落 = {fell}")
    return slip, fell


print("=== 新模型（MR1533 + Blender 棧板）重跑材質實驗 ===\n")
results = {n: experiment(n, c) for n, c in MATERIALS.items()}
print()
for n, (slip, fell) in results.items():
    print(f"{n}: 相對滑動 {slip*100:.1f} cm, {'✗ 掉落' if fell else '✓ 留在牙叉上'}")
    assert not fell
print("\n驗證通過 ✓")
