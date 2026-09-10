"""Blender 版棧板（木頭/塑膠，20 kg）× MR1533 叉車：上上下下左右左右驗證。

棧板在地板上，叉車開過來插取、抬起，然後升降 + 橫移，量測棧板相對牙叉的滑動。
"""
import mujoco
import numpy as np

TEMPLATE = open("models/mr1533_pallet_template.xml").read()

MATERIALS = {
    "木頭棧板": {"obj": "pallet_wood.stl",    "friction": "0.6",  "rgba": "0.45 0.30 0.15 1"},
    "塑膠棧板": {"obj": "pallet_plastic.stl", "friction": "0.35", "rgba": "0.15 0.35 0.75 1"},
}


def experiment(name, cfg):
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
        """棧板相對叉車底盤的（x, y）位置。"""
        return data.xpos[pid][:2] - data.qpos[:2]

    print(f"\n=== {name}（Blender mesh, 20 kg, μ={cfg['friction']}）===")

    phase([0, 0, 0, 0, 0, 0], 0.5)                      # 靜置
    phase([-0.5, 0, 0, 0, 0, 0], 1.5)                   # 倒退讓牙叉插入棧板下方
    p = data.xpos[pid]
    print(f"插入後: 底盤 x={data.qpos[0]:.2f}, 棧板 z={p[2]:.3f}")

    phase([0, 0, 0, 0.35, 0.2, 0], 2.0)                 # 抬起
    home = rel()
    print(f"抬起後: 棧板 z={data.xpos[pid][2]:.3f}（相對車位 {np.round(home, 2)}）")

    phase([0, 0, 0, 0.05, 0.05, 0], 1.5)                # 降
    phase([0, 0, 0, 0.35, 0.2, 0], 1.5)                 # 再升
    phase([0, 0.6, 0, 0.35, 0.2, 0], 1.0)               # 急左移
    phase([0, -0.6, 0, 0.35, 0.2, 0], 1.0)              # 急右移
    phase([0, 0, 0, 0.35, 0.2, 0], 1.0)                 # 停穩

    p = data.xpos[pid]
    slip = np.linalg.norm(rel() - home)
    fell = p[2] < 0.05
    print(f"  最終棧板 z={p[2]:.3f}, 相對滑動 = {slip * 100:.1f} cm, 掉落 = {fell}")
    return slip, fell


results = {name: experiment(name, cfg) for name, cfg in MATERIALS.items()}
print("\n===== 比較 =====")
for name, (slip, fell) in results.items():
    print(f"{name}: 相對滑動 {slip * 100:.1f} cm, {'✗ 掉落' if fell else '✓ 仍在牙叉上'}")
    assert not fell
print("\n結果：Blender 版棧板上下左右驗證通過 ✓")
