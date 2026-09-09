"""棧板材質實驗：20 kg 木頭棧板 vs 塑膠棧板放在牙叉上，上下左右移動驗證。

比較兩種材質的摩擦係數（木頭 μ=0.6、塑膠 μ=0.35）在牙叉動作下的滑動量。
流程：升 → 降 → 左移 → 右移 → 再升降，全程量測棧板相對牙叉的滑動。
"""
import mujoco
import numpy as np

TEMPLATE = open("models/amr_pallet_template.xml").read()

MATERIALS = {
    "木頭棧板": {"friction": "0.6",  "rgba": "0.55 0.38 0.2 1"},
    "塑膠棧板": {"friction": "0.35", "rgba": "0.2  0.45 0.8 1"},
}


def build(friction, rgba):
    xml = TEMPLATE.replace("PALLET_FRICTION", friction).replace("PALLET_RGBA", rgba)
    return mujoco.MjModel.from_xml_string(xml)


def run_phase(model, data, lift, shift, seconds):
    for _ in range(int(seconds / model.opt.timestep)):
        data.ctrl[:] = [lift, shift]
        mujoco.mj_step(model, data)


def experiment(name, friction, rgba):
    model = build(friction, rgba)
    data = mujoco.MjData(model)

    pallet_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet")

    print(f"\n=== {name}（20 kg, μ={friction}）===")

    # 讓棧板先在牙叉上靜置穩定，再記錄起始位置
    run_phase(model, data, 0, 0, 0.5)
    home = data.xpos[pallet_id].copy()
    print(f"起始位置: {np.round(home, 3)}")

    seq = [("上升",     0.40,  0.0,  1.5),
           ("下降",     0.05,  0.0,  1.5),
           ("急左移",   0.05,  0.12, 0.6),
           ("急右移",   0.05, -0.12, 0.6),
           ("急左移2",  0.05,  0.12, 0.6),
           ("急右移2",  0.05, -0.12, 0.6),
           ("再上升",   0.40,  0.0,  1.5),
           ("再下降",   0.0,   0.0,  1.5)]

    for label, lift, shift, sec in seq:
        run_phase(model, data, lift, shift, sec)
        p = data.xpos[pallet_id]
        rel_y = p[1] - data.qpos[1]   # 相對牙叉側移量的 y 偏差 = 真實滑動
        print(f"  {label} → 棧板 x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}（相對滑動 y={rel_y:+.3f}）")

    p = data.xpos[pallet_id]
    slip = np.linalg.norm([p[0] - home[0], p[1] - data.qpos[1] - home[1]])  # 扣除牙叉位移
    fell = p[2] < 0.05  # 腳底原本在 0.082，掉到地板表示摔下來
    print(f"  相對滑動量 = {slip * 100:.1f} cm, 掉落 = {fell}")
    return slip, fell


results = {}
for name, cfg in MATERIALS.items():
    results[name] = experiment(name, **cfg)

print("\n===== 比較 =====")
for name, (slip, fell) in results.items():
    status = "✗ 掉落" if fell else "✓ 仍在牙叉上"
    print(f"{name}: 相對滑動 {slip * 100:.1f} cm, {status}")

for name, (slip, fell) in results.items():
    assert not fell, f"{name} 掉落了！"
print("\n結果：兩種棧板都通過上下左右移動驗證 ✓")
