"""AMR 叉車示範：底盤移動 + 牙叉升降 / 側移控制。

流程（模擬一個簡化的取貨動作）：
1. 開到棧板前方
2. 牙叉降到最低
3. 前進讓牙叉插入棧板下方
4. 升起牙叉把貨物抬起
5. 側移牙叉修正對位
"""
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/amr_forklift.xml")
data = mujoco.MjData(model)

IDX = {"vx": 0, "vy": 1, "wyaw": 2, "lift": 3, "shift": 4}


def run_phase(seconds, cmd):
    """以固定控制命令跑 seconds 秒。"""
    for _ in range(int(seconds / model.opt.timestep)):
        data.ctrl[:] = cmd
        mujoco.mj_step(model, data)


def state():
    lift_h = data.qpos[3]
    shift = data.qpos[4]
    tip = data.sensordata[2:5]  # fork_tip 世界座標
    return lift_h, shift, tip


print("起始：", np.round(data.qpos[:3], 2))

# 1. 先橫移對準棧板（y → 0.3），再往前開
run_phase(1.2, [0, 0.5, 0, 0.65, 0])
print(f"橫移後底盤 y = {data.qpos[1]:.2f} m")
run_phase(1.5, [0.8, 0, 0, 0.65, 0])
print(f"前進後底盤 x = {data.qpos[0]:.2f} m")

# 2. 牙叉降到最低
run_phase(1.0, [0, 0, 0, 0.0, 0])
lift_h, shift, tip = state()
print(f"牙叉降到最低：lift = {lift_h:.3f} m, 叉尖 z = {tip[2]:.3f} m")

# 3. 前進插入棧板下方
run_phase(1.0, [0.5, 0, 0, 0.0, 0])
print(f"插入後底盤 x = {data.qpos[0]:.2f} m")

# 4. 升起牙叉（把貨物抬起）
run_phase(2.0, [0, 0, 0, 0.35, 0])
lift_h, shift, tip = state()
print(f"升起後：lift = {lift_h:.3f} m, 叉尖 z = {tip[2]:.3f} m")

# 5. 側移牙叉（左移 0.1 m）
run_phase(1.0, [0, 0, 0, 0.35, -0.10])
lift_h, shift, tip = state()
print(f"側移後：shift = {shift:.3f} m, 叉尖 y = {tip[1]:.3f} m")

pallet_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet")
pz = data.xpos[pallet_id][2]
print(f"棧板高度 z = {pz:.3f} m（初始 0.0，>0.05 表示被牙叉抬起）")

assert tip[2] > 0.3, "牙叉沒有升起"
assert pz > 0.05, "棧板沒有被抬起"
assert abs(shift + 0.10) < 0.02, "側移沒到位"
print("結果：取貨動作完成 ✓")
