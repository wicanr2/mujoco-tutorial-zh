"""搬運車 + 機械手臂示範：開到貨架旁，再用阻尼最小平方法 IK 把末端移到箱子位置。

IK 用 MuJoCo 內建的 mj_jacSite 取雅可比矩陣（Jacobian），
以 dposed-lsq 迭代：Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ · error
"""
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/mobile_manipulator.xml")
data = mujoco.MjData(model)

ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee")

# 手臂關節在 qpos/qvel 中的索引（前 3 個是底盤 x/y/yaw）
ARM_JOINTS = [3, 4, 5]
ARM_ACT = [3, 4, 5]  # shoulder_ctl, elbow_ctl, wrist_ctl


def ee_pos():
    mujoco.mj_fwdPosition(model, data)
    return data.site_xpos[ee_id].copy()


def ik(target, max_iter=2000, tol=5e-3, lam=0.05):
    """阻尼最小平方法 IK，只調整手臂三關節。回傳收斂與否。"""
    jacp = np.zeros((3, model.nv))
    for i in range(max_iter):
        mujoco.mj_fwdPosition(model, data)
        err = target - data.site_xpos[ee_id]
        if np.linalg.norm(err) < tol:
            return True
        mujoco.mj_jacSite(model, data, jacp, None, ee_id)
        J = jacp[:, ARM_JOINTS]                    # 只取手臂關節的雅可比
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), err)
        data.qpos[ARM_JOINTS] += 0.3 * dq   # 步長縮放（硬 clip 會造成震盪）
    return False


def settle(seconds):
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)


# --- 階段 1：開到貨架旁（x → 0.6, y → 0.5）---
print(f"起始底盤: x={data.qpos[0]:.2f} y={data.qpos[1]:.2f}")
for _ in range(int(3.0 / model.opt.timestep)):
    # P 控制 + 速度上限，快到時自動減速（誤差小 → 命令小）
    data.ctrl[0] = np.clip(1.5 * (0.65 - data.qpos[0]), -0.6, 0.6)
    data.ctrl[1] = np.clip(1.5 * (0.4 - data.qpos[1]), -0.6, 0.6)
    mujoco.mj_step(model, data)
data.ctrl[0] = data.ctrl[1] = 0
settle(1.0)  # 煞車停穩
print(f"到位後底盤: x={data.qpos[0]:.2f} y={data.qpos[1]:.2f}")

# --- 階段 2：IK 求手臂關節角，讓末端抵達箱子 ---
target = np.array([1.2, 0.5, 0.48])  # 箱子初始位置（貨架上）
print(f"箱子位置: {np.round(target, 3)}")
print(f"IK 前末端位置: {np.round(ee_pos(), 3)}")

ok = ik(target)
q_goal = data.qpos[ARM_JOINTS].copy()
print(f"IK 收斂: {ok}, 目標關節角 = {np.round(q_goal, 3)}")

# --- 階段 3：用 position 伺服把手臂開到 IK 解 ---
data.ctrl[ARM_ACT] = q_goal
settle(2.0)
final = ee_pos()
err = np.linalg.norm(final - target)
print(f"到達後末端位置: {np.round(final, 3)}, 誤差 = {err * 100:.1f} cm")

assert ok and err < 0.05, "IK 或追蹤失敗"
print("結果：搬運車機械手臂成功抵達目標 ✓")
