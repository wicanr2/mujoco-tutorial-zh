"""六軸手臂的 6D IK：位置 + 姿態同時收斂。

姿態誤差採用 MuJoCo 官方教學的寫法：
  eq = target_quat ⊗ conj(current_quat)，取向量部分 eq[1:]
（對齊附近向量部分 ≈ 旋轉向量的一半，與 jacr 參考系一致、數值上穩定。
  註：直接用 mju_subQuat 的軸角向量在本例會因參考系不一致而發散。）
"""
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/arm6.xml")
data = mujoco.MjData(model)

ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee")
JOINTS = list(range(6))


def pose_err(target_pos, target_quat):
    """回傳 6D 誤差 [位置(3), 姿態(3)]；姿態用四元數差的向量部分。"""
    mujoco.mj_fwdPosition(model, data)
    err_pos = target_pos - data.site_xpos[ee_id]
    site_quat = np.zeros(4)
    mujoco.mju_mat2Quat(site_quat, data.site_xmat[ee_id])
    conj = np.zeros(4)
    mujoco.mju_negQuat(conj, site_quat)
    eq = np.zeros(4)
    mujoco.mju_mulQuat(eq, target_quat, conj)
    if eq[0] < 0:
        eq = -eq                     # 四元數雙重覆蓋，取較短路徑
    return np.concatenate([err_pos, eq[1:]])


def ik6d(target_pos, target_quat, max_iter=3000, tol=(2e-3, 1e-2), lam=0.1):
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    for i in range(max_iter):
        err = pose_err(target_pos, target_quat)
        if np.linalg.norm(err[:3]) < tol[0] and 2 * np.linalg.norm(err[3:]) < tol[1]:
            return True, i
        mujoco.mj_jacSite(model, data, jacp, jacr, ee_id)
        J = np.vstack([jacp[:, JOINTS], jacr[:, JOINTS]])   # (6, 6)
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), err)
        data.qpos[JOINTS] += 0.4 * dq
        # 限制在關節範圍內（超出會被伺服 ctrlrange 夾住，追蹤失敗）
        data.qpos[JOINTS] = np.clip(data.qpos[JOINTS],
                                    model.jnt_range[:6, 0], model.jnt_range[:6, 1])
    return False, max_iter


# 目標：前方 0.35 m、高 0.3 m，末端 z 軸朝下（夾取姿態）
target_pos = np.array([0.35, 0.0, 0.30])
target_quat = np.array([0.0, 1.0, 0.0, 0.0])  # MuJoCo 四元數 (w,x,y,z)：繞 x 轉 180°

e0 = pose_err(target_pos, target_quat)
print(f"IK 前誤差: pos={np.linalg.norm(e0[:3]):.3f} m, "
      f"ori={2 * np.linalg.norm(e0[3:]):.3f} rad")

ok, iters = ik6d(target_pos, target_quat)
q_goal = data.qpos[JOINTS].copy()
print(f"IK 收斂: {ok}（{iters} 次迭代），關節角 = {np.round(q_goal, 3)}")

# 用 position 伺服追蹤 IK 解
data.ctrl[:] = q_goal
for _ in range(1500):
    mujoco.mj_step(model, data)

err = pose_err(target_pos, target_quat)
pos_cm = np.linalg.norm(err[:3]) * 100
ori_rad = 2 * np.linalg.norm(err[3:])
print(f"到達後誤差: pos={pos_cm:.2f} cm, ori={ori_rad:.3f} rad")

assert ok and pos_cm < 2.0 and ori_rad < 0.1
print("結果：6D 位置與姿態同時到達 ✓")
