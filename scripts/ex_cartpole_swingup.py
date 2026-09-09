"""進階範例：車桿（cart-pole）swing-up — 能量整形盪起 + LQR 接手穩定。

兩段式經典控制：
1. Åström 能量整形：對車施力，把桿的總能量推到「直立位能」的高度。
2. LQR：用 mjd_transitionFD 在直立平衡點做數值線性化，解 Riccati 方程得增益 K。

需安裝 scipy：pip install scipy
"""
import mujoco
import numpy as np
from scipy.linalg import solve_continuous_are

model = mujoco.MjModel.from_xml_path("models/cartpole_swingup.xml")
m_pole = model.body_mass[2]
L_COM, G = 0.25, 9.81          # 桿質心距關節 0.25 m
E_UPRIGHT = 2 * m_pole * G * L_COM  # 直立位能（以垂下為零點的能量慣性系）

# --- 步驟 1：在直立平衡點（x=0, θ=0）數值線性化 ---
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
nx = model.nq + model.nv
A_d = np.zeros((nx, nx))
B_d = np.zeros((nx, model.nu))
mujoco.mjd_transitionFD(model, data, 1e-6, True, A_d, B_d, None, None)

# mjd_transitionFD 給的是離散時間 Jacobian，轉成連續時間再解 CARE
dt = model.opt.timestep
A_c = (A_d - np.eye(nx)) / dt
B_c = B_d / dt
Q = np.diag([1.0, 100.0, 1.0, 10.0])   # 重罰桿角度
R = np.array([[0.1]])
P = solve_continuous_are(A_c, B_c, Q, R)
K = np.linalg.solve(R, B_c.T @ P)[0]
print("LQR 增益 K =", np.round(K, 2))

# --- 步驟 2：模擬 — 先能量整形盪起，接近直立切 LQR ---
data = mujoco.MjData(model)
data.qpos[1] = np.pi            # 桿從垂下出發
swung_up = False
t_up = None

for _ in range(8000):           # 16 秒
    x, th = data.qpos
    xd, thd = data.qvel

    if not swung_up and abs(th) < 0.4:
        swung_up, t_up = True, data.time

    if swung_up:
        u = -K @ np.array([x, th, xd, thd])            # LQR 穩定直立
    else:
        # Åström 能量整形：u = k (E - E0) sign(ω cosθ)
        E = 0.5 * m_pole * L_COM**2 * thd**2 + m_pole * G * L_COM * (1 + np.cos(th))
        u = (E - E_UPRIGHT) * np.sign(thd * np.cos(th))
        if abs(thd * np.cos(th)) < 1e-3:
            u = 3.0                                    # 靜止時踢一下啟動
    data.ctrl[0] = float(np.clip(u, -10, 10))
    mujoco.mj_step(model, data)

x, th = data.qpos
print(f"swing-up 時刻: {t_up:.2f} s")
print(f"最終狀態: x = {x:.3f} m, θ = {th:.3f} rad（應都接近 0）")
assert abs(x) < 0.05 and abs(th) < 0.05, "swing-up 失敗"
print("結果: 成功直立並穩定 ✓")
