"""把 runs/ 的 CSV 畫成圖：RL 學習曲線、17 章滑動曲線、10 章車桿軌跡。

輸出：runs/rl_curves.png、runs/tilt_boundary.png、runs/cartpole_traj.png

用法：
    python scripts/make_curves.py            # 全部
    python scripts/make_curves.py rl         # 只畫指定的（rl / tilt / cartpole）

圖上的文字一律用英文 —— headless 環境的 matplotlib 預設字型沒有 CJK，中文會變豆腐字，
而這支腳本要能在任何人的機器上跑出一樣的圖。中文說明寫在引用它的章節裡。
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNS = REPO_ROOT / "runs"

# ARS 每輪跑 N_DIR×2 = 16 個 episode、每個 1000 步。要跟 PPO / SAC 放在同一張圖上
# 比較，x 軸得換成共同單位：實際跑過的環境步數。（每輪的評估不算訓練樣本。）
ARS_STEPS_PER_ITER = 16 * 1000

INK = "#1a1a1a"
ACCENT = "#a4531d"        # 關鍵路徑色，與網站同一個鏽橙


def read_csv(name):
    with open(RUNS / name, newline="") as f:
        r = csv.DictReader(f)
        return list(r)


def style(ax):
    """細線、淺網格、不畫上右邊框。"""
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_linewidth(0.8)
        ax.spines[side].set_color("#999")
    ax.grid(True, lw=0.5, color="#e2e2e2")
    ax.set_axisbelow(True)
    ax.tick_params(labelsize=9, color="#999")


def rl_curves():
    """三種演算法的學習曲線。

    x 軸用「環境步數」而不是各自的訓練單位 —— ARS 數的是輪，PPO / SAC 數的是步，
    不換算成同一個單位就沒得比。換算之後才看得到一件從訓練時間看不出來的事：
    ARS 跑完只要十幾秒，但它吃掉的樣本是 SAC 的好幾倍。
    """
    series = []
    p = RUNS / "rl_ars_log.csv"
    if p.exists():
        d = read_csv("rl_ars_log.csv")
        series.append(("ARS (hand-written, CPU)",
                       [int(r["iteration"]) * ARS_STEPS_PER_ITER for r in d],
                       [float(r["reward_per_step"]) for r in d], "#4a6fa5"))
    p = RUNS / "rl_ppo_log.csv"
    if p.exists():
        d = read_csv("rl_ppo_log.csv")
        series.append(("PPO (SB3, CPU)", [int(r["steps"]) for r in d],
                       [float(r["reward_per_step"]) for r in d], "#5c8a5c"))
    p = RUNS / "rl_sac_gpu_log.csv"
    if p.exists():
        d = read_csv("rl_sac_gpu_log.csv")
        series.append(("SAC (SB3, GPU)", [int(r["steps"]) for r in d],
                       [float(r["reward_per_step"]) for r in d], ACCENT))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.4, 4.4))
    ZOOM = 80_000

    for ax, xmax in ((ax1, None), (ax2, ZOOM)):
        for label, x, y, c in series:
            ax.plot(x, y, color=c, lw=1.7, label=label)
        # SAC 在 CPU 上 60k 步完全沒學起來，一直貼在 -2.0。那是一整段而不是一條
        # 曲線（原始逐點紀錄沒有保留），用陰影帶標出來，不假造資料點。
        ax.axhspan(-2.01, -1.98, color="#d9534f", alpha=0.10)
        ax.set_ylim(-2.12, 0.12)
        ax.set_xlabel("environment steps", fontsize=9.5)
        style(ax)
        if xmax:
            ax.set_xlim(-2000, xmax)

    xr = max(max(x) for _, x, _, _ in series)
    for label, x, y, c in series:
        ax1.plot(x[-1], y[-1], "o", color=c, ms=4.5)
        # 終點靠左的往右標、靠右的往左標，否則標註會掉到圖外
        dx = 9 if x[-1] < xr * 0.55 else -104
        ax1.annotate(f"{y[-1]:.2f}  ({x[-1] / 1000:.0f}k steps)", (x[-1], y[-1]),
                     textcoords="offset points", xytext=(dx, -13), fontsize=8, color=c)

    ax1.set_ylabel("mean reward per step", fontsize=9.5)
    ax1.set_title("Pendulum swing-up: sample efficiency", fontsize=10.5, color=INK, pad=10)
    ax1.legend(frameon=False, fontsize=8.5, loc="center right")
    ax1.text(ax1.get_xlim()[1] * 0.02, -2.09,
             "SAC on CPU never left this band in 60k steps (ch.12)",
             fontsize=8, color="#a33")
    ax2.set_title(f"first {ZOOM // 1000}k steps", fontsize=10.5, color=INK, pad=10)

    fig.tight_layout()
    out = RUNS / "rl_curves.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"{out.relative_to(REPO_ROOT)}  ({len(series)} 條曲線)")


def tilt_boundary():
    d = read_csv("tilt_boundary_log.csv")
    by_case = defaultdict(list)
    for r in d:
        by_case[r["case"]].append(r)

    meta = {
        "木頭棧板": ("wood (mu=0.6)", np.degrees(np.arctan(0.6)), "#8a5a2b"),
        "塑膠棧板": ("plastic (mu=0.35)", np.degrees(np.arctan(0.35)), "#3a6ea5"),
        "塑膠棧板+重心前移10cm": ("plastic, CG +10cm", np.degrees(np.arctan(0.35)), "#7a4fa3"),
    }

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.4, 4.4))

    # 左：兩種量法的滑移對照
    for case, rows in by_case.items():
        label, theory, color = meta.get(case, (case, None, INK))
        x = [float(r["tilt_deg"]) for r in rows]
        ax1.plot(x, [float(r["slip_cm"]) for r in rows], color=color, lw=1.8, label=label)
        ax1.plot(x, [float(r["slip_world_cm"]) for r in rows], color=color, lw=1.0,
                 ls="--", alpha=0.65)
    ax1.axhline(3.0, color=ACCENT, lw=1.0, ls=":")
    ax1.text(24.4, 3.12, "3 cm threshold", fontsize=8.5, color=ACCENT)
    ax1.axvline(np.degrees(np.arctan(0.35)), color="#3a6ea5", lw=0.8, ls=":", alpha=0.6)
    ax1.text(19.6, 0.12, "atan(0.35) = 19.3°", fontsize=8, color="#3a6ea5")
    ax1.text(16.4, 4.05, "solid = fork frame (true slip)\ndashed = world frame",
             fontsize=8.5, color="#555")
    ax1.set_xlabel("actual mast tilt (deg)", fontsize=9.5)
    ax1.set_ylabel("pallet slip (cm)", fontsize=9.5)
    ax1.set_title("Ch.17 Pallet slip: fork frame vs world frame",
                  fontsize=10.5, color=INK, pad=10)
    ax1.legend(frameon=False, fontsize=8.5, loc="upper left")
    style(ax1)

    # 右：目標角 vs 實際角，以及 kp 掃描
    rows = by_case["木頭棧板"]
    t = [float(r["time"]) for r in rows]
    ax2.plot(t, [float(r["target_deg"]) for r in rows], color="#999", lw=1.4,
             ls="--", label="commanded (ctrl)")
    ax2.plot(t, [float(r["tilt_deg"]) for r in rows], color=ACCENT, lw=1.8,
             label="actual (qpos), kp=500")

    kp_path = RUNS / "tilt_kp_sweep.csv"
    if kp_path.exists():
        sweep = read_csv("tilt_kp_sweep.csv")
        for r in sweep:
            if int(r["kp"]) == 500:
                continue
            ax2.plot(t[-1], float(r["final_deg"]), "o", ms=4.5, color="#4a6fa5")
            ax2.annotate(f"kp={r['kp']} → {float(r['final_deg']):.1f}°",
                         (t[-1], float(r["final_deg"])), textcoords="offset points",
                         xytext=(-96, -3), fontsize=8, color="#4a6fa5")
    ax2.set_xlabel("time (s)", fontsize=9.5)
    ax2.set_ylabel("mast tilt (deg)", fontsize=9.5)
    ax2.set_title("Commanded vs actual tilt (20 kg load)", fontsize=10.5, color=INK, pad=10)
    ax2.legend(frameon=False, fontsize=8.5, loc="upper left")
    style(ax2)

    fig.tight_layout()
    out = RUNS / "tilt_boundary.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"{out.relative_to(REPO_ROOT)}  ({len(by_case)} 個案例)")


def cartpole():
    d = read_csv("cartpole_log.csv")
    t = [float(r["time"]) for r in d]
    th = [float(r["theta"]) for r in d]
    x = [float(r["x"]) for r in d]
    u = [float(r["ctrl"]) for r in d]
    mode = [int(r["mode"]) for r in d]
    t_switch = next((ti for ti, m in zip(t, mode) if m), None)

    fig, axes = plt.subplots(3, 1, figsize=(8.4, 6.2), sharex=True)
    for ax, series, lab, color in (
            (axes[0], th, "pole angle (rad)", ACCENT),
            (axes[1], x, "cart position (m)", "#4a6fa5"),
            (axes[2], u, "control force (N)", "#5c8a5c")):
        ax.plot(t, series, color=color, lw=1.4)
        ax.set_ylabel(lab, fontsize=9)
        if t_switch:
            ax.axvline(t_switch, color="#999", lw=0.9, ls="--")
        style(ax)
    axes[0].axhline(0, color="#bbb", lw=0.7)
    if t_switch:
        axes[0].annotate(f"energy shaping → LQR  @ {t_switch:.2f}s",
                         (t_switch, 2.6), textcoords="offset points", xytext=(8, 0),
                         fontsize=8.5, color="#666")
    axes[2].set_xlabel("time (s)", fontsize=9.5)
    axes[0].set_title("Ch.10 Cart-pole swing-up: energy shaping then LQR",
                      fontsize=11, color=INK, pad=12)
    fig.tight_layout()
    out = RUNS / "cartpole_traj.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"{out.relative_to(REPO_ROOT)}  (切換於 {t_switch:.2f}s)")



def pd_control():
    """03 章：PD 控制的響應與穩態誤差。"""
    d = read_csv("pd_control_log.csv")
    t = [float(r["time"]) for r in d]
    th = [float(r["theta"]) for r in d]
    u = [float(r["ctrl"]) for r in d]
    target = np.pi / 2

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.4, 4.0))

    # 控制力矩的初始尖峰有 6.3，和角度共用一條 y 軸的話，1.44 與 1.571 的差距會被
    # 壓成一條線 —— 而那正是這張圖要看的東西。力矩放第二軸。
    ax1.axhline(target, color="#999", lw=1.0, ls="--")
    ax1.text(0.12, target + 0.035, f"target = pi/2 = {target:.3f}", fontsize=8.5, color="#666")
    ax1.plot(t, th, color=ACCENT, lw=2.0, label="theta (rad)")
    ax1.set_ylim(0, 1.85)

    ax1b = ax1.twinx()
    ax1b.plot(t, u, color="#5c8a5c", lw=1.1, alpha=0.8, label="control torque")
    ax1b.set_ylabel("control torque (N·m)", fontsize=9, color="#5c8a5c")
    ax1b.tick_params(labelsize=8.5, colors="#5c8a5c")
    ax1b.spines["top"].set_visible(False)

    ax1.annotate("", xy=(3.6, target), xytext=(3.6, th[-1]),
                 arrowprops=dict(arrowstyle="<->", color="#a33", lw=1.2))
    ax1.text(3.72, (target + th[-1]) / 2 - 0.055,
             f"steady-state error\n{target - th[-1]:.3f} rad ({np.degrees(target - th[-1]):.1f}°)",
             fontsize=8.5, color="#a33")
    ax1.set_xlabel("time (s)", fontsize=9.5)
    ax1.set_ylabel("pendulum angle (rad)", fontsize=9.5)
    ax1.set_title("Ch.03 PD control: response (KP=4, KD=0.8)",
                  fontsize=10.5, color=INK, pad=10)
    ax1.legend(frameon=False, fontsize=8.5, loc="lower right",
               bbox_to_anchor=(1.0, 0.08))
    style(ax1)

    sw = read_csv("pd_kp_sweep.csv")
    kp = [float(r["kp"]) for r in sw]
    err = [float(r["error_deg"]) for r in sw]
    ax2.plot(kp, err, "o-", color="#4a6fa5", lw=1.6, ms=5)
    for x, y in zip(kp, err):
        ax2.annotate(f"{y:.1f}°", (x, y), textcoords="offset points",
                     xytext=(6, 5), fontsize=8, color="#4a6fa5")
    ax2.set_xscale("log")
    ax2.set_xlabel("KP (log scale)", fontsize=9.5)
    ax2.set_ylabel("steady-state error (deg)", fontsize=9.5)
    ax2.set_title("Error shrinks with KP but never reaches zero",
                  fontsize=10.5, color=INK, pad=10)
    ax2.set_ylim(-1, 16)
    style(ax2)

    fig.tight_layout()
    out = RUNS / "pd_control.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"{out.relative_to(REPO_ROOT)}")


def more_examples():
    """07 章：感測器、position 伺服、平行 rollout 各一格。"""
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(13.6, 4.0))

    d = read_csv("sensors_log.csv")
    t = [float(r["time"]) for r in d]
    ax1.plot(t, [float(r["z"]) for r in d], color="#4a6fa5", lw=1.6, label="height z (m)")
    ax1b = ax1.twinx()
    ax1b.plot(t, [float(r["touch_N"]) for r in d], color=ACCENT, lw=1.4, label="touch (N)")
    ax1b.set_ylabel("touch force (N)", fontsize=9, color=ACCENT)
    ax1b.tick_params(labelsize=8.5, colors=ACCENT)
    ax1b.spines["top"].set_visible(False)
    ax1.set_xlabel("time (s)", fontsize=9.5)
    ax1.set_ylabel("height (m)", fontsize=9.5)
    ax1.set_title("Touch sensor: ball drop", fontsize=10.5, color=INK, pad=10)
    ax1.legend(frameon=False, fontsize=8.5, loc="upper right")
    style(ax1)

    d = read_csv("servo_log.csv")
    t = [float(r["time"]) for r in d]
    ax2.plot(t, [float(r["target"]) for r in d], color="#999", lw=1.3, ls="--", label="target")
    ax2.plot(t, [float(r["actual"]) for r in d], color=ACCENT, lw=1.7, label="actual")
    ax2.plot(t, [float(r["error"]) for r in d], color="#a33", lw=1.0, alpha=0.7, label="error")
    ax2.set_xlabel("time (s)", fontsize=9.5)
    ax2.set_ylabel("angle (rad)", fontsize=9.5)
    ax2.set_title("Position servo tracking sin(t)", fontsize=10.5, color=INK, pad=10)
    ax2.legend(frameon=False, fontsize=8.5, loc="upper left")
    style(ax2)

    d = read_csv("parallel_rollout_log.csv")
    runs = defaultdict(list)
    for r in d:
        runs[int(r["run"])].append((float(r["q0"]), float(r["q1"])))
    cmap = plt.get_cmap("tab20")
    for i, (k, pts) in enumerate(sorted(runs.items())):
        c = cmap(i % 20)
        ax3.plot([p[0] for p in pts], [p[1] for p in pts], lw=0.9, color=c, alpha=0.85)
        ax3.plot(pts[-1][0], pts[-1][1], "o", ms=3.5, color=c)
    ax3.set_xlabel("joint 1 angle (rad)", fontsize=9.5)
    ax3.set_ylabel("joint 2 angle (rad)", fontsize=9.5)
    ax3.set_title(f"{len(runs)} parallel rollouts (double pendulum)",
                  fontsize=10.5, color=INK, pad=10)
    style(ax3)

    fig.tight_layout()
    out = RUNS / "more_examples.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"{out.relative_to(REPO_ROOT)}")


def mjx_throughput():
    """28 章：三種做法的吞吐長條圖。"""
    d = read_csv("mjx_throughput.csv")
    # CSV 的 method 是中文（腳本 stdout 直接轉出來的），圖上要換成英文：
    # matplotlib 的預設字型沒有 CJK，直接畫會變成一排豆腐。
    EN = {"CPU 單執行緒": "CPU, single thread",
          "CPU 4 執行緒": "CPU, 4 threads",
          "MJX 批次（cpu）": "MJX batched (CPU backend)",
          "MJX 批次（gpu）": "MJX batched (GPU backend)"}
    names = [EN.get(r["method"], r["method"]) for r in d]
    sps = [int(r["steps_per_sec"]) for r in d]
    rel = [float(r["relative"]) for r in d]
    colors = ["#4a6fa5", "#8a8a8a", ACCENT]

    fig, ax = plt.subplots(figsize=(8.4, 3.4))
    y = np.arange(len(names))
    ax.barh(y, sps, color=colors[:len(names)], height=0.55)
    for i, (v, r) in enumerate(zip(sps, rel)):
        ax.text(v + 30000, i, f"{v:,} steps/s   ({r:.1f}x)", va="center",
                fontsize=9, color=INK)
    ax.set_yticks(y)
    ax.set_yticklabels(names, fontsize=9.5)
    ax.invert_yaxis()
    ax.set_xlim(0, max(sps) * 1.42)
    ax.set_xlabel("simulation steps per second", fontsize=9.5)
    ax.set_title("Ch.28 Throughput: 2048 trajectories x 500 steps (double pendulum)",
                 fontsize=10.5, color=INK, pad=12)
    style(ax)
    ax.grid(axis="y", lw=0)
    fig.tight_layout()
    out = RUNS / "mjx_throughput.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"{out.relative_to(REPO_ROOT)}")


TARGETS = {"pd": pd_control, "ex07": more_examples,
           "mjx": mjx_throughput,
           "rl": rl_curves, "tilt": tilt_boundary, "cartpole": cartpole}

if __name__ == "__main__":
    for name in sys.argv[1:] or list(TARGETS):
        if name not in TARGETS:
            print(f"未知的圖 {name}；可用：{', '.join(TARGETS)}")
            continue
        TARGETS[name]()
