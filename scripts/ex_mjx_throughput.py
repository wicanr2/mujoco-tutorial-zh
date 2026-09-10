"""MJX（GPU）與 CPU 的模擬吞吐對照：同一個模型、同樣的 rollout 數量。

[07 章](../docs/02-programming/02-more-examples.md) 量過 CPU 多執行緒 rollout 受 GIL
限制（實測 0.33×，比單執行緒還慢）。這支腳本補上另一半：把同一批 rollout 交給 MJX
在 GPU 上跑，看吞吐差多少。

MJX 是 MuJoCo 的 JAX 後端，用 `jax.vmap` 把「一個模型 × 一批狀態」變成單次批次運算，
GPU 上數千條軌跡可以同時前進。適合 RL 訓練的資料收集階段。

用法（需要 CUDA 與 jax[cuda]）：
    python scripts/ex_mjx_throughput.py [批次大小] [每條步數]

沒有 GPU 時 JAX 退回 CPU 後端，仍會跑完 —— 那量到的是「XLA 向量化 vs Python
逐步迴圈」，不是「GPU vs CPU」。輸出會標出實際後端（`cpu` / `gpu`），讀數字前先看它。

CPU 執行緒數用 `WORKERS` 調（預設 4）。在共用主機上別把核心佔滿。腳本會記錄量測
當下的 load average：吞吐是牆鐘時間算出來的，鄰居在跑什麼直接影響 CPU 那兩列。
"""
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = REPO_ROOT / "models" / "double_pendulum.xml"
BATCH = int(sys.argv[1]) if len(sys.argv) > 1 else 2048
STEPS = int(sys.argv[2]) if len(sys.argv) > 2 else 500
WORKERS = int(os.environ.get("WORKERS", "4"))


def cpu_serial(model, batch, steps, seed=0):
    """單執行緒逐條跑。"""
    rng = np.random.default_rng(seed)
    data = mujoco.MjData(model)
    t0 = time.perf_counter()
    for i in range(batch):
        mujoco.mj_resetData(model, data)
        data.qpos[:] = rng.uniform(-0.5, 0.5, model.nq)
        for _ in range(steps):
            mujoco.mj_step(model, data)
    return time.perf_counter() - t0


def cpu_threads(model, batch, steps, workers=WORKERS, seed=0):
    """多執行緒。mj_step 會釋放 GIL，但 Python 端的迴圈開銷仍受限。"""
    def one(i):
        rng = np.random.default_rng(seed + i)
        d = mujoco.MjData(model)
        d.qpos[:] = rng.uniform(-0.5, 0.5, model.nq)
        for _ in range(steps):
            mujoco.mj_step(model, d)
        return float(d.qpos[0])

    t0 = time.perf_counter()
    with ThreadPoolExecutor(workers) as ex:
        list(ex.map(one, range(batch)))
    return time.perf_counter() - t0


def mjx_batch(model, batch, steps, seed=0):
    """MJX：把 batch 條軌跡 vmap 成一次批次運算。"""
    import jax
    import jax.numpy as jp
    from mujoco import mjx

    mx = mjx.put_model(model)
    key = jax.random.PRNGKey(seed)
    qpos0 = jax.random.uniform(key, (batch, model.nq), minval=-0.5, maxval=0.5)

    @jax.jit
    def rollout(qpos):
        dx = mjx.make_data(mx)
        dx = dx.replace(qpos=qpos)

        def one_step(d, _):
            return mjx.step(mx, d), None

        dx, _ = jax.lax.scan(one_step, dx, None, length=steps)
        return dx.qpos

    batched = jax.jit(jax.vmap(rollout))
    out = batched(qpos0)          # 第一次含 JIT 編譯
    out.block_until_ready()

    t0 = time.perf_counter()      # 第二次才是穩態吞吐
    out = batched(qpos0)
    out.block_until_ready()
    return time.perf_counter() - t0, jax.devices()[0].platform


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    total_steps = BATCH * STEPS
    print(f"模型 {MODEL_PATH.name}：nq={model.nq} nv={model.nv}")
    print(f"批次 {BATCH} 條 × 每條 {STEPS} 步 = {total_steps:,} 個 mj_step")
    print(f"CPU {os.cpu_count()} 核，本次用 {WORKERS} 執行緒；"
          f"開跑前 load average = {', '.join(f'{x:.2f}' for x in os.getloadavg())}\n")

    results = []

    # 三種做法都跑完整批次。這個模型小（nq=2），100 萬步 CPU 單執行緒也只要幾秒 ——
    # 能直接量就不要抽樣外推，外推等於在結論裡摻進一個沒必要的誤差來源。
    dt = cpu_serial(model, BATCH, STEPS)
    results.append(("CPU 單執行緒", dt, total_steps / dt))
    print(f"CPU 單執行緒：{dt:.2f}s")

    dt_t = cpu_threads(model, BATCH, STEPS)
    results.append((f"CPU {WORKERS} 執行緒", dt_t, total_steps / dt_t))
    print(f"CPU {WORKERS} 執行緒：{dt_t:.2f}s")

    try:
        dt_gpu, platform = mjx_batch(model, BATCH, STEPS)
        results.append((f"MJX 批次（{platform}）", dt_gpu, total_steps / dt_gpu))
        print(f"MJX（{platform}）：{dt_gpu:.2f}s")
    except ImportError as e:
        print(f"MJX 未安裝（pip install mujoco-mjx jax[cuda13]）：{e}")
    except Exception as e:
        # GPU 後端初始化失敗不是 ImportError —— jax 會在 jax.devices() 丟
        # RuntimeError。只接 ImportError 的話腳本會整支崩掉，而不是少一列對照。
        first = str(e).strip().splitlines()[0][:160]
        print(f"MJX 批次跳過：JAX 後端起不來 —— {first}")
        print("  這台不支援時可用 JAX_PLATFORMS=cpu 降級到 CPU 後端，"
              "量到的是 XLA 向量化的效益（見 28 章）。")

    print("\n吞吐對照")
    print(f"  {'做法':28s} {'耗時':>10s} {'步數/秒':>14s} {'相對':>8s}")
    base = results[0][2]
    for name, secs, sps in results:
        print(f"  {name:28s} {secs:9.1f}s {sps:13,.0f} {sps / base:7.1f}×")
    print(f"\n量測結束 load average = {', '.join(f'{x:.2f}' for x in os.getloadavg())}")


if __name__ == "__main__":
    main()
