"""範例：多個 mjData 共用同一個 mjModel 做平行取樣（多執行緒 rollout）"""
import csv
import os

import mujoco
import numpy as np
from concurrent.futures import ThreadPoolExecutor

model = mujoco.MjModel.from_xml_path("models/double_pendulum.xml")

def rollout(seed):
    """每條執行緒擁有自己的 mjData — mjModel 唯讀可安全共用。

    順便記下整條軌跡：只看終點看不出雙擺的混沌，看整條才知道相近的初始狀態會怎麼散開。
    """
    data = mujoco.MjData(model)
    rng = np.random.default_rng(seed)
    data.qpos[:] = rng.uniform(-np.pi, np.pi, size=2)
    traj = []
    for _ in range(500):
        mujoco.mj_step(model, data)
        if not traj or data.time - traj[-1][1] >= 0.0199:   # 50 Hz
            traj.append([seed, round(data.time, 4),
                         round(float(data.qpos[0]), 5), round(float(data.qpos[1]), 5)])
    return data.qpos.copy(), traj

with ThreadPoolExecutor(max_workers=8) as pool:
    out = list(pool.map(rollout, range(16)))

results = np.array([o[0] for o in out])
trajs = [row for o in out for row in o[1]]
print(f"完成 {len(results)} 條 rollout，每條 1 秒")
print(f"最終 qpos 前 3 條：\n{np.round(results[:3], 3)}")
print(f"qpos 平均值 = {np.round(results.mean(axis=0), 3)}")

os.makedirs("runs", exist_ok=True)
with open("runs/parallel_rollout_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["run", "time", "q0", "q1"])
    w.writerows(trajs)
print(f"runs/parallel_rollout_log.csv（{len(trajs)} 列）已寫出")
