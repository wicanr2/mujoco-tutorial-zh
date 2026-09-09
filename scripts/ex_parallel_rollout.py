"""範例：多個 mjData 共用同一個 mjModel 做平行取樣（多執行緒 rollout）"""
import mujoco
import numpy as np
from concurrent.futures import ThreadPoolExecutor

model = mujoco.MjModel.from_xml_path("models/double_pendulum.xml")

def rollout(seed):
    """每條執行緒擁有自己的 mjData — mjModel 唯讀可安全共用。"""
    data = mujoco.MjData(model)
    rng = np.random.default_rng(seed)
    data.qpos[:] = rng.uniform(-np.pi, np.pi, size=2)
    for _ in range(500):
        mujoco.mj_step(model, data)
    return data.qpos.copy()

with ThreadPoolExecutor(max_workers=8) as pool:
    results = list(pool.map(rollout, range(16)))

results = np.array(results)
print(f"完成 {len(results)} 條 rollout，每條 1 秒")
print(f"最終 qpos 前 3 條：\n{np.round(results[:3], 3)}")
print(f"qpos 平均值 = {np.round(results.mean(axis=0), 3)}")
