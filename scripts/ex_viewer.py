"""範例：視覺化的三種方式。

1. 互動式 viewer（需要顯示器）：
     python -m mujoco.viewer --mjcf=models/cartpole_swingup.xml
   或程式內 mujoco.viewer.launch_passive(model, data) — 邊模擬邊看。

2. 離屏渲染（無顯示器 / 伺服器）：mujoco.Renderer 把畫面渲染成 numpy 陣列，
   可存圖、做影片、或當 CNN 的視覺觀測。

本腳本示範第 2 種：渲染車桿自由擺盪的連續影格。
無顯示器環境需設 MUJOCO_GL=osmesa（或 egl，視機器而定）。
"""
import mujoco
import numpy as np
from PIL import Image

model = mujoco.MjModel.from_xml_path("models/cartpole_swingup.xml")
data = mujoco.MjData(model)
data.qpos[1] = np.pi - 0.6   # 桿從偏離垂下 0.6 rad 出發，靠重力自然擺盪
# 註：正好放在 np.pi（垂下）是穩定平衡點，沒有控制輸入時畫面會完全靜止。

renderer = mujoco.Renderer(model, height=480, width=640)

frames = []
for step in range(400):  # 0.8 秒，約半個擺盪週期
    mujoco.mj_step(model, data)
    if step % 100 == 0:
        renderer.update_scene(data)
        frames.append(renderer.render())   # 不帶 out= 時每次回傳新陣列，直接收集即可

for i, f in enumerate(frames):
    Image.fromarray(f).save(f"docs/assets/viewer_frame{i}.png")
    print(f"已輸出 docs/assets/viewer_frame{i}.png  shape={f.shape}")

# --- 互動式 viewer 的寫法（需要有顯示器，這裡僅展示程式結構）---
INTERACTIVE_SNIPPET = """
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path("models/cartpole_swingup.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)     # 你的控制器也可以在這裡寫入 data.ctrl
        viewer.sync()                   # 把最新狀態送到畫面
        time.sleep(model.opt.timestep)  # 即時播放速度
"""
print("\n互動式 viewer 寫法（需要顯示器）：", INTERACTIVE_SNIPPET)
