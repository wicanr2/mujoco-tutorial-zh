"""產生實驗用的多幀圖條（filmstrip）：取貨流程 + 前傾卸貨流程。

輸出：docs/assets/strip_pickup.png、docs/assets/strip_tilt.png
"""
import mujoco
import numpy as np
from PIL import Image

xml = (open("models/mr1533_pallet_template.xml").read()
       .replace("PALLET_OBJ", "pallet_wood.stl")
       .replace("PALLET_FRICTION", "0.6")
       .replace("PALLET_RGBA", "0.45 0.30 0.15 1")
       .replace('<light pos="2 -3 4" dir="-0.4 0.6 -1"/>',
                '<light pos="2 -3 4" dir="-0.4 0.6 -1"/><light pos="-3 1 3" dir="0.6 -0.2 -0.6"/>'))
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model, 360, 480)
opt = mujoco.MjvOption()
opt.geomgroup[3] = 0


def shot(az=300, el=-18, dist=3.2, lookat=(-0.5, 0, 0.45)):
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = az, el, dist
    cam.lookat = list(lookat)
    renderer.update_scene(data, camera=cam, scene_option=opt)
    return renderer.render()


def strip(frames, path):
    Image.fromarray(np.concatenate(frames, axis=1)).save(path)
    print("saved", path)


def run(ctrl, seconds, capture_at=None, shots=None):
    n = int(seconds / model.opt.timestep)
    for i in range(n):
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
        if capture_at and shots is not None and i in capture_at:
            shots.append(shot())


# ===== 取貨流程圖條：接近 → 插入 → 抬起 → 停穩 =====
run([0, 0, 0, 0, 0, 0], 0.25)                 # 先跑一小段再截圖（t=0 的渲染偶發全黑）
s = [shot()]                                  # 初始狀態
run([-0.5, 0, 0, 0, 0, 0], 1.5)
s.append(shot())                              # 插入
run([0, 0, 0, 0.35, 0.2, 0], 1.0)
s.append(shot())                              # 抬到一半
run([0, 0, 0, 0.35, 0.2, 0], 1.0)
s.append(shot())                              # 抬起完成
strip(s, "docs/assets/strip_pickup.png")

# ===== 前傾卸貨流程：抬起狀態 → 前傾 8° → 12° → 著地 =====
s = [shot()]
run([0, 0, 0, 0.35, 0.2, -0.14], 1.5)
s.append(shot())
run([0, 0, 0, 0.35, 0.2, -0.24], 1.5)
s.append(shot())
run([0, 0, 0, 0.35, 0.2, -0.30], 1.5)
s.append(shot())
strip(s, "docs/assets/strip_tilt.png")

pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet")
print("最終棧板 z =", round(data.xpos[pid][2], 3))
