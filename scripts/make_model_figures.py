"""從模型檔直接模擬並渲染教學圖條（01 / 02 / 04 章的模型長什麼樣、怎麼動）。

輸出：docs/assets/strip_<名稱>.png

與 make_video_strips.py 的差別：那支從 runs/ 的錄影抽幀，這支沒有錄影可抽，直接
載入模型跑一段再渲染。與 make_strips.py 的差別：那支是 16 / 20 章的特定搬運流程。

無顯示器環境要設 MUJOCO_GL=osmesa：

    MUJOCO_GL=osmesa python scripts/make_model_figures.py
"""
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "docs" / "assets"
W, H = 420, 340

# name: dict(model, times, labels, cam, qpos=初始關節角)
# 初始狀態要跟對應章節的腳本一致，否則圖上演的是另一個實驗。
FIGURES = {
    "hello_ball": dict(
        model="models/hello.xml", times=[0.0, 0.3, 0.62, 1.4],
        labels=["起始 z = 2.0", "自由落下", "觸地反彈", "靜止在地面"],
        cam=dict(azimuth=135, elevation=-10, distance=4.4, lookat=[0, 0, 0.9])),
    "double_pendulum": dict(
        model="models/double_pendulum.xml", times=[0.0, 0.35, 0.75, 1.6],
        labels=["初始 qpos = [1.0, 1.0]", "第一次擺盪", "兩節脫序", "1.6 秒後"],
        qpos=[1.0, 1.0],          # 與 scripts/run_pendulum.py 相同
        cam=dict(azimuth=90, elevation=-6, distance=2.2, lookat=[0, 0, 0.7])),
    "two_link_arm": dict(
        model="models/two_link_arm.urdf", times=[0.0, 0.3, 0.7, 1.5],
        labels=["URDF 載入後（qpos = [1.0, 0.5]）", "重力下擺動", "回擺", "1.5 秒後"],
        qpos=[1.0, 0.5],          # 與 scripts/load_urdf.py 相同
        groups=[1],               # 只顯示 visual（group 1）；collision 是 group 0
        # 這個模型往下掛：stat.center 是 [0, 0, -0.4]，extent 0.951。
        # lookat 照抄 stat.center，distance 取 extent 的 1.8 倍左右。
        cam=dict(azimuth=100, elevation=-10, distance=1.55, lookat=[0, 0, -0.28])),
    "two_link_arm_groups": dict(
        model="models/two_link_arm.urdf", times=[0.0, 0.0],
        labels=["visual（group 1）", "collision（group 0）"],
        qpos=[1.0, 0.5],
        per_tile_groups=[[1], [0]],      # 同一個姿態，左右分別只顯示一組
        cam=dict(azimuth=100, elevation=-10, distance=1.55, lookat=[0, 0, -0.28])),
}

FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
]


def load_font(size):
    for f in FONT_CANDIDATES:
        if Path(f).exists():
            try:
                return ImageFont.truetype(f, size)
            except OSError:
                continue
    return ImageFont.load_default()


def annotate(arr, seconds, label):
    im = Image.fromarray(arr)
    d = ImageDraw.Draw(im, "RGBA")
    text = f"{seconds:.2f}s" + (f"　{label}" if label else "")
    font = load_font(17)
    box = d.textbbox((0, 0), text, font=font)
    d.rectangle((0, 0, box[2] + 20, box[3] + 14), fill=(0, 0, 0, 165))
    d.text((10, 5), text, font=font, fill=(255, 255, 255, 255))
    return im


def build(name, spec):
    times, labels = spec["times"], spec["labels"]
    model = mujoco.MjModel.from_xml_path(str(REPO_ROOT / spec["model"]))
    data = mujoco.MjData(model)
    if spec.get("qpos"):
        data.qpos[:len(spec["qpos"])] = spec["qpos"]
    # 一定要算一次：剛建立的 mjData 裡 xpos / xmat 都還是 0，直接渲染會得到全黑的
    # 一格（而且只有 t=0 那格黑，後面的因為 mj_step 算過就正常 —— 很容易誤判成
    # 「第一次渲染的已知問題」）。
    mujoco.mj_forward(model, data)
    # 這幾個教學模型只有一盞燈、沒有地板，預設渲染出來幾乎全黑。調亮 headlight
    # 是渲染端的設定，不動模型檔 —— 模型保持最小、能跑就好，那是章節的重點。
    model.vis.headlight.ambient[:] = 0.45
    model.vis.headlight.diffuse[:] = 0.85
    model.vis.headlight.specular[:] = 0.2

    renderer = mujoco.Renderer(model, H, W)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)     # 先取模型自帶的合理視角
    for k, v in spec["cam"].items():             # 再依模型微調
        setattr(cam, k, v)

    opt = mujoco.MjvOption()

    def set_groups(gs):
        """只顯示指定的 geom group。

        URDF 匯入後 visual 是 group 1、collision 是 group 0，兩者會疊在一起 ——
        不指定就會看到碰撞幾何蓋在外觀上。
        """
        for g in range(len(opt.geomgroup)):
            opt.geomgroup[g] = 1 if (gs is None or g in gs) else 0

    set_groups(spec.get("groups"))

    tiles = []
    for i, t in enumerate(times):
        while data.time < t:
            mujoco.mj_step(model, data)
        if spec.get("per_tile_groups"):
            set_groups(spec["per_tile_groups"][i])
        renderer.update_scene(data, camera=cam, scene_option=opt)
        tiles.append(annotate(renderer.render().copy(), data.time,
                              labels[len(tiles)] if len(tiles) < len(labels) else ""))
    sheet = Image.new("RGB", (W * len(tiles), H), (255, 255, 255))
    for i, tile in enumerate(tiles):
        sheet.paste(tile, (i * W, 0))
    out = OUT_DIR / f"strip_{name}.png"
    sheet.save(out, optimize=True)
    print(f"{out.relative_to(REPO_ROOT)}  {sheet.size[0]}×{sheet.size[1]}  "
          f"{out.stat().st_size / 1024:.0f} KB")


if __name__ == "__main__":
    for n in sys.argv[1:] or list(FIGURES):
        if n not in FIGURES:
            print(f"未知的名稱 {n}；可用：{', '.join(FIGURES)}")
            continue
        build(n, FIGURES[n])
