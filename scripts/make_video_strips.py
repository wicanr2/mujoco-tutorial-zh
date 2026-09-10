"""從 runs/ 的錄影抽幀拼成圖條，讓讀者不用下載影片也能看出實驗過程。

輸出：docs/assets/strip_<實驗名>.png

用法：
    python scripts/make_video_strips.py                    # 全部重做
    python scripts/make_video_strips.py reach_xz           # 只做指定的
    python scripts/make_video_strips.py --suggest reach_xz # 印出候選時間點（新增實驗時用）

抽幀時間點不是等距取樣，而是**從對應的 CSV log 找出動作的轉折點**：控制命令改變的
時刻（`step` 模式），或升降高度的顯著極值（`extrema` 模式）。等距抽樣看起來均勻，
卻會抽到一堆「東西在移動」的中間畫面，讀者要看的是動作換段的地方。

轉折點抓到的是「命令發出」的瞬間，那時畫面還沒動。所以實際取樣往後延 DELAY 秒，
看到的才是命令的效果 —— 但不會延過下一個轉折點。
"""
import sys
from pathlib import Path

import imageio.v2 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "docs" / "assets"
TILE_W = 480          # 每格縮到這個寬度
DELAY = 1.2           # 轉折點之後幾秒取樣（看效果，不是看命令）
TARGET_MED = 70       # 提亮後的目標亮度中位數（0–255）

# 各支錄影的曝光差很多：貨架那幾支的亮度中位數只有 17（縮圖裡幾乎全黑），
# loop 因為相機貼得近、地面反光，中位數本來就高。所以 gamma 不能寫死一個值 ——
# 對亮的那支套暗片用的 gamma 會直接過曝成一片白。改成依實際亮度算。

# 每支影片的取樣時間點與標籤。時間點是挑過的（用 --suggest 找候選，看圖確認後寫死），
# 不是每次自動算 —— 自動選點會隨演算法微調而漂移，標籤卻是照舊選點寫的，結果是每格
# 說明都對不上畫面而且不會報錯。改了實驗腳本要回來重挑。
# labels 依序對應挑出來的時間點；數量對不上時退回只標時間（並印警告）
STRIPS = {
    "mission": dict(
        times=[0.5, 4.0, 6.5, 10.0, 12.5, 16.5],
        labels=["初始：棧板在地面", "牙叉插入棧板", "抬起",
                "載貨移動", "y-reach 側移對位", "放回地面"]),
    "fork_cyclic": dict(
        times=[0.1, 4.9, 10.4, 14.9, 20.3, 23.9, 25.0, 25.9],
        labels=["初始：棧板在地面", "取貨後升起", "升到頂", "降到底",
                "側移 +0.42 m", "側移 −0.42 m", "回到中線", "放回地面"]),
    "reach_xz": dict(
        times=[1.5, 5.4, 7.4, 11.0, 13.0, 17.5, 21.7, 28.5],
        labels=["開向貨架（棧板在層板上）", "牙叉升到層板下方", "stage 深插進棧板",
                "微升＋後傾，貨離開層板", "保持低位退出貨架", "離架後升到搬運高度",
                "載貨行進", "放到地面"]),
    "reach_xyz": dict(
        times=[1.5, 5.4, 7.4, 11.0, 13.0, 17.5, 26.5, 33.0],
        labels=["開向貨架（棧板在層板上）", "牙叉升到層板下方", "stage 深插進棧板",
                "微升＋後傾，貨離開層板", "保持低位退出貨架", "離架後升到搬運高度",
                "y-reach 側移 −0.40 m 對位", "放到地面、reach 收回"]),
    "gripper": dict(
        times=[0.5, 5.0, 8.0, 12.0, 16.5, 20.5],
        labels=["初始：箱子在 A 桌", "手臂伸向箱子", "夾起舉高",
                "繞行到 B 桌", "對準放置點", "放下、鬆開"]),
    "loop": dict(
        # wp_i 是「目前追的目標索引」，遞增代表剛通過上一個 waypoint
        times=[4.2, 16.5, 26.6, 36.9, 47.2, 57.5],
        labels=["過 (2, 0)", "過 (2, 2)", "過 (0, 2)",
                "回到 (0, 0)", "第二圈 (2, 0)", "第二圈 (2, 2)"]),
}

# --suggest 用的偵測設定：新增實驗時先跑 --suggest 看候選點，挑好再寫進上面的 times
SUGGEST = {
    "mission": dict(mode="extrema", cols=["pallet_z"], motion="base_x", cells=6),
    "fork_cyclic": dict(mode="extrema", cols=["lift1", "reach_y"], cells=8),
    "reach_xz": dict(mode="step", cols=["stage", "lift1", "lift2", "tilt"],
                     motion="bx", cells=8),
    "reach_xyz": dict(mode="step", cols=["stage", "lift1", "lift2", "tilt", "reach"],
                      motion="bx", cells=8),
    "gripper": dict(mode="extrema", cols=["pz"], motion="bx", cells=6),
    "loop": dict(mode="step", cols=["wp_i"], cells=6),
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


def read_log(name):
    return np.genfromtxt(REPO_ROOT / "runs" / f"{name}_log.csv",
                         delimiter=",", names=True)


def step_changes(d, cols, tol=1e-9):
    """控制命令改變的時刻 = 動作換段。"""
    sig = np.column_stack([d[c] for c in cols])
    chg = np.any(np.abs(np.diff(sig, axis=0)) > tol, axis=1)
    return d["time"][1:][chg]


def extrema(d, cols, min_gap=1.0, rel=0.25):
    """顯著的局部極值。小抖動也會讓一階差分變號，不濾會抽到一堆一樣的畫面。"""
    out = []
    for col in cols:
        v, t = d[col], d["time"]
        span = float(v.max() - v.min())
        if span <= 0:
            continue
        last = None
        for i in range(1, len(v) - 1):
            if (v[i] - v[i - 1]) * (v[i + 1] - v[i]) >= 0:
                continue
            if last is not None and abs(v[i] - last) < rel * span:
                continue
            out.append(float(t[i])); last = v[i]
    return np.array(sorted(out))


def dedupe(times, gap=0.8):
    out = []
    for t in sorted(times):
        if not out or t - out[-1] > gap:
            out.append(t)
    return out


def pick(d, cols, times, total, cells, motion=None):
    """從候選轉折點挑出 cells 個彼此最不像的。

    不要用均勻索引取樣：升降、側移這種週期動作的極值是交替出現的，等間隔挑會一直
    挑到同一個相位（實測 fork_cyclic 的 8.9 / 11.9 / 14.9 全是「降到底」，三格一模
    一樣）。改成在訊號值的空間做最遠點取樣 —— 先固定頭尾，其餘每次加入「離已選集合
    最遠」的那個，畫面就會盡量互不重複。
    """
    times = dedupe(times)
    if len(times) < cells:
        times = dedupe(list(times) + list(np.linspace(0, total, cells + 2)[1:-1]))
    if len(times) <= cells:
        return times

    def feat(t):
        i = int(np.argmin(np.abs(d["time"] - t)))
        f = []
        for c in list(cols) + ([motion] if motion else []):
            v = d[c]
            span = float(v.max() - v.min()) or 1.0
            f.append(float((v[i] - v.min()) / span))
        f.append(0.35 * t / total)      # 讓時間也稍微分散，避免全擠在同一段
        return np.array(f)

    feats = [feat(t) for t in times]
    chosen = [0, len(times) - 1]
    while len(chosen) < cells:
        best, best_d = None, -1.0
        for i in range(len(times)):
            if i in chosen:
                continue
            dist = min(float(np.linalg.norm(feats[i] - feats[j])) for j in chosen)
            if dist > best_d:
                best, best_d = i, dist
        chosen.append(best)
    return [times[i] for i in sorted(chosen)]


def with_delay(times, total):
    """往後挪 DELAY 秒看效果，但不越過下一個轉折點，也不超出影片長度。"""
    out = []
    for i, t in enumerate(times):
        nxt = times[i + 1] if i + 1 < len(times) else total
        out.append(min(t + DELAY, nxt - 0.15, total - 0.05))
    return out


def top_crop(frames, dark=34, ratio=0.93):
    """算出頂部有多少列是背景（相機視角上方的空白）。

    不能用「該列最大亮度」判斷 —— 門架柱子會穿過整個背景區，一根亮柱子就讓整列
    的最大值變高。改看「該列有多少比例的像素是暗的」，超過 ratio 才算背景列。
    取所有影格的最小值，只裁掉每一格都是背景的部分。
    """
    rows = []
    for f in frames:
        dark_frac = (f.max(axis=2) < dark).mean(axis=1)
        i = 0
        while i < len(dark_frac) and dark_frac[i] > ratio:
            i += 1
        rows.append(i)
    return min(rows) if rows else 0


def auto_gamma(frames):
    """讓亮度中位數落在 TARGET_MED，夾在 [0.45, 1.0] 之間（只提亮，不壓暗）。"""
    med = float(np.median(np.concatenate([f.reshape(-1, 3).max(axis=1) for f in frames])))
    med = min(max(med, 1.0), 254.0)
    g = np.log(TARGET_MED / 255.0) / np.log(med / 255.0)
    return float(min(max(g, 0.45), 1.0))


def annotate(img, seconds, label, crop_top, gamma):
    im = Image.fromarray(img[crop_top:])
    im = im.resize((TILE_W, round(im.height * TILE_W / im.width)), Image.LANCZOS)
    if gamma < 0.995:
        lut = [round(255 * (i / 255) ** gamma) for i in range(256)] * 3
        im = im.point(lut)
    d = ImageDraw.Draw(im, "RGBA")
    text = f"{seconds:.1f}s" + (f"　{label}" if label else "")
    # 字要夠大：這張圖在網頁上會被縮到約 800px 寬，8 格的話每格只剩 200px。
    font = load_font(26)
    box = d.textbbox((0, 0), text, font=font)
    d.rectangle((0, 0, box[2] + 26, box[3] + 20), fill=(0, 0, 0, 175))
    d.text((13, 8), text, font=font, fill=(255, 255, 255, 255))
    return im


def build(name, spec):
    video = REPO_ROOT / "runs" / f"{name}.mp4"
    if not video.exists():
        print(f"跳過 {name}：{video} 不存在")
        return
    d = read_log(name)
    total = float(d["time"][-1])
    times = [min(t, total - 0.05) for t in spec["times"]]
    render(name, video, d, times, spec.get("labels") or [])


def suggest(name):
    """印出自動偵測的候選時間點，供新增實驗時挑選。

    為什麼定稿要用寫死的 times：自動挑點會隨演算法微調而漂移（實測改了特徵權重之後
    三支影片的選點全變了），而標籤是照著舊選點寫的 —— 於是圖上每一格的說明都對不上
    畫面，還不會有任何錯誤訊息。挑點自動化、定稿手動化，是這裡的分工。
    """
    sp = SUGGEST.get(name)
    if not sp:
        print(f"{name}: 沒有偵測設定"); return
    d = read_log(name)
    total = float(d["time"][-1])
    raw = (step_changes(d, sp["cols"]) if sp["mode"] == "step" else extrema(d, sp["cols"]))
    raw = np.append(np.asarray(raw, dtype=float), 0.3)
    raw = np.unique(np.concatenate([raw, (np.sort(raw)[:-1] + np.sort(raw)[1:]) / 2]))
    times = pick(d, sp["cols"], list(map(float, raw)), total, sp["cells"],
                 motion=sp.get("motion"))
    if sp["mode"] == "step":
        times = with_delay(times, total)
    print(f"{name}: 候選 {[round(float(t), 1) for t in times]}")
    for t in times:
        i = int(np.argmin(np.abs(d["time"] - t)))
        vals = "  ".join(f"{c}={d[c][i]:+.3f}"
                         for c in list(sp["cols"]) + ([sp["motion"]] if sp.get("motion") else []))
        print(f"    t={t:6.1f}  {vals}")


def render(name, video, d, times, labels):
    if len(labels) != len(times):
        print(f"  ! {name}: 有 {len(times)} 格但 {len(labels)} 個標籤，這次只標時間")
        labels = [""] * len(times)

    reader = iio.get_reader(str(video))
    fps = reader.get_meta_data()["fps"]
    n = reader.count_frames()
    frames = [reader.get_data(min(int(round(t * fps)), n - 1)) for t in times]
    reader.close()

    crop = top_crop(frames)
    gamma = auto_gamma(frames)
    tiles = [annotate(f, t, lab, crop, gamma) for f, t, lab in zip(frames, times, labels)]
    cols = 4 if len(tiles) > 6 else 3
    tw, th = tiles[0].size
    rows = (len(tiles) + cols - 1) // cols
    sheet = Image.new("RGB", (tw * cols, th * rows), (255, 255, 255))
    for i, tile in enumerate(tiles):
        sheet.paste(tile, ((i % cols) * tw, (i // cols) * th))
    out = OUT_DIR / f"strip_{name}.png"
    sheet.save(out, optimize=True)
    print(f"{out.relative_to(REPO_ROOT)}  {sheet.size[0]}×{sheet.size[1]}  "
          f"{out.stat().st_size / 1024:.0f} KB  裁頂 {crop}px  γ={gamma:.2f}  "
          f"時間點 {[round(float(t), 1) for t in times]}")


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--suggest":
        for n in args[1:] or list(SUGGEST):
            suggest(n)
        sys.exit(0)
    for n in args or list(STRIPS):
        if n not in STRIPS:
            print(f"未知的實驗名 {n}；可用：{', '.join(STRIPS)}")
            continue
        build(n, STRIPS[n])
