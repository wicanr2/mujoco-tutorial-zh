#!/usr/bin/env python3
"""靜態稽核：把三輪人工稽核抓到的問題變成每次都會跑的檢查。

用法（在 repo 根目錄）：
    python scripts/check_docs.py

檢查項目與它們對應的實際事故，見各 check 函式的 docstring。
全部通過回傳 0，任何一項失敗回傳 1 並列出細節。
"""
import re
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
SKIP_DIRS = (".venv", "_site", "workspace", ".git", "__pycache__")


def markdown_files():
    return [p for p in ROOT.glob("**/*.md")
            if not any(d in p.relative_to(ROOT).parts for d in SKIP_DIRS)]


def chapter_files():
    """docs/ 底下的教學章節（不含索引與術語表）。"""
    return sorted(p for p in (ROOT / "docs").glob("**/*.md")
                  if p.name not in ("README.md", "glossary.md"))


def check_absolute_paths():
    """模型與腳本裡不准有絕對路徑。

    事故：models/mr1533_steer.xml 的 meshdir 與兩支腳本寫死 /home/anr2/...，
    在作者機器上永遠正常，別人 clone 下來 24/25/27 章直接失敗。
    """
    bad = []
    pat = re.compile(r'(?:"|\')(?:/home/|/Users/|[A-Za-z]:\\\\)')
    for p in list((ROOT / "scripts").glob("*.py")) + list((ROOT / "models").glob("*.xml")):
        for i, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            if pat.search(line):
                bad.append(f"{p.relative_to(ROOT)}:{i}: {line.strip()[:80]}")
    return bad


def check_relative_links():
    """markdown 的相對連結都要指到存在的檔案。"""
    bad = []
    for md in markdown_files():
        for m in re.finditer(r"\[([^\]]*)\]\(([^)]+)\)", md.read_text(encoding="utf-8")):
            link = m.group(2).split("#")[0].strip()
            if not link or link.startswith(("http://", "https://", "mailto:", "~")):
                continue
            if not (md.parent / link).resolve().exists():
                bad.append(f"{md.relative_to(ROOT)}: [{m.group(1)}]({link})")
    return bad


def check_models_load():
    """models/ 的每個 MJCF 都要能載入（template 先替換佔位符）。

    兩種 meshdir 基準都要照顧：一般模型的 meshdir 相對 XML 位置，用 from_xml_path；
    `_template` 是給腳本 from_xml_string 用的，meshdir 相對 repo 根。
    """
    import os
    import mujoco
    bad = []
    subs = {"PALLET_FRICTION": "0.6", "PALLET_RGBA": "0.5 0.3 0.2 1",
            "CG_OFFSET": "0", "PALLET_OBJ": "pallet_wood.stl"}
    cwd = os.getcwd()
    os.chdir(ROOT)                       # template 的 meshdir 以 repo 根為基準
    try:
        for p in sorted((ROOT / "models").glob("*.xml")):
            raw = p.read_text(encoding="utf-8")
            xml = raw
            for k, v in subs.items():
                xml = xml.replace(k, v)
            try:
                if xml == raw:
                    mujoco.MjModel.from_xml_path(str(p))
                else:
                    mujoco.MjModel.from_xml_string(xml)
            except Exception as e:
                bad.append(f"{p.relative_to(ROOT)}: {str(e).splitlines()[0][:90]}")
    finally:
        os.chdir(cwd)
    return bad


def check_doc_xml_blocks():
    """文件裡的完整 MJCF 區塊要能載入（讀者會照抄）。"""
    import mujoco
    bad = []
    for md in markdown_files():
        for block in re.findall(r"```xml\n(.*?)```", md.read_text(encoding="utf-8"), re.S):
            b = block.strip()
            if not b.startswith("<mujoco") or "..." in b or "…" in b:
                continue
            try:
                mujoco.MjModel.from_xml_string(b)
            except Exception as e:
                bad.append(f"{md.relative_to(ROOT)}: {str(e).splitlines()[0][:90]}")
    return bad


def check_chapter_structure():
    """每章都要有 AGENTS.md 要求的元素。"""
    bad = []
    for md in chapter_files():
        t = md.read_text(encoding="utf-8")
        miss = [s for s in ("## 學習目標", "## 前置知識", "## 延伸閱讀") if s not in t]
        if not ("除錯" in t or "常見錯誤" in t):
            miss.append("除錯／常見錯誤")
        if "```python" not in t and "scripts/" not in t:
            miss.append("可執行範例")
        if miss:
            bad.append(f"{md.relative_to(ROOT)}: 缺 {', '.join(miss)}")
    return bad


def check_chapter_numbering():
    """章節編號要連續、不重複，前置知識不能指向後面的章節。"""
    bad = []
    nums = {}
    for md in chapter_files():
        m = re.match(r"# (\d+)", md.read_text(encoding="utf-8"))
        if not m:
            bad.append(f"{md.relative_to(ROOT)}: 標題沒有章節編號")
            continue
        n = int(m.group(1))
        if n in nums:
            bad.append(f"第 {n} 章重複：{nums[n]} 與 {md.relative_to(ROOT)}")
        nums[n] = md.relative_to(ROOT)
    if nums:
        missing = [n for n in range(1, max(nums) + 1) if n not in nums]
        if missing:
            bad.append(f"章節編號有缺口：{missing}")
    for md in chapter_files():
        t = md.read_text(encoding="utf-8")
        m0 = re.match(r"# (\d+)", t)
        m = re.search(r"## 前置知識\n(.*?)\n##", t, re.S)
        if not (m0 and m):
            continue
        cur = int(m0.group(1))
        for ref in re.findall(r"\[(\d+)[｜|]", m.group(1)):
            if int(ref) >= cur:
                bad.append(f"{md.relative_to(ROOT)}: 前置知識指向第 {ref} 章（不早於自己）")
    return bad


def check_inventories():
    """各目錄的 README 要登記該目錄的所有檔案。"""
    bad = []
    pairs = [
        ("scripts/README.md", sorted(p.name for p in (ROOT / "scripts").glob("*.py"))),
        ("models/README.md", sorted(p.name for p in (ROOT / "models").glob("*.xml"))),
        ("runs/README.md", sorted(p.name for p in (ROOT / "runs").glob("*")
                                  if p.suffix in (".mp4", ".csv", ".png"))),
        ("policies/README.md", sorted(p.name for p in (ROOT / "policies").glob("*")
                                      if p.suffix in (".npy", ".zip"))),
        ("docs/README.md", sorted(str(p.relative_to(ROOT / "docs")) for p in chapter_files())),
    ]
    for readme, names in pairs:
        text = (ROOT / readme).read_text(encoding="utf-8")
        for n in names:
            if n not in text:
                bad.append(f"{readme} 沒有登記 {n}")
    return bad


def check_orphan_assets():
    """docs/assets 的圖片都要被引用。"""
    alltext = "\n".join(p.read_text(encoding="utf-8") for p in markdown_files())
    return [f"未被任何文件引用：docs/assets/{p.name}"
            for p in sorted((ROOT / "docs" / "assets").glob("*.png"))
            if p.name not in alltext]


def check_requirements():
    """文件裡 pip install 的套件都要在 requirements.txt。"""
    req = (ROOT / "requirements.txt").read_text(encoding="utf-8").lower().replace("-", "_")
    bad = []
    for md in markdown_files():
        for m in re.finditer(r"pip install ([a-z0-9\-_ \[\]]+)", md.read_text(encoding="utf-8").lower()):
            for pkg in m.group(1).split():
                pkg = pkg.strip()
                if pkg in ("-r", "requirements.txt", "requirements") or not pkg:
                    continue
                if pkg.replace("-", "_") not in req:
                    bad.append(f"{md.relative_to(ROOT)}: 提到 {pkg} 但不在 requirements.txt")
    return sorted(set(bad))


def check_model_penetration():
    """模型載入後不該有不收斂的接觸穿透。

    事故：mr1533_pallet_template 的叉齒碰撞盒沒對準棧板叉孔，叉車變成「撞著棧板推」
    而不是「插進去抬」。棧板照樣被舉起來、腳本照樣印驗證通過，只有量接觸才看得到
    全程 15 mm 的持續穿透。靜置後仍收不回去的穿透，就是幾何沒對好。
    """
    import mujoco

    subs = {"PALLET_FRICTION": "0.6", "PALLET_RGBA": "0.5 0.4 0.2 1",
            "CG_OFFSET": "0", "PALLET_OBJ": "pallet_wood.stl"}
    bad = []
    for p in sorted(list((ROOT / "models").glob("*.xml"))
                    + list((ROOT / "models").glob("*.urdf"))):
        text = p.read_text(encoding="utf-8")
        if any(k in text for k in subs):
            for k, v in subs.items():
                text = text.replace(k, v)
            text = text.replace('meshdir="meshes/', f'meshdir="{ROOT}/models/meshes/')
            text = text.replace('file="../', f'file="{ROOT}/models/meshes/')
            m = mujoco.MjModel.from_xml_string(text)
        else:
            m = mujoco.MjModel.from_xml_path(str(p))
        d = mujoco.MjData(m)
        # 靜置 1 秒讓初始重疊沉降掉，還剩下的才是真的沒對好
        for _ in range(int(1.0 / m.opt.timestep)):
            mujoco.mj_step(m, d)
        worst = max((-d.contact[i].dist for i in range(d.ncon)), default=0.0)
        if worst > 0.005:          # 5 mm
            names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1])
                     + " ↔ " + mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2])
                     for c in (d.contact[i] for i in range(d.ncon)) if -c.dist > 0.005}
            bad.append(f"{p.relative_to(ROOT)}: 靜置 1 秒後仍有 {worst*1000:.1f} mm 穿透"
                       f"（{', '.join(sorted(names))}）")
    return bad


def check_version_claims():
    """文件裡寫的套件版本要與 requirements.txt 一致。

    升版時最容易漏掉的就是散在各章開頭的「測試環境」那幾行 —— 沒人會記得它們在哪。
    """
    req = {}
    for line in (ROOT / "requirements.txt").read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line.startswith("#") or "==" not in line:
            continue
        name, ver = line.split("==", 1)
        req[name.strip().lower()] = ver.strip()

    # 文件裡的寫法 → requirements.txt 裡的套件名
    alias = {
        "mujoco": "mujoco", "numpy": "numpy", "scipy": "scipy",
        "imageio": "imageio", "matplotlib": "matplotlib", "pillow": "pillow",
        "torch": "torch", "pytorch": "torch",
        "gymnasium": "gymnasium",
        "sb3": "stable-baselines3", "stable-baselines3": "stable-baselines3",
    }
    pat = re.compile(r"(?<![\w.-])(" + "|".join(sorted(alias, key=len, reverse=True))
                     + r")[  ]+v?(\d+\.\d+\.\d+)", re.I)
    # 描述「別人的專案用哪個版本」時不該比對我們的 requirements.txt
    # （例如 06 章寫 gz-physics vendored 的 MuJoCo 是 3.11.0）。
    foreign = re.compile(r"vendor|gz-physics|Isaac|Menagerie|上游", re.I)
    bad = []
    for md in markdown_files():
        for i, line in enumerate(md.read_text(encoding="utf-8").splitlines(), 1):
            if foreign.search(line):
                continue
            for m in pat.finditer(line):
                pkg = alias[m.group(1).lower()]
                want = req.get(pkg)
                # 更正表會同時列出錯的與對的（「MuJoCo 3.12.3 | 3.12.0（筆誤）」），
                # 同一行已經有正確版本就不是錯誤，是在記錄它被改掉了。
                if want and m.group(2) != want and want not in line:
                    bad.append(f"{md.relative_to(ROOT)}:{i}: 寫 {m.group(1)} {m.group(2)}，"
                               f"requirements.txt 是 {want}")
    return sorted(set(bad))


def check_claimed_counts():
    """README / REPORT 宣稱的數量要與實際相符。"""
    bad = []
    actual = {
        "章節": len(chapter_files()),
        "腳本": len(list((ROOT / "scripts").glob("*.py"))),
        "錄影": len(list((ROOT / "runs").glob("*.mp4"))),
    }
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    report = (ROOT / "REPORT.md").read_text(encoding="utf-8")

    # 出處筆數：SOURCES.md 的資料列（扣掉表頭與分隔線）
    src_rows = sum(1 for l in (ROOT / "sources" / "SOURCES.md").read_text(encoding="utf-8").splitlines()
                   if l.startswith("| ") and not l.startswith(("| 教學章節", "| ---")))
    for m in re.finditer(r"(\d+) 筆資料出處", report):
        if int(m.group(1)) != src_rows:
            bad.append(f"REPORT.md: 宣稱 {m.group(1)} 筆出處，實際 {src_rows} 筆")
    for m in re.finditer(r"出處登記（(\d+) 筆）", report):
        if int(m.group(1)) != src_rows:
            bad.append(f"REPORT.md: 宣稱出處登記 {m.group(1)} 筆，實際 {src_rows} 筆")

    # 除錯案例條數：REPORT 第五節的編號清單
    sec = report.split("## 五、除錯案例清單")
    if len(sec) > 1:
        cases = len(re.findall(r"^\d+\. ", sec[1].split("## 六、")[0], re.M))
        # 查表每加一條案例就要手動補一格，補漏了檢查就會誤報。改成算出來的。
        digits = "零一二三四五六七八九"
        def to_zh(n):
            if n < 10:
                return digits[n]
            tens, ones = divmod(n, 10)
            return ("" if tens == 1 else digits[tens]) + "十" + (digits[ones] if ones else "")
        zh = {cases: to_zh(cases)}
        for m in re.finditer(r"(二十[一二三四五六七八九]?|三十[一二三四五六七八九]?)條除錯案例", readme):
            if zh.get(cases) != m.group(1):
                bad.append(f"README.md: 宣稱「{m.group(1)}條」除錯案例，實際 {cases} 條")
    for text, name in ((readme, "README.md"), (report, "REPORT.md")):
        for m in re.finditer(r"(\d+) 篇教學", text):
            if int(m.group(1)) != actual["章節"]:
                bad.append(f"{name}: 宣稱 {m.group(1)} 篇教學，實際 {actual['章節']} 篇")
        for m in re.finditer(r"(\d+) 支 Python", text):
            if int(m.group(1)) != actual["腳本"]:
                bad.append(f"{name}: 宣稱 {m.group(1)} 支腳本，實際 {actual['腳本']} 支")
        for m in re.finditer(r"(\d+) 組錄影", text):
            if int(m.group(1)) != actual["錄影"]:
                bad.append(f"{name}: 宣稱 {m.group(1)} 組錄影，實際 {actual['錄影']} 組")
    return sorted(set(bad))


def check_csv_headers():
    """runs/ 的每個 CSV，header 欄數要與資料欄數一致。

    事故：25 章的 log 加了 rel_x/y/z 三欄，但 header 是單行寫法沒被一起改到，
    寫出 18 欄 header 配 21 欄資料的檔案 —— 讀的人會把欄位對錯。
    """
    import csv as _csv
    bad = []
    for p in sorted((ROOT / "runs").glob("*.csv")):
        with p.open(encoding="utf-8") as f:
            r = _csv.reader(f)
            head = next(r, None)
            row = next(r, None)
        if head and row and len(head) != len(row):
            bad.append(f"{p.relative_to(ROOT)}: header {len(head)} 欄、資料 {len(row)} 欄")
    return bad


CHECKS = [
    ("絕對路徑", check_absolute_paths),
    ("文件相對連結", check_relative_links),
    ("模型可載入", check_models_load),
    ("文件 MJCF 區塊", check_doc_xml_blocks),
    ("章節結構", check_chapter_structure),
    ("章節編號與依賴", check_chapter_numbering),
    ("目錄清單登記", check_inventories),
    ("孤兒圖片", check_orphan_assets),
    ("相依套件登記", check_requirements),
    ("模型接觸穿透", check_model_penetration),
    ("版本號一致", check_version_claims),
    ("宣稱數量", check_claimed_counts),
    ("CSV 欄位一致", check_csv_headers),
]


def main():
    failed = 0
    for name, fn in CHECKS:
        try:
            problems = fn()
        except Exception as e:
            print(f"✗ {name}：檢查本身出錯 — {e}")
            failed += 1
            continue
        if problems:
            failed += 1
            print(f"✗ {name}（{len(problems)} 項）")
            for p in problems[:20]:
                print(f"    {p}")
            if len(problems) > 20:
                print(f"    …另有 {len(problems) - 20} 項")
        else:
            print(f"✓ {name}")
    print()
    if failed:
        print(f"{failed} 項檢查未通過")
        return 1
    print(f"全部 {len(CHECKS)} 項檢查通過")
    return 0


if __name__ == "__main__":
    sys.exit(main())
