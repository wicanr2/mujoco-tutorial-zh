"""檢查文件裡的外部連結是否還通（需要網路，所以不放進 check_docs.py 的必跑項）。

組織改名、repo 搬家都會讓舊網址變 404，而且不會有人通知你 —— 定期跑一次。

用法：
    python scripts/check_links.py           # 檢查全部
    python scripts/check_links.py --slow 2  # 每個請求之間多等 2 秒
"""
import re
import sys
import time
import urllib.error
import urllib.request
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SKIP_DIRS = (".venv", "_site", "workspace", ".git", "__pycache__")
UA = {"User-Agent": "Mozilla/5.0 (compatible; mujoco-tutorial-zh link check)"}
TIMEOUT = 25


def markdown_files():
    return [p for p in ROOT.glob("**/*.md")
            if not any(d in p.relative_to(ROOT).parts for d in SKIP_DIRS)]


def collect():
    """回傳 {url: [出現的檔案]}。"""
    pat = re.compile(r"\[[^\]]*\]\((https?://[^)\s]+)\)")
    urls = defaultdict(list)
    for md in markdown_files():
        for m in pat.finditer(md.read_text(encoding="utf-8")):
            urls[m.group(1).rstrip(".,;")].append(str(md.relative_to(ROOT)))
    return urls


def check(url):
    """回傳 (狀態碼或錯誤字串, 最終網址)。HEAD 不支援時退回 GET。"""
    for method in ("HEAD", "GET"):
        req = urllib.request.Request(url, headers=UA, method=method)
        try:
            with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
                return r.status, r.url
        except urllib.error.HTTPError as e:
            if method == "HEAD" and e.code in (403, 405, 501):
                continue                      # 有些站擋 HEAD，換 GET 再試
            return e.code, url
        except Exception as e:                # noqa: BLE001 - 網路錯誤形式很多
            if method == "HEAD":
                continue
            return type(e).__name__, url
    return "unknown", url


if __name__ == "__main__":
    slow = 0.0
    if "--slow" in sys.argv:
        slow = float(sys.argv[sys.argv.index("--slow") + 1])
    urls = collect()
    print(f"檢查 {len(urls)} 個外部連結（出現在 {len(markdown_files())} 份文件裡）\n")
    bad, moved = [], []
    for i, (url, files) in enumerate(sorted(urls.items()), 1):
        status, final = check(url)
        ok = status == 200
        if not ok:
            bad.append((url, status, files))
        elif final.rstrip("/") != url.rstrip("/"):
            moved.append((url, final, files))
        print(f"  [{i:2d}/{len(urls)}] {status}  {url[:88]}")
        if slow:
            time.sleep(slow)

    print()
    if moved:
        print(f"重導向 {len(moved)} 個（仍可用，但原網址已經搬家）：")
        for url, final, files in moved:
            print(f"  {url}\n    → {final}\n    出現在：{', '.join(sorted(set(files)))}")
    if bad:
        print(f"\n✗ {len(bad)} 個連結不通：")
        for url, status, files in bad:
            print(f"  [{status}] {url}\n    出現在：{', '.join(sorted(set(files)))}")
        sys.exit(1)
    print("✓ 全部可用")
