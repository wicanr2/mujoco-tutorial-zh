#!/usr/bin/env bash
# 逐支重跑範例並記錄結果，用來確認文件裡的範例還跑得動。
#
# 用法：
#   bash scripts/verify_examples.sh basic scripts/hello_mujoco.py scripts/pd_control.py
#   PYTHON=python3 bash scripts/verify_examples.sh basic scripts/hello_mujoco.py
#   MUJOCO_GL=osmesa bash scripts/verify_examples.sh render scripts/ex_loop_steer.py
#   TIMEOUT=14400 bash scripts/verify_examples.sh sac scripts/ex_rl_sac.py
#
# 每支腳本的上限預設一小時，用 TIMEOUT（秒）調整。CPU 版 SAC 在忙碌的機器上會超過，
# 而被砍掉的 rc=124 看起來就只是「失敗」，不會告訴你它其實只是還沒跑完。
#
# 輸出：workspace/verify/<批次名>/ 底下每支腳本一份 .log，加一份彙總 result.tsv
#       （腳本名、exit code、耗時秒數）。需在 repo 根目錄執行。
set -u
# 直譯器可用 PYTHON 覆寫（CI 沒有 .venv/）
PYTHON="${PYTHON:-.venv/bin/python}"
BATCH="$1"; shift
OUT="workspace/verify/${BATCH}"
mkdir -p "$OUT"
RESULT="$OUT/result.tsv"
: > "$RESULT"
for s in "$@"; do
  name=$(basename "$s" .py)
  start=$(date +%s.%N)
  timeout "${TIMEOUT:-3600}" "$PYTHON" "$s" > "$OUT/$name.log" 2>&1
  rc=$?
  end=$(date +%s.%N)
  dur=$(echo "$end - $start" | bc)
  printf '%s\t%s\t%.1f\n' "$name" "$rc" "$dur" >> "$RESULT"
  echo "[$BATCH] $name rc=$rc ${dur}s"
done
echo "=== $BATCH 完成 ==="
cat "$RESULT"
