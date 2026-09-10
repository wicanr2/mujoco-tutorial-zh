# workspace/ — 本機工作區

這個目錄放**執行過程產生、不屬於交付物**的檔案。除了本說明外，其餘內容都被 `.gitignore` 排除，
可以隨時刪掉重建。

| 子目錄 | 內容 |
| --- | --- |
| `verify/` | 範例重跑驗證的 log 與結果（`scripts/verify_examples.sh` 產生，每支腳本一份 `.log` 與彙總 `result.tsv`） |
| `backup/` | 動到 `runs/` 之前的備份 |

交付物各有固定去處，不要放這裡：

- 模擬模型 → `models/`
- 範例程式 → `scripts/`
- 實驗輸出（影片、CSV、軌跡圖）→ `runs/`
- 訓練好的策略權重 → `policies/`
- 教學文件與圖片 → `docs/`

## 重跑驗證

```bash
bash scripts/verify_examples.sh <批次名> scripts/xxx.py scripts/yyy.py
MUJOCO_GL=osmesa bash scripts/verify_examples.sh render scripts/ex_loop_steer.py
```

驗證腳本本身在 `scripts/verify_examples.sh`（進版控），輸出才落到這裡。
