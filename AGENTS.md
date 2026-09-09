# MuJoCo 中文教學專案 — AGENTS.md

本檔案為本專案的協作規範，所有參與者（包含 AI 助理）皆應遵守。

## 專案目標

建立一套以 MuJoCo 物理引擎為核心的**繁體中文教學文件**，涵蓋四大主題：

1. **MuJoCo 基礎教學**：以官方文件 <https://mujoco.readthedocs.io/en/stable/overview.html> 為起點，逐步整理 Overview、Programming、Modeling（MJCF）、API 等章節。
2. **資料收集與翻譯**：蒐集官方文件、範例與社群資源，翻譯為繁體中文並補充本地讀者需要的說明。
3. **在 Isaac Sim 與 Gazebo 中替換 / 使用 MuJoCo 引擎**：教學如何整合或改用 MuJoCo 作為物理後端。
4. **URDF 模型匯入**：教學如何將 URDF 模型轉換 / 匯入 MuJoCo（含常見陷阱與修正）。

## 目錄結構

```
docs/
  00-intro/            # 導論、安裝、環境設定
  01-basics/           # MJCF 建模基礎（body, geom, joint, actuator...）
  02-programming/      # Python/C API、模擬迴圈
  03-urdf-import/      # URDF 匯入與轉換
  04-isaac-sim/        # Isaac Sim 中使用/替換 MuJoCo
  05-gazebo/           # Gazebo 中使用/替換 MuJoCo
  glossary.md          # 中英術語對照表
models/                # 教學用範例模型（MJCF、URDF、mesh）
scripts/               # 可執行的範例程式
sources/               # 收集的原始資料清單與出處（SOURCES.md）
```

## 資料收集規範

1. **以官方文件為第一來源**：<https://mujoco.readthedocs.io/en/stable/>，其次為 MuJoCo GitHub（google-deepmind/mujoco）、官方教學 notebook、Isaac Sim / Gazebo 官方文件。
2. **每筆資料必須記錄出處**：在 `sources/SOURCES.md` 登記 URL、擷取日期、對應的教學章節。
3. **注意版本**：記錄資料對應的 MuJoCo 版本號（例如 3.x）；不同版本行為有差異時需在文中註明。
4. **不得抄襲非官方受版權保護內容**：引用官方文件需標註來源；第三方內容僅摘要重述並附連結。

## 翻譯規範（英文 → 繁體中文）

1. 使用**繁體中文**，語氣為教學式、口語但精確。
2. **專有名詞首次出現時附中英文對照**，例如：「關節（joint）」、「致動器（actuator）」；之後可單用中文或英文，但全篇一致。
3. 常見術語一律依 `docs/glossary.md` 對照表翻譯；新術語需先加入對照表再使用。
4. **程式碼、API 名稱、XML 標籤、參數名不翻譯**，保留原文（例如 `<body>`、`mj_step`）。
5. 翻譯不是逐字直譯：以清楚傳達概念為優先，必要時補充原文沒有的解釋（用「譯註：」標示）。
6. 每篇翻譯文件開頭附上**原文連結與對應版本**。

## 教學文件撰寫規範

1. 每篇教學包含：學習目標、前置知識、步驟、**可執行的完整範例**、常見錯誤與除錯、延伸閱讀。
2. 所有程式範例放在 `scripts/` 或 models 放 `models/`，文件中引用路徑，不得只貼無法驗證的片段。
3. 範例以 **Python（mujoco 官方 pip 套件）** 為主；C API 僅在必要時補充。
4. 範例程式必須實際跑過驗證，並註明測試環境（OS、MuJoCo 版本、Python 版本）。
5. 截圖 / 動畫有助於理解時應附上，圖檔放 `docs/assets/`。

## 主題特定要求

### URDF 匯入
- 涵蓋：`<include file="...">` 方式載入、`urdf2mjcf` 類工具、mesh 路徑與 `meshdir`、慣性參數、碰撞幾何、自由度差異（URDF 無 closed-loop）。
- 至少一個完整範例：從現成 URDF（如官方 menagerie 或自建）轉到可在 MuJoCo 模擬。

### Isaac Sim / Gazebo 替換 MuJoCo
- 說明各平台物理引擎架構（PhysX、DART 等）與 MuJoCo 的定位差異。
- 若官方無直接支援，教學以**橋接 / 遷移 workflow** 呈現（例如 MJCF/URDF 模型在兩邊共用、或以 MuJoCo 做獨立物理驗證），並明確標示可行性與限制，**不得宣稱不存在的外掛支援**。

## 版本與相依性

- MuJoCo：以最新穩定版為主，文件中標明版本。
- Python 範例需列出相依套件（`pip install mujoco` 等）。
- 修改程式或模型後，需確認對應教學步驟仍然可執行。

## 協作流程

1. 新章節先在 issue/待辦中列出大綱再動筆。
2. 修改教學內容時，同步更新 `glossary.md`、`SOURCES.md` 與相關索引。
3. 完成標準：文件可讀、範例可跑、出處可查、術語一致。
