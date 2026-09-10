# MuJoCo 中文教學專案 — AGENTS.md

本檔案為本專案的協作規範，所有參與者（包含 AI 助理）皆應遵守。

## 專案目標

建立一套以 MuJoCo 物理引擎為核心的**繁體中文教學文件**，涵蓋五條主線：

1. **MuJoCo 基礎教學**：以官方文件 <https://mujoco.readthedocs.io/en/stable/overview.html> 為起點，逐步整理 Overview、Programming、Modeling（MJCF）、API 等章節。
2. **資料收集與翻譯**：蒐集官方文件、範例與社群資源，翻譯為繁體中文並補充本地讀者需要的說明。
3. **在 Isaac Sim 與 Gazebo 中替換 / 使用 MuJoCo 引擎**：教學如何整合或改用 MuJoCo 作為物理後端。
4. **URDF 模型匯入**：教學如何將 URDF 模型轉換 / 匯入 MuJoCo（含常見陷阱與修正）。
5. **模擬實驗**：以強化學習（RL）訓練與 AMR 叉車 / 搬運車的搬運任務為題材，把前四條線的內容用在會動、可量測、有輸出的完整實驗上。

## 目錄結構

```
docs/
  00-intro/            # 導論、安裝、環境設定
  01-basics/           # MJCF 建模基礎（body, geom, joint, actuator...）
  02-programming/      # Python/C API、模擬迴圈、RL 訓練
  03-urdf-import/      # URDF 匯入與轉換
  04-isaac-sim/        # Isaac Sim 中使用/替換 MuJoCo
  05-gazebo/           # Gazebo 中使用/替換 MuJoCo
  06-amr/              # AMR 叉車 / 搬運車實驗
  assets/              # 教學插圖、渲染圖、filmstrip
  glossary.md          # 中英術語對照表
  README.md            # 章節索引（唯一的章節清單）
models/                # 模擬模型：MJCF、URDF、mesh（OBJ/STL）
scripts/               # 可執行範例與建模 / 驗證工具
runs/                  # 實驗輸出：影片、CSV、軌跡圖（見 runs/README.md）
policies/              # 訓練好的策略權重（.npy / .zip）
sources/SOURCES.md     # 資料出處登記
workspace/             # 本機工作區，不進版控（見 workspace/README.md）
requirements.txt       # 實測過的套件版本
LICENSE                # 授權條款（source-available，基於 RRSAL-1.0）
_config.yml            # GitHub Pages（Jekyll）設定
_layouts/default.html  # 網站版面樣板
assets/css/main.css    # 網站樣式
```

### 檔案歸屬（決定東西該放哪）

| 類型 | 位置 | 進版控 |
| --- | --- | --- |
| 模擬模型、mesh | `models/` | 是 |
| 可執行範例、建模與驗證工具 | `scripts/` | 是 |
| 實驗輸出（影片、CSV、軌跡圖） | `runs/` | 是 |
| 訓練好的策略權重 | `policies/` | 是 |
| 教學文件與插圖 | `docs/` | 是 |
| 重跑 log、備份、訓練中間檔 | `workspace/` | 否 |
| 授權條款 | 根目錄 `LICENSE` | 是（發行包也要帶一份） |
| 網站設定與版面 | `_config.yml`、`_layouts/`、`assets/css/` | 是 |
| Jekyll 建置產物 | `_site/` | 否 |
| MuJoCo 執行時警告紀錄（`MUJOCO_LOG.TXT`） | 產生於 cwd | 否 |

新增檔案前先對照這張表；沒有對應欄位的產物，預設放 `workspace/`。

## 資料收集規範

1. **以官方文件為第一來源**：<https://mujoco.readthedocs.io/en/stable/>，其次為 MuJoCo GitHub（google-deepmind/mujoco）、官方教學 notebook、Isaac Sim / Gazebo 官方文件。
2. **每筆資料必須記錄出處**：在 `sources/SOURCES.md` 登記 URL、擷取日期、對應的教學章節。
3. **注意版本**：記錄資料對應的 MuJoCo 版本號（例如 3.x）；不同版本行為有差異時需在文中註明。
4. **不得抄襲非官方受版權保護內容**：引用官方文件需標註來源；第三方內容僅摘要重述並附連結。
5. **描述別人專案狀態的句子要標查證日期**（「某功能還沒有」「外掛未維護」「某工具已重寫」
   這類）。這種斷言寫的當下是對的，幾個月後就不是了，而且不會有人通知你。定期重查，
   尤其是**否定斷言**——「還不存在」最容易在你沒看的時候變成假的。
6. **組織與 repo 會改名**，舊網址會變 404（`ignitionrobotics` → `gazebosim` 就是一例）。
   引用 issue / PR 前先確認網址現在還通。

## 翻譯規範（英文 → 繁體中文）

1. 使用**繁體中文**，語氣為教學式、口語但精確。
2. **專有名詞首次出現時附中英文對照**，例如：「關節（joint）」、「致動器（actuator）」；之後可單用中文或英文，但全篇一致。
3. 常見術語一律依 `docs/glossary.md` 對照表翻譯；新術語需先加入對照表再使用。
4. **程式碼、API 名稱、XML 標籤、參數名不翻譯**，保留原文（例如 `<body>`、`mj_step`）。
5. 翻譯不是逐字直譯：以清楚傳達概念為優先，必要時補充原文沒有的解釋（用「譯註：」標示）。
6. 每篇翻譯文件開頭附上**原文連結與對應版本**。

## 教學文件撰寫規範

1. 每篇教學包含：學習目標、前置知識、步驟、**可執行的完整範例**、常見錯誤與除錯、延伸閱讀。
2. 所有程式範例放在 `scripts/`、模型放 `models/`，文件中引用路徑，不得只貼無法驗證的片段。
   **模型與腳本裡不准出現絕對路徑**：`meshdir` 與 mesh `file` 用相對於 XML 的路徑；腳本
   要組路徑時用 `Path(__file__).resolve().parent.parent` 算 repo 根。絕對路徑在作者的
   機器上永遠正常，別人 clone 下來直接失敗 — 改完換一個工作目錄實測一次。
3. 範例以 **Python（mujoco 官方 pip 套件）** 為主；C API 僅在必要時補充。
4. 範例程式必須實際跑過驗證，並在文件開頭註明測試環境（OS、MuJoCo 版本、Python 版本）。
5. 截圖 / 動畫有助於理解時應附上，圖檔放 `docs/assets/`。
6. **除錯紀錄要寫進文件**：開發過程踩到的坑（現象 → 原因 → 解法）是本專案的主要價值之一，不要在收尾時刪掉。
7. **文件描述的是現況**，不是「相對上一版改了什麼」。修正錯誤時直接改寫成正確內容，不在正文留「原本以為…後來發現…」的敘述；已經修好的 bug 不留註記，否則下一位讀者會照著不存在的問題去查。

## 實驗紀錄規範

實驗類章節（RL、AMR）除了文件本身，還要留下可重跑、可查證的輸出。

1. **輸出位置與命名**：一律寫進 `runs/`，同一個實驗共用前綴 — `<實驗名>.mp4`、`<實驗名>_log.csv`、`<實驗名>_traj.png`。腳本自己 `os.makedirs("runs", exist_ok=True)`。
2. **登記**：新增輸出時同步更新 `runs/README.md` 的檔案清單（檔名、產生腳本、對應章節、內容摘要），CSV 要有欄位說明。
3. **CSV**：50 Hz 取樣、第一欄為 `time`（模擬時間，秒）。姿態用 RPY（弧度）並註明四元數轉換方式；滑動 / 漂移量要說明**參考座標系**與**基準時刻**。
4. **影片**：30 fps，`mujoco.Renderer` 離屏渲染。無顯示器環境需 `MUJOCO_GL=osmesa`（或 `egl`）。
5. **數字來自實跑**：文件裡的每個數字都要能在對應的 log 或 stdout 找到。心算的幾何值不算 — 用 `site_xpos` 之類的實際量測驗證。模型參數（質量、關節範圍、摩擦）要用 `MjModel` 載入後讀出來對，不要憑 XML 印象寫。
6. **引用另一篇文件的說法前，先驗那個說法**。錯誤斷言會沿著交叉引用擴散：一處寫錯，下一篇照抄，就變成兩處。
7. **判定條件要對準你真正想證明的事**。「棧板被抬起來過」和「棧板被搬到了」是兩件事；
   只檢查終點位置，「掉下去」會被算成「放下去」。搬運與放置類的實驗要檢查**過程**
   （例如 z 單調下降、貨與車的相對位移在容差內），不是只看最後一幀。
8. **腳本印出「驗證通過」不等於實驗成立**。宣告成功前把 CSV 拉出來看整段軌跡。
9. **失敗的實驗照實寫**：不收斂、平台限制、與理論對不上的結果都保留，並寫清楚原因與適用邊界。

## 驗證與交付

1. 改動腳本或模型後，用 `scripts/verify_examples.sh` 重跑受影響的範例，確認 exit code 與輸出仍符合文件所述。

   ```bash
   bash scripts/verify_examples.sh basic scripts/hello_mujoco.py
   MUJOCO_GL=osmesa bash scripts/verify_examples.sh render scripts/ex_loop_steer.py
   ```

2. 重跑會覆蓋 `runs/` 既有輸出，動手前先備份到 `workspace/backup/`，跑完比對差異再決定是否保留新版。
   MuJoCo 的模擬與 Blender 匯出的 STL 都是確定性的，重跑後檔案應該位元相同；出現差異就是真的
   有東西變了，要查清楚。例外是 Blender 的 EEVEE 渲染圖，每次取樣不同、位元必然不一致但視覺
   內容相同 — 這類圖驗證完直接還原，不要拿去覆蓋版控裡的檔案。
3. **章節清單只維護一份**：`docs/README.md` 是唯一的章節索引，根目錄 `README.md` 只放成果摘要與入口連結。新增章節時，`docs/README.md`、`README.md` 的成果段落、`sources/SOURCES.md` 一起更新。

## 主題特定要求

### URDF 匯入
- 涵蓋：`<include file="...">` 方式載入、`urdf2mjcf` 類工具、mesh 路徑與 `meshdir`、慣性參數、碰撞幾何、自由度差異（URDF 無 closed-loop）。
- 至少一個完整範例：從現成 URDF（如官方 menagerie 或自建）轉到可在 MuJoCo 模擬。

### Isaac Sim / Gazebo 替換 MuJoCo
- 說明各平台物理引擎架構（PhysX、DART 等）與 MuJoCo 的定位差異。
- 若官方無直接支援，教學以**橋接 / 遷移 workflow** 呈現（例如 MJCF/URDF 模型在兩邊共用、或以 MuJoCo 做獨立物理驗證），並明確標示可行性與限制，**不得宣稱不存在的外掛支援**。

### AMR 實驗
- 視覺與碰撞分離：高面數 mesh 設 `contype="0" conaffinity="0"`，碰撞另用簡化幾何。
- 機構內部零件之間預設不互相碰撞（用 `contact/exclude` 或關掉碰撞），避免摩擦鎖死造成「關節不動」。
- 用外部資產（TB3 mesh 等）時在文件標明來源路徑與授權狀態。

## 執行環境

範例直接在本機 venv 執行（`.venv/`），不走容器 — GPU 訓練那台 RTX Pro 6000 不能跑
Docker，只能用 userland 執行，本機端跟著用同一種方式，兩邊的驗證結果才對得起來。
`requirements.txt` 是這套環境的實測版本組合。

## 網站（GitHub Pages）

<https://wicanr2.github.io/mujoco-tutorial-zh/> 由 Jekyll 從 repo 根目錄建置，
`README.md` 是首頁（`jekyll-readme-index`），各目錄的 `README.md` 成為該目錄的索引頁。

- **文件裡一律用相對路徑連結**（`docs/06-amr/01-amr-forklift.md`、`../../runs/loop.mp4`）。
  `jekyll-relative-links` 會把 `.md` 連結改寫成對應的 `.html`，同一份 markdown 在 GitHub
  與網站上都能點。寫成絕對路徑或 `/` 開頭會在網站上壞掉（站台有 baseurl）。
- **版面規則**：層級靠細線、留白與字級，不用卡片與色塊；關鍵路徑色只有一個（鏽橙）；
  表格只用細分隔線，不加粗外框、彩色表頭或斑馬紋。深色模式共用同一組語意變數。
- **改完版面要實際渲染出來看**，只確認建置成功不算數。本地預覽：

  ```bash
  docker run --rm --network none -u "$(id -u):$(id -g)" -e HOME=/tmp \
    --tmpfs /tmp -v "$PWD":/srv/jekyll -w /srv/jekyll acan-jekyll:gh-pages \
    jekyll build --destination /srv/jekyll/_site
  ```

## 版本與相依性

- 套件版本以 `requirements.txt` 為準，那是實際跑過的組合；文件內提到版本時要與它一致。
- MuJoCo 以最新穩定版為主，升版後需重跑 `scripts/verify_examples.sh` 確認範例仍可執行。
- 修改程式或模型後，需確認對應教學步驟仍然可執行。

## 協作流程

1. 新章節先列出大綱再動筆。
2. 修改教學內容時，同步更新 `docs/glossary.md`、`sources/SOURCES.md` 與 `docs/README.md`。
3. 完成標準：文件可讀、範例可跑、出處可查、術語一致、輸出可重現。
