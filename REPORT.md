# MuJoCo 繁體中文教學專案 — 成果報告

> 日期：2026-09-09
> Repo：https://github.com/wicanr2/mujoco-tutorial-zh

## 一、專案目標

依 AGENTS.md 規範建立 MuJoCo 物理引擎的繁體中文教學，涵蓋：官方文件收集與翻譯、MJCF 建模、程式設計、URDF 匯入、Isaac Sim / Gazebo 整合可行性、RL 訓練範例。所有範例須實際執行驗證，資料須登記出處。

## 二、產出總覽

| 篇 | 文件 | 範例 | 驗證狀態 |
| --- | --- | --- | --- |
| 01 | MuJoCo 導論與安裝 | `hello_mujoco.py` 自由落體 | ✅ 本機實測 |
| 02 | MJCF 建模基礎 | `run_pendulum.py` 雙擺 | ✅ 本機實測 |
| 03 | 程式設計入門 | `pd_control.py` PD 控制單擺 | ✅ 本機實測 |
| 04 | URDF 匯入 | `load_urdf.py` 二連桿 + 轉存 MJCF | ✅ 本機實測 |
| 05 | Isaac Sim 整合 | MJCF/URDF Importer 流程 | ⚠️ 文件整理，本機無 Isaac Sim 未實測（已標註） |
| 06 | Gazebo 整合 | gz-physics 外掛現況分析 | ⚠️ 文件整理，未實測（已標註） |
| 07 | 更多 Python 範例 | 感測器 / 伺服 / keyframe / 平行取樣 / mjSpec（5 支） | ✅ 全部實測 |
| 08 | RL：ARS 手寫實作 | `ex_rl_swingup.py` | ✅ 21 秒收斂至 -0.30 |
| 09 | RL：Gymnasium + PPO | `ex_rl_ppo.py` | ✅ 6 分鐘收斂至 -0.11 |
| 10 | 車桿 swing-up | 能量整形 + `mjd_transitionFD` + LQR | ✅ 1.19s 盪起，完美穩定 |
| 11 | Viewer 與離屏渲染 | `ex_viewer.py` + 渲染圖 | ✅ 無頭環境 osmesa 實測 |
| 12 | RL：SAC 對照實驗 | CPU 版 / GPU 版 | ✅ CPU 未收斂（如實記錄）；GPU 收斂至 -0.21 |

附錄：`docs/glossary.md` 中英術語對照表（31 條）、`sources/SOURCES.md` 資料出處登記（18 筆）。

## 三、RL 三部曲實驗數據

同一個單擺 swing-up 任務（觀測 `[cosθ, sinθ, ω/8]`、回報 `-(1+cosθ) - 0.05(ω/8)² - 0.01u²`）：

| 方法 | 最終回報 | 訓練時間 | 硬體 |
| --- | --- | --- | --- |
| ARS（手寫隨機搜索，08 章） | -0.30 | ~21 秒 | CPU |
| PPO（SB3，09 章） | -0.11 | ~6 分鐘（150k 步） | CPU |
| SAC（SB3，12 章） | **未收斂**（60k 步卡在 -2.0） | ~30 分鐘 | CPU |
| SAC GPU 版（zoo 超參） | **-0.21**（60k 步收斂） | **~4 分鐘** | RTX Pro 6000 |

結論：SAC 的樣本效率優勢需要足夠的更新頻率（`train_freq=1`）才能發揮；CPU 上更新太慢被迫降頻，導致不收斂。GPU 補足更新量後，SAC 用 60k 步收斂（PPO 用了 150k 步）。

## 四、開發過程中記錄的真實除錯案例

這些都已成為教學內容（「常見錯誤與除錯」段落）：

1. **touch 感測器讀值為 0** — site 放在球心量不到接觸，需移到接觸點附近（07 章）。
2. **PD 控制的穩態誤差** — 無重力補償時目標 π/2 只到 1.44 rad（03 章）。
3. **ARS 探索尺度** — 噪音 0.1 / 學習率 0.05 太小完全不學習，調大才收斂（08 章）。
4. **PPO 樣本預算** — 60k 步不夠，150k 步 + frame skip 2 才收斂（09 章）。
5. **能量常數慣性系** — 車桿能量整形的目標能量 E₀ 差一個常數（2mgl vs mgl）（10 章）。
6. **mjd_transitionFD 是離散 Jacobian** — 忘記除以 dt 轉連續時間，LQR 增益爆出 10⁹（10 章）。
7. **雙擺能量整形失敗** — 手調控制器不穩定，改為車桿並在文中誠實標註（10 章）。
8. **模型沒有 `<light>` 畫面全黑** — MuJoCo 無全域光照（11 章）。
9. **SAC CPU 更新頻率陷阱** — `train_freq` 降頻導致不收斂；GPU 解決（12 章）。
10. **URDF 不做 schema 檢查** — 屬性拼錯會被靜靜忽略（04 章）。

## 五、Isaac Sim / Gazebo 可行性結論

- **Isaac Sim**：物理引擎深度綁定 PhysX，**無法替換成 MuJoCo**。可行路線是模型互通：MJCF/URDF Importer 轉 USD（Isaac Sim 6.0 起用 `mujoco-usd-converter` 重寫），搭配「MuJoCo 做物理驗證、Isaac Sim 做渲染與感測器」的雙引擎工作流。
- **Gazebo**：gz-physics 為外掛架構（預設 DART、另有 Bullet）。MuJoCo 外掛**僅有未維護的實驗 MVP**（ign-physics#299），無官方支援；實務上以 URDF 為共同交換格式，Gazebo 端 `gz sdf -p` 轉 SDF。

## 六、環境與版本

- 本機：Linux、MuJoCo 3.12.0、Python 3.12.3、sb3 2.9.0、gymnasium 1.3.0、torch 2.14.0、scipy、pillow
- GPU 訓練：遠端 RTX Pro 6000（torch 2.14.0+cu130）；訓練完成後遠端環境已依使用者要求全數清除

## 七、後續建議

1. 04/05 章（Isaac Sim / Gazebo）在裝有對應軟體的機器上實測後更新。
2. 03 章補 Menagerie 真實機器人（Franka / Unitree）URDF 匯入實例。
3. RL 線可延伸：MJX（物理上 GPU）、domain randomization、acrobot（PFL 控制）。
4. 建立 CI：自動執行 `scripts/` 所有範例，確保文件與程式不脫節。
