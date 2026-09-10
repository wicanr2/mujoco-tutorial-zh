# MuJoCo 繁體中文教學專案 — 成果報告

> 日期：2026-09-10（更新）
> Repo：https://github.com/wicanr2/mujoco-tutorial-zh

## 一、專案目標

依 AGENTS.md 規範建立 MuJoCo 物理引擎的繁體中文教學，涵蓋：官方文件收集與翻譯、MJCF 建模、程式設計、URDF 匯入、Isaac Sim / Gazebo 整合可行性、RL 訓練範例、AMR（叉車 / 搬運車）實驗。所有範例須實際執行驗證，資料須登記出處。

## 二、產出總覽（19 篇教學 + 15 支可執行範例）

### 基礎系列（docs/00–05）

| 篇 | 文件 | 驗證狀態 |
| --- | --- | --- |
| 01 | MuJoCo 導論與安裝 | ✅ 本機實測 |
| 02 | MJCF 建模基礎（雙擺） | ✅ 本機實測 |
| 03 | 程式設計入門（PD 控制） | ✅ 本機實測 |
| 04 | URDF 匯入（二連桿 + 轉存 MJCF） | ✅ 本機實測 |
| 05 | Isaac Sim 整合（MJCF/URDF Importer 流程） | ⚠️ 文件整理，無 Isaac Sim 環境（已標註） |
| 06 | Gazebo 整合（gz-physics 外掛現況分析） | ⚠️ 文件整理（已標註） |

### 程式設計 / RL 系列（docs/02）

| 篇 | 文件 | 驗證狀態 |
| --- | --- | --- |
| 07 | 五個 Python 範例（感測器/伺服/keyframe/平行取樣/mjSpec） | ✅ 全部實測 |
| 08 | RL：ARS 手寫實作 | ✅ 21 秒收斂至 -0.30 |
| 09 | RL：Gymnasium + PPO | ✅ 6 分鐘收斂至 -0.11 |
| 10 | 車桿 swing-up（能量整形 + mjd_transitionFD + LQR） | ✅ 1.19s 盪起、完美穩定 |
| 11 | Viewer 與離屏渲染（osmesa） | ✅ 實測 + 渲染圖 |
| 12 | RL：SAC 對照實驗 | ✅ CPU 未收斂（如實記錄）→ RTX6000 GPU 收斂至 -0.21 |

### AMR 系列（docs/06）

| 篇 | 文件 | 驗證狀態 |
| --- | --- | --- |
| 13 | AMR 叉車建模與牙叉控制（取貨流程） | ✅ 棧板實際抬起 |
| 14 | 搬運車機械手臂 + 阻尼最小平方法 IK | ✅ 末端誤差 2.6 cm |
| 15 | 六軸手臂 6D IK（位置+姿態） | ✅ 30 次迭代收斂，1.1 cm / 0.036 rad |
| 16 | 棧板材質實驗（木頭 vs 塑膠 20 kg） | ✅ 木 0.4 cm vs 塑膠 11.9 cm 滑動 |
| 17 | 門架前傾滑落邊界掃參 | ✅ 三案例實測 + 理論對照 |
| 18 | MR1533 真實 mesh 叉車（TB3 資產匯入） | ✅ 升降/前傾實測 + Blender 渲染 |
| 19 | Blender 程序化棧板建模（STL）+ 載重驗證 | ✅ 兩材質通過上下左右測試 |

附錄：`docs/glossary.md` 中英術語對照表、`sources/SOURCES.md` 資料出處登記（20+ 筆）。

## 三、RL 對照實驗數據

同一單擺 swing-up 任務：

| 方法 | 最終回報 | 訓練時間 | 硬體 |
| --- | --- | --- | --- |
| ARS（手寫） | -0.30 | ~21 秒 | CPU |
| PPO（SB3） | -0.11 | ~6 分鐘（150k 步） | CPU |
| SAC（SB3） | 未收斂（60k 步） | ~30 分鐘 | CPU |
| SAC（zoo 超參） | **-0.21（60k 步）** | **~4 分鐘** | RTX Pro 6000 |

結論：SAC 的樣本效率需要足夠更新頻率（`train_freq=1`）；CPU 被迫降頻導致不收斂，GPU 補足後 60k 步收斂（PPO 用 150k 步）。遠端訓練環境事後已依使用者要求全數清除。

## 四、AMR 實驗數據

- **取貨流程**（13）：對位 → 插入 → 升降 → 側移，棧板實際被抬起（z=0.14 m）
- **手臂 IK**（14/15）：3D 位置 IK 2.6 cm；6D IK 1.1 cm / 0.036 rad
- **摩擦對照**（16）：急側移下木頭棧板滑 0.4 cm、塑膠 11.9 cm（近滑落邊緣）
- **前傾邊界**（17）：塑膠 ~20.6° 開始蠕動（理論 atan μ=19.3°）；45° 內皆不掉落；快傾反而不滑（慣性效應）
- **Blender 資產**（18/19）：MR1533 叉車 mesh（TB3 實驗）+ 程序化棧板，視覺/碰撞分離原則

## 五、累積除錯案例（皆已成為教學內容）

1. touch 感測器 site 要放在接觸點附近
2. 純 PD 的穩態誤差（無重力補償）
3. ARS 探索尺度敏感
4. PPO 樣本預算（60k 不夠 → 150k + frame skip）
5. 能量整形目標能量的慣性系常數
6. mjd_transitionFD 是離散 Jacobian（需除以 dt）
7. 手調雙擺能量控制器失敗 → 改車桿並誠實標註
8. 模型沒有 `<light>` 畫面全黑
9. SAC CPU 更新頻率陷阱
10. URDF 不做 schema 檢查（拼字靜默忽略）
11. 裝飾輪 / 滑架與門架 / 叉齒與底盤的**內部接觸摩擦鎖死**（三度出現！）
12. MuJoCo 接觸摩擦預設**取兩 geom 最大值**
13. IK 不收斂先檢查**工作空間**；姿態誤差用官方四元數配方（mju_subQuat 參考系不同）
14. MuJoCo 編譯器對 mesh 做**主軸重新對齊** — 對稱 mesh 方向會亂，解法：join 單一 mesh + STL
15. 掉落判定不能用絕對高度（前傾幾何會造成誤判）

## 六、Isaac Sim / Gazebo 可行性結論

- **Isaac Sim**：綁定 PhysX，引擎不可替換；走 MJCF/URDF Importer 模型互通（6.0 起 `mujoco-usd-converter`）+ 雙引擎工作流
- **Gazebo**：gz-physics 外掛架構，MuJoCo 外掛僅有未維護 MVP；以 URDF 為共同交換格式

## 七、環境與版本

- 本機：Linux、MuJoCo 3.12.0、Python 3.12.3、sb3 2.9.0、torch 2.14.0、scipy、pillow、Blender 4.2.11（headless EEVEE）
- GPU 訓練：RTX Pro 6000（torch cu130），環境已清除

## 八、後續建議

1. 04/05 章在有 Isaac Sim / Gazebo 的機器上實測後更新
2. 03 章補 Menagerie 真實機器人實例
3. AMR 線：完整 pick-and-place 串聯、差速/舵輪真實底盤、RL 對位訓練
4. CI 自動執行所有範例，防止文件與程式脫節
