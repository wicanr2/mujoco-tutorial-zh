# 05｜Isaac Sim 與 MuJoCo：模型互通與工作流程

> 來源：[MJCF Importer Extension — Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_mjcf.html)、[isaacsim.asset.importer.mjcf API](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.asset.importer.mjcf/docs/index.html)、[Importing a New Asset — Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)、[Isaac Sim Discussion #160](https://github.com/isaac-sim/IsaacSim/discussions/160)
>
> 擷取日期：2026-09-09
>
> ⚠️ 本章依 AGENTS.md 規範如實標示可行性：本章範例需在 Isaac Sim 環境執行，**本機未安裝 Isaac Sim，程式碼僅依官方文件整理、未實測**。

## 學習目標

- 理解 Isaac Sim（PhysX）與 MuJoCo 的引擎定位差異
- 了解「在 Isaac Sim 中替換物理引擎」這件事的可行性
- 學會 MJCF / URDF → USD 的匯入工作流程
- 建立「MuJoCo 驗證、Isaac Sim 呈現」的雙引擎工作模式

## 前置知識

- [01｜導論](../00-intro/01-what-is-mujoco.md)、[04｜URDF 匯入](../03-urdf-import/01-import-urdf.md)

## 先說結論：Isaac Sim 的物理引擎不能被「換成」MuJoCo

Isaac Sim 的物理模擬**深度綁定 NVIDIA PhysX**（GPU 加速、與 USD 場景圖、RTX 渲染、ROS 橋接整合），官方沒有提供把物理後端替換成 MuJoCo 的機制。這與 Gazebo 的外掛式物理引擎架構不同（見下一章）。

因此「Isaac Sim 使用 MuJoCo」的實際做法是**模型與流程層面的整合**，而非引擎替換：

1. **把 MJCF / URDF 模型匯入 Isaac Sim**（轉成 USD），用 PhysX 跑；
2. **同一模型在 MuJoCo 中獨立做物理驗證**（控制演算法、動力學分析）；
3. 兩邊共用同一份 URDF/MJCF 作為「單一事實來源」。

## 引擎定位差異

| | MuJoCo | Isaac Sim (PhysX) |
| --- | --- | --- |
| 座標表示 | 廣義座標（generalized coordinates） | 最大座標（maximal/Cartesian coordinates） |
| 接觸模型 | 凸最佳化、柔軟約束、可解析求逆 | LCP 類求解、GPU 平行 |
| 強項 | 接觸豐富的動力學、控制、系統辨識 | 高擬真渲染、感測器模擬、大規模平行環境 |
| 模型格式 | MJCF / URDF | USD（USD Physics schema） |
| 典型用途 | RL 訓練、控制研究、生物力學 | 數位孿生、合成資料、sim-to-real |

## MJCF / URDF 匯入 Isaac Sim

Isaac Sim 內建兩個匯入器：

- **MJCF Importer**（`isaacsim.asset.importer.mjcf`）：把 MJCF 轉成 USD。Isaac Sim 6.0.0 起
  以新的 Python 後端 [`mujoco-usd-converter`](https://github.com/newton-physics/mujoco-usd-converter)
  重寫（見 [Discussion #160](https://github.com/isaac-sim/IsaacSim/discussions/160)）。該工具由
  newton-physics 維護、Apache-2.0、也發佈在 PyPI（查證 2026-09-10 為 0.5.0），可獨立當 Python
  模組或 CLI 用，不必開 Isaac Sim。**官方自述仍是 Alpha**：轉換涵蓋視覺幾何與材質、body、
  碰撞幾何、site、關節與致動器，已知限制列在專案的 CHANGELOG。
- **URDF Importer**（`isaacsim.asset.importer.urdf`）：把 URDF 轉成 USD。

### GUI 方式

Isaac Sim 選單：`File → Import`，或在 Extension Manager 啟用對應 importer 後直接開啟 `.xml`（MJCF）/ `.urdf` 檔。

### Python 方式（官方範例，依 [API 文件](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.asset.importer.mjcf/docs/index.html)整理）

```python
from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig

config = MJCFImporterConfig()
config.import_config.asset_path = "/path/to/robot.xml"   # MJCF 檔
config.import_config.fix_base = False

importer = MJCFImporter()
# 匯入後產生 USD stage，關節會映射為 USD Physics 的 Joint，
# 致動器對應為 Drive API
```

在 Isaac Lab 中則可用 `MjcfConverter` / `MjcfConverterCfg`（`isaaclab.sim.converters.mjcf_converter`）做同樣的轉換，例如官方教學就是把 MuJoCo 版的 Unitree H1 人形機器人匯入（[Importing a New Asset](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)）。

## 建議工作流程

```
            ┌──────────── 單一模型來源 ────────────┐
            │        URDF / MJCF（Git 管理）        │
            └──────┬───────────────────┬───────────┘
                   │                   │
        MjModel.from_xml_path   MJCF/URDF Importer
                   │                   │
              MuJoCo 引擎          Isaac Sim (USD/PhysX)
        控制演算法、動力學驗證    渲染、感測器、sim-to-real
```

1. 模型維護在 URDF 或 MJCF（參考 [04｜URDF 匯入](../03-urdf-import/01-import-urdf.md) 的轉換流程）。
2. 需要嚴謹接觸動力學時，在 MuJoCo 跑（例如用本專案 [03｜PD 控制](../02-programming/01-simulation-loop.md) 的方式）。
3. 需要相機/LiDAR 等感測器資料或高擬真場景時，匯入 Isaac Sim。
4. **兩邊物理結果不會完全一致**：接觸模型不同（凸最佳化 vs LCP），控制器在兩邊都需重新調參。

## 常見錯誤與除錯

- **匯入後關節不會動**：舊版 MJCF 轉換器對 revolute joint 的 Drive 支援不全（[論壇回報](https://forums.developer.nvidia.com/t/mjcf-conversion-problem/257331)），請用 Isaac Sim ≥ 6.0.0 的新轉換器，或先轉成 URDF 再匯入。
- **複雜 MJCF 功能遺失**：tendon、composite、自訂 solver 參數等在 USD 沒有對應，匯入時會被捨棄 — 儘量用 URDF 或簡化的 MJCF 子集作為交換格式。
- **座標/方向跑掉**：檢查 `compiler angle` 與 USD 的上方向（Z-up vs Y-up stage 設定）。
- **物理行為差異**：這是引擎差異而非 bug，見上表。

## 延伸閱讀

- [MJCF Importer Extension（Isaac Sim 5.1 文件）](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_mjcf.html)
- [isaacsim.asset.importer.mjcf API](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.asset.importer.mjcf/docs/index.html)
- [Isaac Lab: Importing a New Asset](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)
- [Isaac Sim Discussion #160：MJCF Converter 6.0.0 重寫](https://github.com/isaac-sim/IsaacSim/discussions/160)
