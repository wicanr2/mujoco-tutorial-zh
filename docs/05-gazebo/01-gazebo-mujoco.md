# 06｜Gazebo 與 MuJoCo：物理引擎外掛機制與模型互通

> 來源：[Gazebo Sim: Physics engines](https://gazebosim.org/api/sim/9/physics.html)、[Switching physics engines](https://gazebosim.org/api/physics/6/switchphysicsengines.html)、[Use a custom engine with Gazebo Physics](https://gazebosim.org/api/physics/9/usecustomengine.html)、[gz-physics GitHub](https://github.com/gazebosim/gz-physics)、[gz-physics#299: MuJoCo plugin 實驗](https://github.com/ignitionrobotics/ign-physics/issues/299)
>
> 擷取日期：2026-09-09
>
> ⚠️ 本章依 AGENTS.md 規範如實標示可行性：**官方並無維護中的 MuJoCo 物理引擎外掛**；文中所述客製外掛做法需在 Gazebo 環境實作，本機未安裝 Gazebo，程式碼未實測。

## 學習目標

- 理解 Gazebo 的外掛式物理引擎架構（gz-physics）
- 了解「在 Gazebo 中改用 MuJoCo」的真實可行性與現況
- 學會 URDF ↔ SDF ↔ MJCF 的模型互通方式

## 前置知識

- [05｜Isaac Sim 與 MuJoCo](../04-isaac-sim/01-isaac-sim-mujoco.md)

## Gazebo 的物理引擎架構

與 Isaac Sim 深度綁定 PhysX 不同，**Gazebo 的物理引擎是可插拔的**。Gazebo Sim 透過抽象層 [gz-physics](https://github.com/gazebosim/gz-physics) 在執行期選擇物理引擎：

- **DART**：預設引擎，功能最完整（與 MuJoCo 同屬廣義座標陣營）。
- **Bullet**：官方提供兩種變體（`bullet`、bullet-featherstone）。
- **Trivial Physics Engine**：官方示範用的極簡引擎，展示如何寫自己的外掛。

切換方式（需在 SDF world 或指令列指定）：

```xml
<physics name="1ms" type="dart">  <!-- 預設；也可為 bullet -->
```

```bash
gz sim --physics-engine gz-physics-bullet world.sdf
```

## 那 MuJoCo 呢？現況與可行性

gz-physics 的設計允許第三方實作新引擎外掛（官方教學：[Use a custom engine](https://gazebosim.org/api/physics/9/usecustomengine.html)）。針對 MuJoCo：

- 社群曾有**實驗性的 MuJoCo 外掛 MVP**（[ign-physics#299](https://github.com/ignitionrobotics/ign-physics/issues/299)，traversaro 的分支），展示了固定基座雙擺在 Gazebo 中由 MuJoCo 驅動；
- 但該工作**停留在原型階段，未合併、未維護**，不等於可用的官方支援。

結論：**現階段在 Gazebo 中「換用 MuJoCo」需要自己實作 gz-physics 外掛**，屬於進階研究題目，不是開箱即用的功能。若你的目標只是「同一個機器人在兩個模擬器跑」，走模型互通路線實際得多。

## 模型互通：URDF ↔ SDF ↔ MJCF

```
        URDF ──(sdformat 轉換)──▶ SDF ──▶ Gazebo (DART/Bullet)
         │
         └──(MuJoCo 內建解析)──▶ mjModel ──▶ MuJoCo
```

- Gazebo 原生格式是 **SDF**；`sdformat` 工具可把 URDF 轉成 SDF（`gz sdf -p robot.urdf > robot.sdf`）。
- MuJoCo 直接吃 URDF（見 [04｜URDF 匯入](../03-urdf-import/01-import-urdf.md)）。
- 因此 **URDF 是兩邊共同的交換格式**：維護一份 URDF，Gazebo 端轉 SDF，MuJoCo 端直接載入或再轉 MJCF。

> 譯註：也可以用本專案的 `scripts/load_urdf.py` 驗證同一份 URDF 在 MuJoCo 的行為，再到 Gazebo 用 SDF 版跑感測器與場景。

## 自製 MuJoCo gz-physics 外掛（進階，概念指引）

若確實要在 Gazebo 中跑 MuJoCo，路徑是：

1. 參考 `gz-physics/examples/simple_plugin` 建立外掛骨架；
2. 實作 gz-physics 的 Feature 介面（`SimulationFeatures`、`SDFFeatures`、關節/碰撞相關 Feature）；
3. 在外掛內部把請求轉成 MJCF/`mjSpec` 建模型，每步呼叫 `mj_step` 並回填狀態；
4. 以 `--physics-engine` 或 SDF `<physics type="...">` 載入。

工作量與限制需先有心理準備：gz-physics 的 Feature 集合龐大，MuJoCo 的廣義座標資料結構與 DART 式的 entity 抽象之間的映射（尤其接觸點回報、感測器）是最難的部分 — 這正是當年 MVP 停住的地方。

## 常見錯誤與除錯

- **以為有現成 MuJoCo 外掛可裝**：沒有；網路上的教學多指向未維護的實驗分支。
- **URDF 轉 SDF 後行為不同**：檢查 `<gazebo>` 擴充標籤、慣性、以及 DART 與 MuJoCo 的接觸參數差異。
- **Gazebo 版本**：外掛 API 隨 gz-physics 大版本變動，教學連結請對應你安裝的 Gazebo 發行版（Harmonic = gz-physics 7/8 等）。

## 延伸閱讀

- [Gazebo Sim: Physics engines](https://gazebosim.org/api/sim/9/physics.html)
- [Switching physics engines](https://gazebosim.org/api/physics/6/switchphysicsengines.html)
- [Use a custom engine with Gazebo Physics](https://gazebosim.org/api/physics/9/usecustomengine.html)
- [gz-physics](https://github.com/gazebosim/gz-physics) 與 [MuJoCo 外掛實驗 issue #299](https://github.com/ignitionrobotics/ign-physics/issues/299)
