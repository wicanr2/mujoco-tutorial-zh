# 資料來源登記（SOURCES）

格式：章節 | 來源 URL | 擷取日期 | 對應版本 | 備註

| 教學章節 | 來源 URL | 擷取日期 | 對應版本 | 備註 |
| --- | --- | --- | --- | --- |
| docs/00-intro | https://mujoco.readthedocs.io/en/stable/overview.html | 2026-09-09 | stable (3.x) | Introduction、Key features、Model instances |
| docs/00-intro | https://github.com/google-deepmind/mujoco | 2026-09-09 | latest | 安裝方式 pip install mujoco |
| docs/01-basics | https://mujoco.readthedocs.io/en/stable/modeling.html | 2026-09-09 | stable (3.x) | Kinematic tree、Default settings、Coordinate frames |
| docs/02-programming | https://mujoco.readthedocs.io/en/stable/programming/ | 2026-09-09 | stable (3.x) | 架構、命名慣例、mjModel/mjData |
| docs/03-urdf-import | https://mujoco.readthedocs.io/en/stable/modeling.html#urdf-extensions | 2026-09-09 | stable (3.x) | `<mujoco>` 擴充區段、URDF 預設值差異、建議工作流程 |
| docs/04-isaac-sim | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_mjcf.html | 2026-09-09 | Isaac Sim 5.1 | MJCF Importer 擴充 |
| docs/04-isaac-sim | https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.asset.importer.mjcf/docs/index.html | 2026-09-09 | latest | MJCFImporter Python API |
| docs/04-isaac-sim | https://github.com/isaac-sim/IsaacSim/discussions/160 | 2026-09-09 | Isaac Sim 6.0.0+ | MJCF 轉換器以 mujoco-usd-converter 重寫 |
| docs/04-isaac-sim | https://github.com/newton-physics/mujoco-usd-converter | 2026-09-10 | 0.5.0（Alpha） | MJCF → OpenUSD 轉換器本體，Apache-2.0，可獨立使用 |
| docs/04-isaac-sim | https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html | 2026-09-09 | main | Isaac Lab 匯入 MJCF 範例（Unitree H1） |
| docs/05-gazebo | https://gazebosim.org/api/sim/9/physics.html | 2026-09-09 | Gazebo Sim 9 | 物理引擎外掛架構，預設 DART |
| docs/05-gazebo | https://gazebosim.org/api/physics/6/switchphysicsengines.html | 2026-09-09 | gz-physics 6 | 執行期切換引擎（DART/Bullet） |
| docs/05-gazebo | https://gazebosim.org/api/physics/9/usecustomengine.html | 2026-09-09 | gz-physics 9 | 客製引擎外掛教學 |
| docs/05-gazebo | https://github.com/gazebosim/gz-physics/issues/299 | 2026-09-10 | 仍 open | MuJoCo gz-physics 外掛的功能分工與進度追蹤 |
| docs/05-gazebo | https://github.com/gazebosim/gz-physics/pull/811 | 2026-09-10 | 已於 2026-03-14 合併 | MuJoCo 外掛初步實作（21 檔、+2158 行） |
| docs/05-gazebo | https://github.com/gazebosim/gz-physics/blob/main/mujoco/README.md | 2026-09-10 | main 分支 | 外掛已實作的 Feature 與 TODO 清單 |
| docs/02-programming/02 | https://mujoco.readthedocs.io/en/stable/programming/modeledit.html | 2026-09-09 | stable (3.x) | mjSpec 程序化建模 |
| docs/02-programming/03 | https://arxiv.org/abs/1803.07055 | 2026-09-09 | 2018 論文 | ARS 隨機搜索演算法 |
| docs/02-programming/04 | https://stable-baselines3.readthedocs.io/ | 2026-09-09 | sb3 2.9.0 | PPO 實作 |
| docs/02-programming/04 | https://gymnasium.farama.org/ | 2026-09-09 | gymnasium 1.3.0 | 標準 Env 介面 |
| docs/02-programming/04 | https://arxiv.org/abs/1707.06347 | 2026-09-09 | 2017 論文 | PPO 演算法 |
| docs/02-programming/05 | https://mujoco.readthedocs.io/en/stable/APIreference/ | 2026-09-09 | stable (3.x) | mjd_transitionFD 數值線性化 |
| docs/02-programming/06 | https://mujoco.readthedocs.io/en/stable/python.html | 2026-09-09 | stable (3.x) | viewer、Renderer、MUJOCO_GL |
| docs/02-programming/07 | https://stable-baselines3.readthedocs.io/en/master/modules/sac.html | 2026-09-09 | sb3 2.9.0 | SAC 實作與超參 |
| docs/02-programming/07 | https://arxiv.org/abs/1801.01290 | 2026-09-09 | 2018 論文 | SAC 演算法 |
| docs/06-amr | https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator | 2026-09-09 | stable (3.x) | velocity/position 致動器 |
| docs/06-amr | https://mujoco.readthedocs.io/en/stable/APIreference/ | 2026-09-09 | stable (3.x) | mj_jacSite 雅可比 |
| docs/06-amr | https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh | 2026-09-10 | stable (3.x) | mesh 主軸對齊、refpos/refquat |
| docs/06-amr/12 | https://mujoco.readthedocs.io/en/stable/XMLreference.html#contact-exclude | 2026-09-10 | stable (3.x) | contact/exclude 排除機構內部碰撞 |
| docs/06-amr/14 | https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-weld | 2026-09-10 | stable (3.x) | weld equality constraint 抓取 |
| docs/06-amr/15 | https://mujoco.readthedocs.io/en/stable/XMLreference.html#option | 2026-09-10 | stable (3.x) | timestep 與接觸求解穩定性 |
| docs/06-amr/06,09,12 | 本機資產 `~/tmp2/TB3/assets/mr1533_light/`（MR1533 三輪舵輪叉車，OBJ mesh + URDF） | 2026-09-10 | — | 內部實驗資產，非公開來源 |
| docs/06-amr/07,09 | https://docs.blender.org/api/4.2/ | 2026-09-10 | Blender 4.2 LTS | headless 程序化建模 Python API |
| （通用模型庫）| https://github.com/google-deepmind/mujoco_menagerie | 2026-09-09 | latest | 官方模型庫，URDF/MJCF 範例來源 |
