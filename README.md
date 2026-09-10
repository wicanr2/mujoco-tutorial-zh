# MuJoCo 繁體中文教學

以 MuJoCo 物理引擎為核心的繁體中文教學專案。從官方文件出發，涵蓋 MJCF 建模、Python 程式設計、URDF 匯入、Isaac Sim / Gazebo 整合、RL 訓練，以及一系列的 **AMR 叉車實驗**（含 Blender 建模、錄影與 6DOF 軌跡記錄）。

協作規範見 [AGENTS.md](AGENTS.md)；成果摘要見 [REPORT.md](REPORT.md)。

## 特色

- **全部範例都實際執行過**：每篇教學標註測試環境與實測輸出；不成功的實驗（例如 CPU 版 SAC）也如實記錄。
- **除錯導向**：各章「除錯紀錄」收錄開發時真實踩過的坑（摩擦合併規則、mesh 主軸對齊、IK 工作空間、離散 Jacobian…）。
- **真實模型**：AMR 系列使用 TB3 實驗的 MR1533 舵輪叉車 mesh 與 Blender 程序化建模的棧板，含錄影與軌跡記錄。

## 教學目錄

完整索引在 [docs/README.md](docs/README.md)。

**基礎**：01 導論安裝｜02 MJCF 建模｜03 程式設計入門｜04 URDF 匯入｜05 Isaac Sim 整合｜06 Gazebo 整合

**程式設計 / RL**：07 五個 Python 範例｜08 ARS 手寫 RL｜09 Gymnasium + PPO｜10 車桿 swing-up（能量整形+LQR）｜11 Viewer 與離屏渲染｜12 SAC 對照實驗（CPU vs GPU）

**AMR 叉車**：13 叉車建模與牙叉控制｜14 車載手臂 IK｜15 六軸 6D IK｜16 棧板材質摩擦｜17 門架前傾邊界｜18 MR1533 真實 mesh 叉車｜19 Blender 棧板建模｜20 真實模型重跑實驗｜21 y 向 reach 滑台車型｜22 取放任務（錄影+軌跡）｜23 連續動作實驗（錄影）

## 快速開始

```bash
python3 -m venv .venv
.venv/bin/pip install mujoco numpy scipy pillow
.venv/bin/python scripts/hello_mujoco.py     # 第一個模擬
```

RL 範例需要額外套件：

```bash
.venv/bin/pip install gymnasium stable-baselines3
.venv/bin/python scripts/ex_rl_swingup.py    # ARS 訓練單擺（約 21 秒）
```

錄影與圖表（無顯示器環境）：

```bash
.venv/bin/pip install imageio imageio-ffmpeg matplotlib
MUJOCO_GL=osmesa .venv/bin/python scripts/ex_fork_cyclic.py   # 產生 runs/fork_cyclic.mp4
```

Blender 建模腳本（需本機 Blender 4.x）：

```bash
blender --background --python scripts/make_pallet_blender.py -- "$PWD"
```

## 目錄結構

```
docs/       教學文件（01–23 篇）與圖片（assets/）
models/     MJCF / URDF 模型與 mesh（MR1533、Blender 棧板、y-reach 滑台）
scripts/    所有可執行範例（每支對應教學章節）
runs/       實驗輸出：影片、CSV、軌跡圖（見 runs/README.md）
sources/    資料出處登記（SOURCES.md）
AGENTS.md   專案協作規範
REPORT.md   成果報告
```

## 致謝與出處

- [MuJoCo 官方文件](https://mujoco.readthedocs.io/en/stable/)（各章翻譯與摘譯的主要來源）
- MR1533 叉車 mesh 取自本機 TB3 實驗資產（Blender 4.2 建模）
- RL 章節使用 [Stable-Baselines3](https://stable-baselines3.readthedocs.io/) 與 [Gymnasium](https://gymnasium.farama.org/)
