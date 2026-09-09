# MuJoCo 繁體中文教學

以 MuJoCo 物理引擎為核心的繁體中文教學文件。協作規範見 [AGENTS.md](../AGENTS.md)。

## 章節索引

- **00 導論**
  - [01｜MuJoCo 是什麼？導論與安裝](00-intro/01-what-is-mujoco.md)
- **01 MJCF 建模基礎**
  - [02｜MJCF 建模基礎：body、geom、joint、defaults](01-basics/01-mjcf-basics.md)
- **02 Programming：Python/C API**
  - [03｜程式設計入門：模擬迴圈、命名慣例與致動器控制](02-programming/01-simulation-loop.md)
  - [07｜更多 Python 範例：感測器、伺服、Keyframe、平行取樣、mjSpec](02-programming/02-more-examples.md)
  - [08｜RL 訓練範例：單擺 Swing-up（ARS 隨機搜索）](02-programming/03-rl-swingup.md)
  - [09｜接軌標準 RL 工具鏈：Gymnasium + Stable-Baselines3 PPO](02-programming/04-ppo-gymnasium.md)
  - [10｜進階任務：車桿 Swing-up（能量整形 + LQR）](02-programming/05-cartpole-swingup.md)
  - [11｜視覺化：互動式 Viewer 與離屏渲染](02-programming/06-viewer-rendering.md)
  - [12｜SAC：Off-policy 對照實驗](02-programming/07-sac.md)
- **03 URDF 匯入**
  - [04｜URDF 模型匯入 MuJoCo](03-urdf-import/01-import-urdf.md)
- **04 Isaac Sim 中使用/替換 MuJoCo**
  - [05｜Isaac Sim 與 MuJoCo：模型互通與工作流程](04-isaac-sim/01-isaac-sim-mujoco.md)
- **05 Gazebo 中使用/替換 MuJoCo**
  - [06｜Gazebo 與 MuJoCo：物理引擎外掛機制與模型互通](05-gazebo/01-gazebo-mujoco.md)
- **06 AMR 實驗**
  - [13｜AMR 實驗（一）：叉車建模與牙叉控制](06-amr/01-amr-forklift.md)
  - [14｜AMR 實驗（二）：搬運車上的機械手臂（逆運動學 IK）](06-amr/02-mobile-manipulator.md)
  - [15｜六軸手臂的 6D IK：位置與姿態同時控制](06-amr/03-ik-6d.md)
  - [16｜棧板材質實驗：木頭 vs 塑膠（20 kg 載重摩擦測試）](06-amr/04-pallet-materials.md)
  - [17｜門架前傾實驗：棧板滑落邊界（木頭 vs 塑膠、重心偏移）](06-amr/05-tilt-boundary.md)

## 附錄

- [中英術語對照表](glossary.md)
- 資料來源登記：../sources/SOURCES.md
