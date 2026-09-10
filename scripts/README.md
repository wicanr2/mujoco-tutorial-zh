# 範例程式清單（scripts/）

全部在 repo 根目錄執行（模型路徑是相對路徑）：

```bash
.venv/bin/python scripts/hello_mujoco.py
MUJOCO_GL=osmesa .venv/bin/python scripts/ex_fork_cyclic.py    # 需要渲染的加這個環境變數
```

「輸出」欄標示會寫檔的腳本；沒標的只印到 stdout。

## 基礎與程式設計

| 腳本 | 章節 | 內容 | 輸出 |
| --- | --- | --- | --- |
| `hello_mujoco.py` | [01](../docs/00-intro/01-what-is-mujoco.md) | 最小模擬迴圈 | |
| `run_pendulum.py` | [02](../docs/01-basics/01-mjcf-basics.md) | 雙擺自由擺盪 | |
| `pd_control.py` | [03](../docs/02-programming/01-simulation-loop.md) | PD 控制與穩態誤差 | |
| `load_urdf.py` | [04](../docs/03-urdf-import/01-import-urdf.md) | 載入 URDF 並轉存 MJCF | `models/two_link_arm_converted.xml` |
| `ex_sensors.py` | [07](../docs/02-programming/02-more-examples.md) | 觸覺感測器與加速度計 | |
| `ex_position_servo.py` | [07](../docs/02-programming/02-more-examples.md) | position 伺服追蹤正弦軌跡 | |
| `ex_keyframe.py` | [07](../docs/02-programming/02-more-examples.md) | keyframe 儲存與重置狀態 | |
| `ex_mjspec.py` | [07](../docs/02-programming/02-more-examples.md) | mjSpec 程序化建模 | `models/spec_generated.xml` |
| `ex_parallel_rollout.py` | [07](../docs/02-programming/02-more-examples.md) | 多執行緒共用 mjModel 取樣 | |
| `ex_cartpole_swingup.py` | [10](../docs/02-programming/05-cartpole-swingup.md) | 能量整形盪起 + LQR 穩定 | |
| `ex_viewer.py` | [11](../docs/02-programming/06-viewer-rendering.md) | 離屏渲染四個影格 | `docs/assets/viewer_frame*.png` |

## 強化學習

| 腳本 | 章節 | 內容 | 輸出 |
| --- | --- | --- | --- |
| `ex_rl_swingup.py` | [08](../docs/02-programming/03-rl-swingup.md) | 手寫 ARS，約 21 秒 | `policies/swingup_policy.npy` |
| `ex_rl_ppo.py` | [09](../docs/02-programming/04-ppo-gymnasium.md) | Gymnasium Env + SB3 PPO，約 6 分鐘 | `policies/swingup_ppo.zip` |
| `ex_rl_sac.py` | [12](../docs/02-programming/07-sac.md) | SAC on CPU（如實記錄未收斂） | `policies/swingup_sac.zip` |
| `ex_rl_sac_gpu.py` | [12](../docs/02-programming/07-sac.md) | SAC 的 GPU 版，zoo 超參 | `policies/swingup_sac_gpu.zip` |

`ex_rl_sac_gpu.py` 需要 CUDA 環境，在只有 CPU 的機器上跑會很慢。

## AMR 實驗

| 腳本 | 章節 | 內容 | 輸出 |
| --- | --- | --- | --- |
| `ex_forklift.py` | [13](../docs/06-amr/01-amr-forklift.md) | 簡化叉車取貨五階段 | |
| `ex_mobile_manipulator.py` | [14](../docs/06-amr/02-mobile-manipulator.md) | 底盤 P 控制 + 三軸手臂 IK | |
| `ex_ik6d.py` | [15](../docs/06-amr/03-ik-6d.md) | 六軸手臂 6D IK | |
| `ex_pallet_materials.py` | [16](../docs/06-amr/04-pallet-materials.md) | 木頭 vs 塑膠棧板急側移 | |
| `ex_tilt_boundary.py` | [17](../docs/06-amr/05-tilt-boundary.md) | 門架前傾掃參找滑動角 | |
| `ex_mr1533.py` | [18](../docs/06-amr/06-mr1533-mesh.md) | MR1533 mesh 車動作驗證 | |
| `ex_mesh_pallet.py` | [19](../docs/06-amr/07-blender-pallet.md) | Blender 棧板載重驗證 | |
| `ex_rerun_materials.py` | [20](../docs/06-amr/08-rerun-real-model.md) | 在真實車上重跑材質實驗 | |
| `ex_yreach.py` | [21](../docs/06-amr/09-yreach.md) | y-reach 滑台動作驗證 | |
| `ex_yreach_mission.py` | [22](../docs/06-amr/10-yreach-mission.md) | 完整取放任務 | `runs/mission.*` |
| `ex_fork_cyclic.py` | [23](../docs/06-amr/11-fork-cyclic.md) | 載貨連續升降與側移 | `runs/fork_cyclic.*` |
| `ex_reach_xz.py` | [24](../docs/06-amr/12-steer-reach-xz.md) | 舵輪底盤貨架取放 X/Z | `runs/reach_xz.*` |
| `ex_reach_xyz.py` | [25](../docs/06-amr/13-reach-xyz.md) | 貨架取放 X/Y/Z 全軸 | `runs/reach_xyz.*` |
| `ex_gripper_ab.py` | [26](../docs/06-amr/14-gripper-ab.md) | 夾爪 A 取 B 放 | `runs/gripper.*` |
| `ex_loop_steer.py` | [27](../docs/06-amr/15-steer-loop.md) | 舵輪繞圈 waypoint 追蹤 | `runs/loop.*` |

## 建模與工具

| 腳本 | 用途 | 需要 |
| --- | --- | --- |
| `make_pallet_blender.py` | 程序化建木 / 塑膠棧板，匯出 STL 並渲染 | Blender 4.x |
| `make_yreach_blender.py` | 程序化建 y 向 reach 滑台，匯出 STL | Blender 4.x |
| `render_mr1533_blender.py` | 用原始 .blend 檔渲染 MR1533 展示圖 | Blender 4.x + TB3 資產 |
| `make_strips.py` | 把實驗過程做成多幀圖條 | MuJoCo 離屏渲染 |
| `verify_examples.sh` | 逐支重跑範例並記錄 exit code 與耗時 | — |
| `check_docs.py` | 靜態稽核：絕對路徑、連結、模型可載入、章節結構、清單與數字一致性 | mujoco |

Blender 腳本的用法（`--` 後面接 repo 根目錄）：

```bash
blender --background --python scripts/make_pallet_blender.py -- "$PWD"
```
