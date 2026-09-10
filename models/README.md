# 模型清單（models/）

MJCF、URDF 與 mesh。每個模型對應的教學章節如下，`_template` 結尾的檔案含字串佔位符
（材質、摩擦係數），由腳本替換後再載入。

## 基礎教學

| 檔案 | 章節 | 內容 |
| --- | --- | --- |
| `hello.xml` | [01](../docs/00-intro/01-what-is-mujoco.md) | 一顆球加地板，最小可跑模型 |
| `double_pendulum.xml` | [02](../docs/01-basics/01-mjcf-basics.md) | 雙擺，示範 body / geom / joint |
| `pendulum_actuated.xml` | [03](../docs/02-programming/01-simulation-loop.md) | 單擺加 motor，PD 控制範例 |
| `two_link_arm.urdf` | [04](../docs/03-urdf-import/01-import-urdf.md) | 二連桿手臂，URDF 匯入來源 |
| `two_link_arm_converted.xml` | [04](../docs/03-urdf-import/01-import-urdf.md) | 上者經 `mj_saveLastXML` 轉存的 MJCF（腳本產生） |
| `touch_ball.xml` | [07](../docs/02-programming/02-more-examples.md) | 觸覺感測器與加速度計 |
| `servo_pendulum.xml` | [07](../docs/02-programming/02-more-examples.md) | position 伺服追蹤正弦軌跡 |
| `spec_generated.xml` | [07](../docs/02-programming/02-more-examples.md) | mjSpec 程序化建模的輸出（腳本產生） |
| `pendulum_swingup.xml` | [08](../docs/02-programming/03-rl-swingup.md)、[09](../docs/02-programming/04-ppo-gymnasium.md)、[12](../docs/02-programming/07-sac.md) | RL swing-up 任務環境 |
| `cartpole_swingup.xml` | [10](../docs/02-programming/05-cartpole-swingup.md)、[11](../docs/02-programming/06-viewer-rendering.md) | 車桿，能量整形 + LQR |

## AMR 實驗

| 檔案 | 章節 | 內容 |
| --- | --- | --- |
| `amr_forklift.xml` | [13](../docs/06-amr/01-amr-forklift.md) | 盒子堆的簡化叉車：平面底盤 + 升降 + 側移 |
| `mobile_manipulator.xml` | [14](../docs/06-amr/02-mobile-manipulator.md) | 搬運車 + 三軸手臂 |
| `arm6.xml` | [15](../docs/06-amr/03-ik-6d.md) | 六軸手臂，6D IK 用 |
| `amr_pallet_template.xml` | [16](../docs/06-amr/04-pallet-materials.md) | 叉車 + 棧板，材質與摩擦以佔位符替換 |
| `amr_tilt_template.xml` | [17](../docs/06-amr/05-tilt-boundary.md) | 加門架前傾自由度，掃滑落邊界 |
| `mr1533_forklift.xml` | [18](../docs/06-amr/06-mr1533-mesh.md) | MR1533 真實 mesh 叉車（TB3 資產） |
| `mr1533_pallet_template.xml` | [19](../docs/06-amr/07-blender-pallet.md)、[20](../docs/06-amr/08-rerun-real-model.md) | MR1533 + Blender 棧板 |
| `mr1533_yreach.xml` | [21](../docs/06-amr/09-yreach.md)–[23](../docs/06-amr/11-fork-cyclic.md) | MR1533 加 y 向 reach 滑台 |
| `mr1533_steer.xml` | [24](../docs/06-amr/12-steer-reach-xz.md)、[25](../docs/06-amr/13-reach-xyz.md)、[27](../docs/06-amr/15-steer-loop.md) | 真實舵輪底盤（前雙固定輪 + 後舵輪），timestep 0.0005 |
| `mm_gripper.xml` | [26](../docs/06-amr/14-gripper-ab.md) | 搬運車 + 三軸手臂 + 二指夾爪 + A/B 兩張桌 |

## mesh

| 路徑 | 來源 | 說明 |
| --- | --- | --- |
| `meshes/mr1533/*.obj` | 本機 TB3 實驗資產 | MR1533 車體零件，視覺用（碰撞另以簡化幾何處理） |
| `meshes/pallet_wood.stl`、`meshes/pallet_plastic.stl` | `scripts/make_pallet_blender.py` | Blender 程序化建模的歐規棧板 |
| `meshes/yreach_carriage.stl` | `scripts/make_yreach_blender.py` | y 向 reach 側移滑台 |

Blender 產生的 STL 不帶材質，顏色由 MJCF 的 `rgba` 指定。mesh 的主軸會被 MuJoCo 編譯器
重新對齊，零件要先 join 成單一 mesh 再匯出，細節見 [19 章](../docs/06-amr/07-blender-pallet.md)。
