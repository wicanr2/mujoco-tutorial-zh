# 04｜URDF 模型匯入 MuJoCo

> 原文來源：[Modeling — URDF extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#urdf-extensions)、[XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)（stable，MuJoCo 3.x）
> 擷取日期：2026-09-09

## 學習目標

- 直接在 MuJoCo 中載入 URDF
- 學會用 `<mujoco>` 擴充區段補上 URDF 缺少的資訊
- 了解 URDF → MJCF 的建議工作流程
- 認識匯入時的常見陷阱

## 前置知識

- [02｜MJCF 建模基礎](../01-basics/01-mjcf-basics.md)、URDF 基本結構（link / joint）

## 為什麼可以直接載入 URDF？

URDF 是機器人領域常見的 XML 模型格式。雖然 URDF 只能表達 MJCF 的一部分功能，MuJoCo 仍**內建 URDF 解析器** — `MjModel.from_xml_path()` 給它 `.urdf` 檔就能載入，不需要額外工具。

### 觀念差異對照

| URDF | MJCF |
| --- | --- |
| link 各自獨立，joint 用 parent/child 連接 | body 巢狀構成運動學樹 |
| 一個 joint 連接兩個 link | joint 定義在 body 內，建立與父 body 間的自由度 |
| `<visual>` / `<collision>` 分開定義 | 通常只用 `<geom>`（可由 `discardvisual` 控制） |
| 無致動器、無場景元素 | actuator、tendon、sensor、light…都在模型檔裡 |
| 沒有 fixed joint 以外的焊接概念 | 無 joint 的 body 自動焊在父 body 上 |

## `<mujoco>` 擴充區段

MuJoCo 允許在 URDF 的 `<robot>` 底下加一個自訂的 `<mujoco>` 元素，裡面可以放 `compiler`、`option`、`size`，語法與 MJCF 相同：

```xml
<robot name="my_robot">
  <mujoco>
    <compiler meshdir="../meshes/" balanceinertia="true" discardvisual="false"/>
    <option gravity="0 0 -9.81" timestep="0.002"/>
  </mujoco>
  ...
</robot>
```

常用屬性：

- **`meshdir`**：指定 mesh 檔的搜尋目錄 — URDF 沒有這個概念，mesh 路徑問題幾乎都靠它解決。
- **`balanceinertia`**：許多現成 URDF 的慣性參數不符合物理（編譯器會拒絕），開啟後自動修正。
- **`discardvisual`**：URDF 的預設是 `false`（visual 也保留為 geom）；設 `true` 只留 collision。
- **預設值差異**：`strippath`、`angle`、`fusestatic`、`discardvisual` 在 URDF 模式下的預設值與 MJCF 不同（例如 URDF 預設 `angle="radian"`）。

> ⚠️ **陷阱**：MJCF 會用 XML schema 檢查，URDF（含內嵌的 `<mujoco>` 區段）**不會**。屬性名打錯字會被靜靜忽略，不報錯 — 行為不如預期時先檢查拼字。

## 完整範例：二連桿機械臂

`models/two_link_arm.urdf` 是一個手寫的二連桿手臂（含 `<mujoco>` 擴充區段、inertial / visual / collision 三件套）。載入與模擬程式 `scripts/load_urdf.py`：

```python
import mujoco
import numpy as np

# 直接載入 URDF：MuJoCo 內建支援
model = mujoco.MjModel.from_xml_path("models/two_link_arm.urdf")
data = mujoco.MjData(model)

print("自由度 nv =", model.nv, "| 關節數 njnt =", model.njnt)

# 拉離平衡點（垂下為 0），在重力下自然擺動 1 秒
data.qpos[:] = [1.0, 0.5]
for _ in range(500):
    mujoco.mj_step(model, data)
print("1 秒後 qpos =", np.round(data.qpos, 3))

# 將載入的模型轉存成 MJCF，之後可直接以 MJCF 編輯維護
mujoco.mj_saveLastXML("models/two_link_arm_converted.xml", model)
```

執行：

```bash
python scripts/load_urdf.py
```

輸出（實測）：

```
自由度 nv = 2 | 關節數 njnt = 2
關節名稱  = ['joint1', 'joint2']
1 秒後 qpos = [-0.637 -0.802]
已轉存 MJCF → models/two_link_arm_converted.xml
```

> 測試環境：Linux、MuJoCo 3.12.0、Python 3.12.3。轉存產生的 `models/two_link_arm_converted.xml` 已一併收錄供對照。

## 官方建議的工作流程

擴充區段能讓 URDF 更好用，但仍受限於 URDF 的表達能力。若要完整發揮 MuJoCo，官方建議：

1. 在 URDF 中加入必要的 `<mujoco>` 擴充；
2. 載入後**轉存成 MJCF**（如上方 `mj_saveLastXML`）；
3. 之後只維護 MJCF，需要新增元素時優先用 `<include>` 組織檔案；
4. 若 URDF 上游有更新，重新轉一次即可。

實務上 URDF 通常是靜態的、MJCF 才是常被編輯的，所以「轉一次之後只管 MJCF」最常見。

## 常見錯誤與除錯

- **mesh 找不到**：用 `<mujoco><compiler meshdir="..."/></mujoco>` 指定目錄；URDF 常見的 `package://` 路徑要改成相對路徑。
- **編譯器拒絕載入、報慣性錯誤**：加上 `balanceinertia="true"`；或檢查 `<inertial>` 的 inertia 是否正定。
- **fixed joint 的 link 消失了**：這是正常的 — 固定連接的 link 會被 `fusestatic`/焊接合併。
- **行為怪但沒有錯誤訊息**：URDF 不做 schema 檢查，優先懷疑屬性拼字。
- **需要真實機器人模型**：直接參考官方模型庫 [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)，裡面的模型（Franka、UR5e、Unitree…）都是已調校好的 MJCF/URDF 範例。

## 延伸閱讀

- [URDF extensions — MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html#urdf-extensions)
- [XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
