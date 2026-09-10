# 02｜MJCF 建模基礎：body、geom、joint、defaults

> 原文來源：[Modeling — MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html)（stable，MuJoCo 3.x）
>
> 擷取日期：2026-09-09

## 學習目標

- 理解 MJCF 的運動學樹（kinematic tree）結構
- 學會使用 `body`、`geom`、`joint`、`site`
- 理解 MJCF 獨特的 defaults 預設值機制
- 建立一個雙擺（double pendulum）並模擬

## 前置知識

- [01｜MuJoCo 是什麼？導論與安裝](../00-intro/01-what-is-mujoco.md)

## MJCF 概觀

MJCF 是 MuJoCo 的原生模型格式，一種描述複雜動力系統的 XML 語言。它可以被視為「建模格式與程式語言的混合體」：MJCF 檔會經過內建**編譯器**轉成 `mjModel`，且依模型設計自動觸發許多編譯期的計算（例如由 geom 推斷慣性）。

### 運動學樹

MJCF 的主體是由巢狀 `<body>` 元素構成的 XML 樹，頂層是特殊的 `<worldbody>`。這與 URDF 很不同：URDF 是先建立一堆 link 再用 joint 指定 parent/child 連起來；**在 MJCF 中，子 body 在 XML 層級上就是父 body 的子元素**。

關鍵觀念：

- 定義在 body 內的 `<joint>` **不是用來連接父子 body，而是建立兩者之間的運動自由度（degree of freedom）**。
- body 內若**沒有任何 joint，該 body 會被焊死（weld）在父 body 上**。
- 一個 body 可以放**多個 joint**，不需要像 URDF 那樣用虛擬 link 組合複合關節。例如兩個 slide + 一個 hinge 就能做平面運動。
- `<geom>`、`<site>`、`<camera>`、`<light>` 定義在 body 內時，會固定在該 body 的局部座標系上，跟著它動。

### 常用元素速查

| 元素 | 角色 | 說明 |
| --- | --- | --- |
| `<body>` | 動力學實體 | 有質量與慣性，構成運動學樹 |
| `<geom>` | 幾何體 | 附在 body 上，用於碰撞與渲染；編譯器由它推斷質量/慣性 |
| `<joint>` | 關節 | 在 body 與其父之間建立自由度；類型：`free`、`ball`、`slide`、`hinge` |
| `<site>` | 站點 | 無質量、不碰撞的參考座標系，用於感測器、肌腱端點、標記位置 |
| `<camera>` / `<light>` | 相機 / 光源 | 固定在 body 局部座標系上 |

geom 常見 `type`：`plane`、`sphere`、`capsule`、`box`、`cylinder`、`ellipsoid`、`mesh`。

### 座標與方向

- 所有位置/方向都以**父 body 的局部座標**表示（`pos="x y z"`）。
- 方向可用五種互斥屬性指定：`quat`（四元數，預設 `1 0 0 0`）、`axisangle`、`euler`、`xyaxes`、`zaxis`。內部一律轉成單位四元數。
- `<compiler angle="degree|radian"/>` 決定 MJCF 中角度的單位（編譯後一律是弧度），預設是 degree。

## defaults 預設值機制

MJCF 有一套類似 CSS 的預設值機制，讓模型檔保持簡短：

```xml
<mujoco>
  <default class="main">
    <geom rgba="1 0 0 1"/>
    <default class="sub">
      <geom rgba="0 1 0 1"/>
    </default>
  </default>
  ...
</mujoco>
```

規則重點：

1. defaults class 可無限巢狀，子 class 繼承父 class 的所有屬性值，可再覆寫。
2. 頂層 class 未命名時叫做 `main`；元素沒指定 class 時使用頂層 class。
3. body 上的 `childclass` 決定其子孫元素使用哪個 class；元素自身的 `class` 屬性優先權最高。
4. 某些屬性（如 body 慣性）有特殊的「未定義」狀態，會讓編譯器自動推斷；一旦在 class 中被定義就無法回到未定義。

> 譯註：善用 defaults 是寫出簡潔 MJCF 的關鍵 — 例如把所有關節的 `damping` 寫在一個 class 裡，改一處即全模型生效。

## 完整範例：雙擺

`models/double_pendulum.xml`：

```xml
<mujoco>
  <compiler angle="radian"/>

  <default>
    <joint type="hinge" axis="0 1 0" damping="0.1"/>
    <geom type="capsule" size="0.02" rgba="0.8 0.2 0.2 1"/>
  </default>

  <worldbody>
    <light pos="0 -1 2"/>
    <body pos="0 0 1">
      <joint name="hinge1"/>
      <geom fromto="0 0 0 0 0 -0.4"/>
      <body pos="0 0 -0.4">
        <joint name="hinge2"/>
        <geom fromto="0 0 0 0 0 -0.4"/>
      </body>
    </body>
  </worldbody>
</mujoco>
```

注意：

- defaults 一次設定所有 joint 的類型、轉軸與阻尼（damping），以及所有 geom 的形狀與顏色。
- capsule 用 `fromto="x1 y1 z1 x2 y2 z2"` 指定兩端點，長度自動決定。
- 第一節 body 內只有一個 hinge joint，所以它只能繞 y 軸轉動。

`scripts/run_pendulum.py`：

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("models/double_pendulum.xml")
data = mujoco.MjData(model)

# 初始姿態：把兩個關節各拉開 1 rad
data.qpos[:] = [1.0, 1.0]

for _ in range(500):  # 500 步 × 0.002s = 1 秒
    mujoco.mj_step(model, data)

print("關節角度 qpos =", np.round(data.qpos, 3))
print("關節角速度 qvel =", np.round(data.qvel, 3))
```

執行：

```bash
python scripts/run_pendulum.py
```

雙擺是混沌系統，放開後會持續擺盪；`qpos` 是各關節角度、`qvel` 是角速度（這正是「廣義座標」：狀態直接用關節量表示，而不是每個 body 的 6 自由度卡氏座標）。

> 測試環境：Linux、MuJoCo 3.12.0、Python 3.12.3；實測輸出 `qpos = [-0.649 -0.385]`、`qvel = [2.251 0.812]`（混沌系統，小數位可能因版本略有差異）。

想互動觀看，可執行：

```bash
python -m mujoco.viewer --mjcf=models/double_pendulum.xml
```

## 常見錯誤與除錯

- **body 不會動**：忘了放 joint — 沒有 joint 的 body 會被焊在父 body 上。
- **方向不對**：檢查 `compiler angle` 單位，或改用 `fromto` / `zaxis` 這類較直覺的指定方式。
- **attribute 沒生效**：檢查是否被 defaults class 或 `childclass` 覆寫。
- **XML 解析錯誤**：錯誤訊息會給出行號；`<freejoint/>` 只能放在 `<worldbody>` 的直接子 body 中。

## 延伸閱讀

- [Modeling — MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [XML Reference（完整元素與屬性手冊）](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
