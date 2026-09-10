# 13｜AMR 實驗（一）：叉車建模與牙叉控制

> 參考來源：[Modeling — MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html)、[XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)（stable，MuJoCo 3.x）
>
> 擷取日期：2026-09-09
>
> 測試環境：Linux、MuJoCo 3.12.0、Python 3.12.3；範例已實測通過。

## 學習目標

- 用 MJCF 建立一台 AMR 叉車：底盤移動 + 牙叉升降（lift）+ 左右側移（sideshift）
- 認識 `<velocity>` 與 `<position>` 兩種致動器的分工
- 完成一段「取貨」動作流程並驗證貨物真的被抬起

## 前置知識

- [02｜MJCF 建模基礎](../01-basics/01-mjcf-basics.md)、[03｜程式設計入門](../02-programming/01-simulation-loop.md)

## 模型設計（`models/amr_forklift.xml`）

![AMR 叉車](../assets/amr_forklift.png)

### 底盤：平面式簡化模型

真實 AMR 是差速輪驅動；教學上常用的簡化是把底盤做成「平面自由度」：

```xml
<joint name="move_x" type="slide" axis="1 0 0"/>
<joint name="move_y" type="slide" axis="0 1 0"/>
<joint name="yaw"    type="hinge" axis="0 0 1"/>
```

三個關節讓底盤可在地面任意移動與轉向，搭配 `<velocity>` 致動器直接命令速度 — 這等於把底盤控制問題抽象掉，專注在上層任務。需要更真實時再換成輪子模型。

### 門架與牙叉

- **升降**：滑架（carriage）用 z 軸 slide joint（`range="0 0.65"`），配 `<position>` 伺服 — 控制量是**目標高度**。
- **側移**：牙叉（forks）用 y 軸 slide joint（`range="-0.12 0.12"`），同樣用 `<position>` 伺服。

### 棧板：下方要留空隙

> ⚠️ 本章第一版把棧板做成實心箱子，牙叉永遠「插不進去」。真實棧板下方有腳、留有空隙，模型也要照做：板面 + 四隻腳。

## 控制範例（`scripts/ex_forklift.py`）

五個階段的取貨流程，每階段給定固定控制命令：

```python
run_phase(1.2, [0, 0.5, 0, 0.65, 0])   # 橫移對準棧板（y 方向）
run_phase(1.5, [0.8, 0, 0, 0.65, 0])   # 前進接近
run_phase(1.0, [0, 0, 0, 0.0, 0])      # 牙叉降到最低
run_phase(1.0, [0.5, 0, 0, 0.0, 0])    # 插入棧板下方
run_phase(2.0, [0, 0, 0, 0.35, 0])     # 升起（抬起貨物）
run_phase(1.0, [0, 0, 0, 0.35, -0.10]) # 側移修正
```

實測輸出：

```
橫移後底盤 y = 0.21 m
前進後底盤 x = 0.46 m
牙叉降到最低：lift = 0.000 m
插入後底盤 x = 1.08 m
升起後：lift = 0.296 m, 叉尖 z = 0.477 m
側移後：shift = -0.113 m
棧板高度 z = 0.140 m（初始 0.0，>0.05 表示被牙叉抬起）
結果：取貨動作完成 ✓
```

## 開發中踩過的坑（本章除錯紀錄）

1. **底盤完全不動**：裝飾用輪子 geom 與地面接觸摩擦把車鎖死（底盤 80 kg，摩擦力遠大於驅動力）。解法：裝飾件設 `contype="0" conaffinity="0"` 關掉碰撞。
2. **升降升不到目標高度**：`<position>` 伺服是 PD，`kp` 太小時重力造成穩態誤差 — 從 200 調到 2000。
3. **車子從棧板旁邊開過去**：路徑只有 x 方向，棧板在 y=0.3 — 實機上也一樣，**先對位再插入**。

## 常見錯誤與除錯

- **關節不動**：檢查致動器是否存在（`model.nu`）、`ctrlrange` 是否限制住、`kv/kp` 相對負載是否足夠。
- **東西穿模**：檢查 geom 的 `contype/conaffinity` 與初始幾何是否已重疊。
- **position 伺服到不了目標**：重力方向上的關節需要更大的 `kp` 或前饋補償。

## 延伸閱讀

- [Actuators — XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator)
- 下一篇：[14｜AMR 實驗（二）：搬運車 + 機械手臂 IK](02-mobile-manipulator.md)
