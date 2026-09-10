# 07｜更多 Python 範例：感測器、伺服、Keyframe、平行取樣、mjSpec

> 參考來源：[API Reference](https://mujoco.readthedocs.io/en/stable/APIreference/)、[Modeling](https://mujoco.readthedocs.io/en/stable/modeling.html)、[Model Editing（mjSpec）](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html)（stable，MuJoCo 3.x）
>
> 擷取日期：2026-09-09
>
> 測試環境：Linux、MuJoCo 3.12.0、Python 3.12.3；本章所有範例皆已實測，輸出為實際執行結果。

## 學習目標

透過五個可直接執行的範例，學會：

1. 觸覺感測器與加速度計（`scripts/ex_sensors.py`）
2. position 伺服致動器（`scripts/ex_position_servo.py`）
3. keyframe 狀態重置（`scripts/ex_keyframe.py`）
4. 多執行緒平行取樣（`scripts/ex_parallel_rollout.py`）
5. mjSpec 程序化建模（`scripts/ex_mjspec.py`）

## 前置知識

- [03｜程式設計入門](01-simulation-loop.md)

---

## 範例 1：觸覺感測器（touch）與加速度計

模型 `models/touch_ball.xml`：一顆用 slide 關節掛著的球自由落向地面，底部放一個 site，掛上 `<touch>` 與 `<accelerometer>` 感測器。

```python
model = mujoco.MjModel.from_xml_path("models/touch_ball.xml")
data = mujoco.MjData(model)

for _ in range(2000):
    mujoco.mj_step(model, data)
    touch, ax, ay, az = data.sensordata
    if touch > 0 and not touched:
        print(f"碰地！時間 t = {data.time:.3f} s, touch = {touch:.2f} N")
```

實測輸出：

```
碰地！時間 t = 0.288 s
  touch 讀值       = 1242.2718 N        ← 撞擊瞬間的法向力峰值
  accelerometer    = [0.00, 0.00, 296.64] m/s²
最終世界高度 z = 0.0996 m（球半徑 0.1，應停在附近）
```

> 譯註：**touch 感測器只量測 site 附近的接觸**。一開始把 site 放在球心時讀值永遠是 0，移到球底部（接觸發生處）才有訊號 — 這是除錯感測器時的第一個檢查點。

## 範例 2：position 伺服 — 不用自己寫 PD

[03 章](01-simulation-loop.md)用 `<motor>` 自己寫 PD；其實 MJCF 的 `<position>` 致動器內建 PD，`ctrl` 直接就是目標角度。模型 `models/servo_pendulum.xml`：

```xml
<position name="servo" joint="hinge" kp="10" ctrllimited="true" ctrlrange="-1.6 1.6"/>
```

`scripts/ex_position_servo.py` 讓擺追蹤正弦軌跡：

```python
while data.time < 3.0:
    data.ctrl[0] = np.sin(data.time)   # 目標角度
    mujoco.mj_step(model, data)
```

實測輸出：`追蹤誤差 RMS = 0.0690 rad`，最後 100 步降到 `0.0216 rad`。
其他伺服捷徑還有 `<velocity>`（kv）、`<general>`（自訂增益）。注意：致動器相關捷徑與 defaults 機制的互動有額外規則，見 [Modeling — Actuator shortcuts](https://mujoco.readthedocs.io/en/stable/modeling.html#actuator-shortcuts)。

## 範例 3：keyframe — episode 重置的標準做法

`<keyframe>` 把一組 `qpos`/`qvel`/`ctrl` 等狀態存在模型裡，`mj_resetDataKeyframe` 一行程式回到該狀態（連 `time` 都歸零）— 強化學習每回合重置環境就用它：

```python
key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "raised")
mujoco.mj_resetDataKeyframe(model, data, key_id)
```

實測輸出：

```
從 keyframe 擺 0.5s 後 qpos = -0.944
重置回 keyframe：qpos = 1.000（應為 1.0）, time = 0.0
```

## 範例 4：多執行緒平行取樣

回顧 [01 章](../00-intro/01-what-is-mujoco.md) 的「模型與資料分離」：`mjModel` 唯讀，可被多條執行緒共用；每條執行緒建立自己的 `mjData` 即可安全平行。`scripts/ex_parallel_rollout.py` 用 8 條執行緒對雙擺跑 16 個隨機初始狀態的 rollout：

```python
def rollout(seed):
    data = mujoco.MjData(model)          # 每條執行緒自己的 mjData
    data.qpos[:] = rng.uniform(-np.pi, np.pi, size=2)
    for _ in range(500):
        mujoco.mj_step(model, data)
    return data.qpos.copy()
```

實測輸出：16 條 rollout 全部完成，最終 qpos 分布各異（雙擺混沌特性）。這個模式正是 RL 蒐集訓練資料的基礎；更大規模時執行緒會先撞到 GIL，改用批次向量化的 [MJX](08-mjx-gpu.md)（28 章有三種做法的吞吐對照）或 MuJoCo Warp。

## 範例 5：mjSpec 程序化建模

不寫 XML，直接用 Python 建模型 — 適合參數化產生（例如一排便當長度的擺）：

```python
spec = mujoco.MjSpec()
for i in range(4):
    body = spec.worldbody.add_body(name=f"pend{i}", pos=[i * 0.3, 0, 1.0])
    body.add_joint(name=f"hinge{i}", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                  fromto=[0, 0, 0, 0, 0, -0.3 - 0.1 * i], size=[0.02])
model = spec.compile()
```

`spec.to_xml()` 還能把程序化建立的模型輸出成 MJCF 存檔（實測產生 `models/spec_generated.xml`）。完整 API 見 [Model Editing](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html)。

## 常見錯誤與除錯

- **touch 感測器讀不到**：site 必須在接觸發生的位置附近。
- **多執行緒結果互相污染**：每條執行緒要用自己的 `mjData`；共用同一個 `mjData` 是未定義行為。
- **`<position>` 追不上**：調大 `kp`、檢查 `ctrlrange`，或確認 `forcerange` 沒把力量限制掉。
- **mjSpec 編譯失敗**：`spec.compile()` 的錯誤訊息與 XML 載入相同，會指出是哪個元素有問題。

## 延伸閱讀

- [API Reference](https://mujoco.readthedocs.io/en/stable/APIreference/)
- [Sensors（XML Reference）](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor)
- [Model Editing — mjSpec](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html)
