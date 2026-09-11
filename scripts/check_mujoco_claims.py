"""把教學文件裡的 MuJoCo 行為斷言變成可執行的驗證。

文件裡有一類敘述是「MuJoCo 會這樣做」—— 接觸摩擦怎麼合併、四元數什麼慣例、
哪個屬性的預設值是什麼。這些在寫的當下查證過，但**升版可能改變行為**，而文件
不會自己更新。把它們寫成測試，每次升版跑一遍就知道哪一條需要重寫。

每個檢查都標出它支撐文件的哪一段。

用法：
    python scripts/check_mujoco_claims.py
"""
import sys

import mujoco
import numpy as np

CHECKS = []


def claim(where, text):
    def deco(fn):
        CHECKS.append((where, text, fn))
        return fn
    return deco


@claim("16 章、20 章", "接觸摩擦取兩個 geom 的最大值")
def friction_max():
    m = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <geom type="plane" size="5 5 .1" friction="0.9 .005 .0001"/>
      <body pos="0 0 .3"><freejoint/>
        <geom type="box" size=".1 .1 .1" friction="0.2 .005 .0001"/></body>
    </worldbody></mujoco>""")
    d = mujoco.MjData(m)
    for _ in range(400):
        mujoco.mj_step(m, d)
    assert d.ncon, "沒有接觸，測不到"
    got = float(d.contact[0].friction[0])
    return abs(got - 0.9) < 1e-6, f"0.9 vs 0.2 → {got:.3f}"


@claim("多章", "四元數慣例是 (w, x, y, z)")
def quat_order():
    m = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <body quat="0.7071068 0 0 0.7071068"><geom type="box" size=".1 .1 .1"/></body>
    </worldbody></mujoco>""")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    x_axis = d.xmat[1].reshape(3, 3)[:, 0]
    return abs(x_axis[1] - 1) < 1e-3, f"x 軸轉到 {np.round(x_axis, 3)}（應為繞 z 轉 90°）"


@claim("11 章、22 章", "renderer.render() 不帶參數時每次回傳新陣列")
def render_new_array():
    m = mujoco.MjModel.from_xml_string(
        "<mujoco><worldbody><light pos='0 0 3'/>"
        "<geom type='plane' size='2 2 .1'/></worldbody></mujoco>")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    try:
        r = mujoco.Renderer(m, 64, 64)
    except Exception as e:                                  # noqa: BLE001
        # 沒有可用的 GL 後端（CI 的非渲染 job 就是這樣）—— 跳過，不算失敗
        return None, f"跳過：建立 Renderer 失敗（{type(e).__name__}）"
    r.update_scene(d)
    a, b = r.render(), r.render()
    return a is not b, f"兩次呼叫是同一個物件：{a is b}"


@claim("02 章、04 章", "MJCF 的 angle 預設是 degree")
def mjcf_degree():
    m = mujoco.MjModel.from_xml_string(
        "<mujoco><worldbody><body><joint type='hinge' axis='0 1 0' range='0 90'/>"
        "<geom type='box' size='.1 .1 .1'/></body></worldbody></mujoco>")
    return abs(m.jnt_range[0][1] - np.pi / 2) < 1e-3, \
        f"range='0 90' → {m.jnt_range[0][1]:.4f} rad"


@claim("04 章", "URDF 的 angle 預設是 radian")
def urdf_radian():
    import tempfile, pathlib
    urdf = """<?xml version="1.0"?><robot name="t">
      <link name="a"><inertial><mass value="1"/><origin xyz="0 0 0"/>
        <inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial></link>
      <link name="b"><inertial><mass value="1"/><origin xyz="0 0 0"/>
        <inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial></link>
      <joint name="j" type="revolute"><parent link="a"/><child link="b"/>
        <origin xyz="0 0 -.5"/><axis xyz="0 1 0"/>
        <limit lower="0" upper="1.5707963" effort="1" velocity="1"/></joint>
    </robot>"""
    with tempfile.TemporaryDirectory() as td:
        p = pathlib.Path(td) / "t.urdf"
        p.write_text(urdf)
        m = mujoco.MjModel.from_xml_path(str(p))
    return abs(m.jnt_range[0][1] - np.pi / 2) < 1e-3, \
        f"upper=1.5707963 → {m.jnt_range[0][1]:.4f} rad（radian 直接採用）"


@claim("04 章", "URDF 匯入時 discardvisual 預設為 true（MJCF 是 false）")
def urdf_discardvisual():
    import tempfile, pathlib
    body = """<link name="a">
        <inertial><mass value="1"/><origin xyz="0 0 0"/>
          <inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial>
        <visual><geometry><box size=".2 .2 .2"/></geometry></visual>
        <collision><geometry><box size=".2 .2 .2"/></geometry></collision>
      </link>"""
    out = {}
    for tag, extra in (("預設", ""),
                       ("discardvisual=false",
                        '<mujoco><compiler discardvisual="false"/></mujoco>')):
        with tempfile.TemporaryDirectory() as td:
            p = pathlib.Path(td) / "t.urdf"
            p.write_text(f'<?xml version="1.0"?><robot name="t">{extra}{body}</robot>')
            out[tag] = mujoco.MjModel.from_xml_path(str(p)).ngeom
    return out["預設"] == 1 and out["discardvisual=false"] == 2, \
        f"geom 數：預設 {out['預設']}、discardvisual=false {out['discardvisual=false']}"


@claim("04 章", "world 與其後代之間沒有自動碰撞排除（父子排除不涵蓋 world）")
def world_no_exclude():
    m = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <geom type="box" size=".2 .2 .05" pos="0 0 0"/>
      <body pos="0 0 .02"><joint type="hinge" axis="0 1 0"/>
        <geom type="box" size=".02 .02 .2" pos="0 0 -.1"/></body>
    </worldbody></mujoco>""")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return d.ncon > 0, f"world 的 geom 與其子 body 的 geom 重疊 → ncon={d.ncon}"


@claim("10 章", "mjd_transitionFD 回傳的是離散 Jacobian（接 LQR 前要除以 dt）")
def transition_fd_discrete():
    m = mujoco.MjModel.from_xml_string("""<mujoco><option timestep="0.01"/><worldbody>
      <body><joint name="j" type="slide" axis="1 0 0"/>
        <geom type="box" size=".1 .1 .1" mass="1"/></body>
    </worldbody><actuator><motor joint="j" gear="1"/></actuator></mujoco>""")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    n = 2 * m.nv
    A = np.zeros((n, n)); B = np.zeros((n, m.nu))
    mujoco.mjd_transitionFD(m, d, 1e-7, 1, A, B, None, None)
    # 離散型的 A 對角應該接近 1（x_{k+1} ≈ x_k + ...）；連續型會接近 0
    return abs(A[0, 0] - 1.0) < 1e-3, f"A[0,0] = {A[0,0]:.4f}（離散應 ≈1，連續應 ≈0）"


if __name__ == "__main__":
    print(f"MuJoCo {mujoco.__version__} — 驗證文件裡的行為斷言\n")
    bad = skipped = 0
    for where, text, fn in CHECKS:
        try:
            ok, detail = fn()
        except Exception as e:                              # noqa: BLE001
            ok, detail = False, f"執行失敗：{type(e).__name__}: {e}"
        mark = "–" if ok is None else ("✓" if ok else "✗")
        print(f"  {mark} [{where}] {text}")
        print(f"      {detail}")
        if ok is None:
            skipped += 1
        else:
            bad += not ok
    print()
    if bad:
        print(f"✗ {bad} 條斷言與實際行為不符 — 對應章節要重寫")
        sys.exit(1)
    tail = f"（跳過 {skipped} 條）" if skipped else ""
    print(f"✓ {len(CHECKS) - skipped} 條斷言全部成立{tail}")
