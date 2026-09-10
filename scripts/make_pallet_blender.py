"""Blender headless：程序化建立木棧板 / 塑膠棧板（歐規 EUR pallet 比例簡化版）。

輸出：
  models/meshes/pallet_wood.obj / pallet_plastic.obj  （MuJoCo 用，公尺、Z-up）
  docs/assets/pallet_wood_blender.png / pallet_plastic_blender.png（展示圖）

用法：blender --background --python scripts/make_pallet_blender.py -- <repo根目錄>
"""
import bpy
import sys
import math
from mathutils import Vector

ROOT = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "."

# 棧板尺寸（公尺）：1.0 × 0.8，板厚 0.022，腳塊高 0.09
L, W = 1.0, 0.8
DECK_T = 0.022
FOOT_H = 0.09


def clean_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def box(name, loc, scale, mat=None, bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = (scale[0], scale[1], scale[2])   # cube size=1 → 直接用全寬
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    if bevel > 0:
        mod = obj.modifiers.new("bevel", "BEVEL")
        mod.width = bevel
        mod.segments = 2
        bpy.ops.object.modifier_apply(modifier="bevel")
    if mat:
        obj.data.materials.append(mat)
    return obj


def make_material(name, color, roughness):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1)
    bsdf.inputs["Roughness"].default_value = roughness
    return m


def build_pallet(mat):
    """建一塊棧板，回傳所有零件。原點在棧板底面中心。"""
    parts = []
    # 頂板：7 片板條（留縫）
    n_boards = 7
    gap = 0.012
    bw = (W - gap * (n_boards - 1)) / n_boards
    for i in range(n_boards):
        y = -W / 2 + bw / 2 + i * (bw + gap)
        parts.append(box(f"deck{i}", (0, y, FOOT_H + DECK_T / 2),
                         (L, bw, DECK_T), mat, bevel=0.003))
    # 三根縱向枕木（stringer）
    for x in (-L / 2 + 0.08, 0, L / 2 - 0.08):
        parts.append(box(f"stringer{x}", (x, 0, FOOT_H / 2 + 0.01),
                         (0.1, W, FOOT_H - 0.02 + 0.022), mat, bevel=0.004))
    return parts


def export_obj(objs, path):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.join()                      # 合併成單一 mesh
    bpy.ops.wm.stl_export(filepath=path.replace(".obj", ".stl"),
                          export_selected_objects=True)


def setup_render():
    scene = bpy.context.scene
    for obj in [o for o in scene.objects if o.type in {"CAMERA", "LIGHT"}]:
        bpy.data.objects.remove(obj)
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    cam.location = Vector((1.6, -1.6, 1.2))
    cam.rotation_euler = (Vector((0, 0, 0.06)) - cam.location).to_track_quat("-Z", "Y").to_euler()
    scene.camera = cam
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", type="SUN"))
    scene.collection.objects.link(sun)
    sun.data.energy = 3.0
    sun.rotation_euler = (math.radians(45), 0, math.radians(30))
    if not scene.world:
        scene.world = bpy.data.worlds.new("w")
    scene.world.use_nodes = True
    scene.world.node_tree.nodes["Background"].inputs[0].default_value = (0.4, 0.45, 0.55, 1)
    scene.world.node_tree.nodes["Background"].inputs[1].default_value = 0.5
    scene.view_settings.exposure = -0.3
    bpy.ops.mesh.primitive_plane_add(size=20, location=(0, 0, -0.001))
    floor_mat = make_material("floor", (0.28, 0.3, 0.34), 0.9)
    bpy.context.active_object.data.materials.append(floor_mat)
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 960
    scene.render.resolution_y = 720


CASES = {
    "pallet_wood":    ((0.45, 0.30, 0.15), 0.8),   # 木頭：棕、粗糙
    "pallet_plastic": ((0.15, 0.35, 0.75), 0.35),  # 塑膠：藍、略亮
}

for name, (color, rough) in CASES.items():
    clean_scene()
    mat = make_material(name, color, rough)
    parts = build_pallet(mat)
    export_obj(parts, f"{ROOT}/models/meshes/{name}.obj")
    setup_render()
    bpy.context.scene.render.filepath = f"{ROOT}/docs/assets/{name}_blender.png"
    bpy.ops.render.render(write_still=True)
    print("done:", name)
