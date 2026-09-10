"""Blender headless：為 MR1533 建立 y 向 reach 機構（側移伸叉滑台）。

產出：
  models/meshes/yreach_carriage.stl  — reach 滑台 + 牙叉（原點在滑台背板中心）
  docs/assets/yreach_blender.png     — 展示圖

用法：blender --background --python scripts/make_yreach_blender.py -- <repo根目錄>
"""
import bpy
import sys
import math
from mathutils import Vector

ROOT = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "."


def clean_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def mat(name, color, rough, metallic=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metallic
    return m


def box(name, loc, dims, material, bevel=0.005):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.active_object
    o.name = name
    o.scale = dims
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    if bevel:
        mod = o.modifiers.new("bevel", "BEVEL")
        mod.width = bevel
        mod.segments = 2
        bpy.ops.object.modifier_apply(modifier="bevel")
    o.data.materials.append(material)
    return o


clean_scene()

steel = mat("steel", (0.30, 0.31, 0.34), 0.35, metallic=0.8)
yellow = mat("fork_yellow", (0.85, 0.65, 0.08), 0.45, metallic=0.3)

parts = []
# 滑台背板（貼在 MR1533 滑架上）
parts.append(box("backplate", (0, 0, 0.35), (0.04, 0.72, 0.70), steel, 0.01))
# 上下導軌（y 向滑動的軌道座）
parts.append(box("rail_top", (0.045, 0, 0.62), (0.05, 0.68, 0.06), steel))
parts.append(box("rail_bot", (0.045, 0, 0.08), (0.05, 0.68, 0.06), steel))
# 伸叉座（沿 y 滑出的部分）+ 兩根叉齒（叉齒沿 -x 伸出）
parts.append(box("reach_block", (0.09, 0, 0.35), (0.10, 0.30, 0.55), steel))
for sgn in (1, -1):
    parts.append(box(f"fork_{sgn}", (-0.45, sgn * 0.315, 0.16), (0.95, 0.14, 0.05), yellow, 0.008))
    # 叉齒後端垂直段（L 形）
    parts.append(box(f"fork_heel_{sgn}", (0.06, sgn * 0.315, 0.30), (0.07, 0.14, 0.33), yellow, 0.008))

# 合併成單一 mesh → STL（join 會烘培世界變換，原點保持在世界原點）
bpy.ops.object.select_all(action="DESELECT")
for o in parts:
    o.select_set(True)
bpy.context.view_layer.objects.active = parts[0]
bpy.ops.object.join()
bpy.ops.wm.stl_export(filepath=f"{ROOT}/models/meshes/yreach_carriage.stl",
                      export_selected_objects=True)
print("exported yreach_carriage.stl")

# ===== 展示圖 =====
scene = bpy.context.scene
cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
scene.collection.objects.link(cam)
cam.location = Vector((-1.8, -1.8, 1.3))
cam.rotation_euler = (Vector((-0.1, 0, 0.3)) - cam.location).to_track_quat("-Z", "Y").to_euler()
scene.camera = cam
sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", type="SUN"))
scene.collection.objects.link(sun)
sun.data.energy = 3.0
sun.rotation_euler = (math.radians(45), 0, math.radians(-30))
scene.world = bpy.data.worlds.new("w")
scene.world.use_nodes = True
scene.world.node_tree.nodes["Background"].inputs[0].default_value = (0.4, 0.45, 0.55, 1)
scene.world.node_tree.nodes["Background"].inputs[1].default_value = 0.6
scene.view_settings.exposure = -0.3
bpy.ops.mesh.primitive_plane_add(size=20)
fl = bpy.context.active_object
fl.data.materials.append(mat("floor", (0.28, 0.3, 0.34), 0.9))
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x = 960
scene.render.resolution_y = 720
scene.render.filepath = f"{ROOT}/docs/assets/yreach_blender.png"
bpy.ops.render.render(write_still=True)
print("rendered yreach_blender.png")
