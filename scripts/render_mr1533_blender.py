"""Blender headless 渲染 MR1533 叉車（TB3 實驗資產）。

用法：blender --background ~/tmp2/TB3/assets/mr1533_light/mr1533_light.blend \
        --python scripts/render_mr1533_blender.py -- 輸出.png
"""
import bpy
import sys
import math
from mathutils import Vector

out = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "/tmp/mr1533_blender.png"

scene = bpy.context.scene

# 移除既有相機/光源，重建（場景若已有則沿用物件不清場景模型）
for obj in [o for o in scene.objects if o.type in {"CAMERA", "LIGHT"}]:
    bpy.data.objects.remove(obj)

# 相機：側前方 45° 俯視
cam_data = bpy.data.cameras.new("cam")
cam = bpy.data.objects.new("cam", cam_data)
scene.collection.objects.link(cam)
cam.location = Vector((3.2, -3.2, 2.4))
target = Vector((0.6, 0.0, 0.9))
cam.rotation_euler = (target - cam.location).to_track_quat("-Z", "Y").to_euler()
scene.camera = cam

# 主光源（太陽光）+ 補光
sun_data = bpy.data.lights.new("sun", type="SUN")
sun_data.energy = 2.5
sun = bpy.data.objects.new("sun", sun_data)
scene.collection.objects.link(sun)
sun.rotation_euler = (math.radians(50), 0, math.radians(30))

world = bpy.data.worlds.new("world") if not scene.world else scene.world
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs[0].default_value = (0.35, 0.4, 0.5, 1.0)
world.node_tree.nodes["Background"].inputs[1].default_value = 0.5
scene.view_settings.exposure = -0.7

# 地面
bpy.ops.mesh.primitive_plane_add(size=40, location=(0, 0, 0))
plane = bpy.context.active_object
mat = bpy.data.materials.new("floor")
mat.use_nodes = True
mat.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.3, 0.32, 0.36, 1)
plane.data.materials.append(mat)

# 渲染設定：EEVEE（快速）
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x = 960
scene.render.resolution_y = 720
scene.render.filepath = out

bpy.ops.render.render(write_still=True)
print("rendered ->", out)
