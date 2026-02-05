#!/usr/bin/env python3
"""Test can rendering with assets dict."""

import mujoco
import mujoco.viewer
from pathlib import Path

# Read can XML
can_xml = Path("src/prl_assets/objects/can/can.xml").read_text()

# Read mesh file
mesh_path = Path("src/prl_assets/objects/can/can_visual.stl")
mesh_data = mesh_path.read_bytes()

# Create assets dict
assets = {"can_visual.stl": mesh_data}

# Create wrapper XML
xml = f"""
<mujoco model="test_can">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81"/>

  <asset>
    <mesh name="can_visual" file="can_visual.stl" scale="1 1 1"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"/>
    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true"/>
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <light pos="1 1 3" dir="0 0 -1" diffuse="0.5 0.5 0.5"/>

    <!-- Ground plane -->
    <geom name="ground" type="plane" size="2 2 0.1" material="grid" contype="1" conaffinity="1"/>

    <body name="can" pos="0 0 0.0615">
      <freejoint name="can_joint"/>

      <!-- Collision geometry - invisible -->
      <geom name="can_collision"
            type="cylinder"
            size="0.033 0.0615"
            mass="0.05"
            friction="0.6 0.005 0.0001"
            contype="1"
            conaffinity="1"
            rgba="0 0 0 0"/>

      <!-- Visual mesh -->
      <geom name="can_visual"
            type="mesh"
            mesh="can_visual"
            rgba="0.8 0.1 0.1 1"
            contype="0"
            conaffinity="0"
            mass="0"/>
    </body>
  </worldbody>
</mujoco>
"""

print("Loading model with assets dict...", flush=True)
model = mujoco.MjModel.from_xml_string(xml, assets=assets)
data = mujoco.MjData(model)

print(f"Model loaded successfully", flush=True)
print(f"  Geoms: {model.ngeom}", flush=True)
for i in range(model.ngeom):
    geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
    geom_type = model.geom_type[i]
    rgba = model.geom_rgba[i]
    print(f"    {i}: {geom_name} type={geom_type} rgba={rgba}", flush=True)

print("\nLaunching viewer...", flush=True)
print("In viewer, press:", flush=True)
print("  G - toggle geom rendering", flush=True)
print("  C - toggle contact forces", flush=True)
print("  Esc - close viewer", flush=True)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.8
    viewer.cam.lookat[:] = [0, 0, 0.06]

    print("\nCan is at position (0, 0, 0.0615) - should be visible on the ground plane", flush=True)
    print("Red mesh can with ~65mm diameter, 123mm height", flush=True)

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

print("Done!", flush=True)
