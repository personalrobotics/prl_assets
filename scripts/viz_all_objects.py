#!/usr/bin/env python3
"""Visualize all prl_assets objects with collision geometry and coordinate frames."""

import mujoco
import mujoco.viewer
import tempfile
from pathlib import Path

OBJECTS_DIR = Path(__file__).parent.parent / "src" / "prl_assets" / "objects"

# Grid layout for 17 objects (5 columns x 4 rows)
GRID_COLS = 5
GRID_SPACING = 0.5

OBJECTS = [
    "can", "cracker_box", "fuze_bottle", "gelatin_box", "herring_tin",
    "lunchbox", "notebook", "plastic_bowl", "plastic_glass", "plastic_plate",
    "pocky_box", "pop_tarts", "potted_meat_can", "recycle_bin", "sugar_box",
    "wicker_tray", "yellow_tote"
]


def create_scene_xml():
    """Create XML with all objects arranged in a grid."""

    # Collect all assets and bodies
    assets = []
    bodies = []

    for i, obj_name in enumerate(OBJECTS):
        obj_dir = OBJECTS_DIR / obj_name
        xml_path = obj_dir / f"{obj_name}.xml"

        if not xml_path.exists():
            continue

        # Calculate grid position
        col = i % GRID_COLS
        row = i // GRID_COLS
        x = (col - GRID_COLS / 2 + 0.5) * GRID_SPACING
        y = (row - 2) * GRID_SPACING

        # Read and parse the object XML
        xml_content = xml_path.read_text()

        # Extract asset section
        import re
        asset_match = re.search(r'<asset>(.*?)</asset>', xml_content, re.DOTALL)
        if asset_match:
            asset_content = asset_match.group(1)
            # Prefix names to avoid collisions and fix file paths
            asset_content = asset_content.replace('name="', f'name="{obj_name}_')
            asset_content = asset_content.replace('texture="', f'texture="{obj_name}_')
            asset_content = asset_content.replace('mesh="', f'mesh="{obj_name}_')
            asset_content = asset_content.replace('file="', f'file="{obj_dir}/')
            assets.append(asset_content)

        # Extract body section - preserve the original z offset
        body_match = re.search(r'<body name="[^"]*"\s+pos="([^"]*)"\s*>(.*?)</body>\s*</worldbody>', xml_content, re.DOTALL)
        if body_match:
            orig_pos = body_match.group(1).split()
            orig_z = float(orig_pos[2]) if len(orig_pos) >= 3 else 0
            body_content = body_match.group(0).replace('</worldbody>', '')
            # Update position preserving original z offset
            body_content = re.sub(r'pos="([^"]*)"', f'pos="{x} {y} {orig_z}"', body_content, count=1)
            body_content = body_content.replace('name="', f'name="{obj_name}_')
            body_content = body_content.replace('mesh="', f'mesh="{obj_name}_')
            body_content = body_content.replace('material="', f'material="{obj_name}_')
            body_content = body_content.replace('texture="', f'texture="{obj_name}_')
            # Remove freejoint for static display
            body_content = re.sub(r'<freejoint[^/]*/>', '', body_content)
            bodies.append(body_content)

            # Add coordinate frame axes at object position
            axis_len = 0.05
            bodies.append(f'''
    <geom name="{obj_name}_x_axis" type="cylinder" fromto="{x} {y} 0.001 {x+axis_len} {y} 0.001" size="0.002" rgba="1 0 0 1"/>
    <geom name="{obj_name}_y_axis" type="cylinder" fromto="{x} {y} 0.001 {x} {y+axis_len} 0.001" size="0.002" rgba="0 1 0 1"/>
    <geom name="{obj_name}_z_axis" type="cylinder" fromto="{x} {y} 0.001 {x} {y} {0.001+axis_len}" size="0.002" rgba="0 0 1 1"/>''')

    # Build complete scene XML
    scene_xml = f'''<mujoco model="all_objects">
  <compiler angle="radian"/>

  <option gravity="0 0 0"/>

  <visual>
    <rgba haze="0.15 0.25 0.35 1"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    {"".join(assets)}
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="1 1 1"/>
    <geom name="floor" type="plane" size="3 3 0.1" material="groundplane"/>

    <!-- Coordinate frame at origin -->
    <geom name="x_axis" type="cylinder" fromto="0 0 0.001 0.1 0 0.001" size="0.003" rgba="1 0 0 1"/>
    <geom name="y_axis" type="cylinder" fromto="0 0 0.001 0 0.1 0.001" size="0.003" rgba="0 1 0 1"/>
    <geom name="z_axis" type="cylinder" fromto="0 0 0.001 0 0 0.1" size="0.003" rgba="0 0 1 1"/>

    {"".join(bodies)}
  </worldbody>
</mujoco>'''

    return scene_xml


def main():
    scene_xml = create_scene_xml()

    # Write to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(scene_xml)
        xml_path = f.name

    print(f"Scene XML: {xml_path}")
    print("\nAll 17 objects arranged in grid with collision overlays.")
    print("Controls: Mouse to rotate/pan/zoom, Esc to quit")

    # Load and display
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Make all collision geoms visible (green, semi-transparent)
    for i in range(model.ngeom):
        if model.geom_contype[i] == 1:
            model.geom_rgba[i] = [0, 1, 0, 0.3]

    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
