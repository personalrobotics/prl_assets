#!/usr/bin/env python3
"""Demo scene: stacked bowls/plates/glasses, objects in yellow tote - with physics."""

import mujoco
import mujoco.viewer
import tempfile
from pathlib import Path

OBJECTS_DIR = Path(__file__).parent.parent / "src" / "prl_assets" / "objects"


def load_object_xml(obj_name):
    """Load and return processed XML content for an object."""
    obj_dir = OBJECTS_DIR / obj_name
    xml_path = obj_dir / f"{obj_name}.xml"

    if not xml_path.exists():
        return None, None

    import re
    xml_content = xml_path.read_text()

    # Extract asset section - prefix with obj_name to avoid conflicts
    asset_match = re.search(r'<asset>(.*?)</asset>', xml_content, re.DOTALL)
    asset_content = ""
    if asset_match:
        asset_content = asset_match.group(1)
        asset_content = asset_content.replace('name="', f'name="{obj_name}_')
        asset_content = asset_content.replace('texture="', f'texture="{obj_name}_')
        asset_content = asset_content.replace('mesh="', f'mesh="{obj_name}_')
        asset_content = asset_content.replace('file="', f'file="{obj_dir}/')

    # Extract body content (inside <body>...</body>) - don't prefix geom names yet
    body_match = re.search(r'<body name="[^"]*"[^>]*>(.*?)</body>\s*</worldbody>', xml_content, re.DOTALL)
    body_content = ""
    if body_match:
        body_content = body_match.group(1)
        # Only prefix mesh/material/texture refs to match asset names
        body_content = body_content.replace('mesh="', f'mesh="{obj_name}_')
        body_content = body_content.replace('material="', f'material="{obj_name}_')
        body_content = body_content.replace('texture="', f'texture="{obj_name}_')

    return asset_content, body_content


def create_body(instance_name, pos, body_content, with_joint=True):
    """Create a body element with given position."""
    import re
    # Make geom names unique by prefixing with instance_name
    def replace_name(m):
        return f'name="{instance_name}_{m.group(1)}"'
    unique_content = re.sub(r'name="([^"]*)"', replace_name, body_content)
    # Remove any existing freejoint
    unique_content = re.sub(r'<freejoint[^/]*/>', '', unique_content)

    joint = f'<freejoint name="{instance_name}_joint"/>' if with_joint else ''
    return f'''
    <body name="{instance_name}" pos="{pos[0]} {pos[1]} {pos[2]}">
      {joint}
      {unique_content}
    </body>'''


def main():
    assets = []
    bodies = []

    # Load all needed objects
    objects_needed = [
        "plastic_bowl", "plastic_plate", "plastic_glass", "wicker_tray", "yellow_tote",
        "can", "sugar_box", "pop_tarts", "gelatin_box", "pocky_box"
    ]

    obj_data = {}
    for obj_name in objects_needed:
        result = load_object_xml(obj_name)
        if result[0] and result[1]:
            asset, body = result
            obj_data[obj_name] = body
            if asset not in assets:
                assets.append(asset)

    # ===== SCENE 1: Wicker tray with stacked dishes =====
    # Wicker tray at (-0.4, 0, 0) - static
    bodies.append(create_body("tray1", (-0.4, 0, 0), obj_data["wicker_tray"], with_joint=False))

    # Stack of 3 plates (height=0.027 each) - tight spacing
    for i in range(3):
        z = 0.02 + i * 0.03  # just above each other
        bodies.append(create_body(f"plate_{i}", (-0.4, -0.15, z), obj_data["plastic_plate"]))

    # Stack of 3 bowls (height=0.076 each)
    for i in range(3):
        z = 0.02 + i * 0.08  # tight spacing
        bodies.append(create_body(f"bowl_{i}", (-0.4, 0.0, z), obj_data["plastic_bowl"]))

    # Stack of 3 glasses (height=0.171 each)
    for i in range(3):
        z = 0.02 + i * 0.18  # tight spacing
        bodies.append(create_body(f"glass_{i}", (-0.4, 0.15, z), obj_data["plastic_glass"]))

    # ===== SCENE 2: Yellow tote with objects =====
    # Yellow tote at (0.4, 0, 0) - static
    bodies.append(create_body("tote1", (0.4, 0, 0), obj_data["yellow_tote"], with_joint=False))

    # Objects dropped into tote (staggered heights to avoid collision)
    tote_items = [
        ("can", (0.35, -0.05, 0.20)),
        ("sugar_box", (0.45, 0.05, 0.28)),
        ("pop_tarts", (0.38, 0.08, 0.36)),
        ("gelatin_box", (0.42, -0.08, 0.44)),
        ("pocky_box", (0.4, 0, 0.52)),
    ]

    for i, (obj_name, pos) in enumerate(tote_items):
        if obj_name in obj_data:
            bodies.append(create_body(f"tote_item_{i}", pos, obj_data[obj_name]))

    # Build scene XML
    scene_xml = f'''<mujoco model="stacking_demo">
  <compiler angle="radian"/>

  <option gravity="0 0 -9.81" timestep="0.001" iterations="50" integrator="implicitfast" viscosity="2.0"/>

  <visual>
    <rgba haze="0.15 0.25 0.35 1"/>
    <quality shadowsize="4096"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.4 0.6 0.8" rgb2="0.1 0.1 0.2" width="512" height="3072"/>
    <texture type="2d" name="wood" builtin="flat" rgb1="0.6 0.4 0.2" rgb2="0.5 0.35 0.15" width="512" height="512" mark="random" markrgb="0.55 0.37 0.17"/>
    <material name="table" texture="wood" texrepeat="3 3" specular="0.3" shininess="0.3"/>
    {"".join(assets)}
  </asset>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>
    <light pos="1 1 2" dir="-0.5 -0.5 -1" diffuse="0.4 0.4 0.4"/>

    <!-- Table surface -->
    <geom name="table" type="box" size="1.2 0.6 0.02" pos="0 0 -0.02" material="table" contype="1" conaffinity="1"/>

    {"".join(bodies)}
  </worldbody>
</mujoco>'''

    # Write to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(scene_xml)
        xml_path = f.name

    print(f"Scene XML: {xml_path}")
    print("\nStacking Demo Scene (with physics):")
    print("  Left: Wicker tray with stacked plates, bowls, and glasses")
    print("  Right: Yellow tote with objects dropped in")
    print("\nLet it run for a few seconds to settle, then take screenshot")
    print("Controls: Mouse to rotate/pan/zoom, Space to pause, Esc to quit")

    # Load and display
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
