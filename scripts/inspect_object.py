#!/usr/bin/env python3
"""Inspect a single prl_assets object with collision, visual, and coordinate frame.

Shows:
- Visual mesh (original colors)
- Collision geometry (green semi-transparent)
- Coordinate frame axes (RGB = XYZ)
- Table surface at z=0

Usage:
    python scripts/inspect_object.py can
    python scripts/inspect_object.py cracker_box
"""

import argparse
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


def create_inspection_scene(obj_dir: Path, obj_name: str) -> str:
    """Create a MuJoCo XML for inspecting a single object."""
    xml_path = obj_dir / f"{obj_name}.xml"

    if not xml_path.exists():
        raise FileNotFoundError(f"No XML file found: {xml_path}")

    # Parse the object XML
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Extract assets
    assets_xml = []
    asset_elem = root.find("asset")
    if asset_elem is not None:
        for mesh in asset_elem.findall("mesh"):
            mesh_file = mesh.get("file")
            if mesh_file:
                mesh.set("file", str(obj_dir / mesh_file))
            assets_xml.append(ET.tostring(mesh, encoding="unicode"))
        for texture in asset_elem.findall("texture"):
            tex_file = texture.get("file")
            if tex_file:
                texture.set("file", str(obj_dir / tex_file))
            assets_xml.append(ET.tostring(texture, encoding="unicode"))
        for material in asset_elem.findall("material"):
            assets_xml.append(ET.tostring(material, encoding="unicode"))

    # Extract body and modify for inspection
    worldbody = root.find("worldbody")
    body = worldbody.find("body") if worldbody is not None else None

    if body is None:
        raise ValueError("No body found in object XML")

    # Get body position for coordinate frame
    body_pos = body.get("pos", "0 0 0")
    pos_parts = [float(x) for x in body_pos.split()]
    body_z = pos_parts[2] if len(pos_parts) >= 3 else 0

    # Remove freejoint for static display
    for joint in body.findall("freejoint"):
        body.remove(joint)

    # Make collision geoms visible
    for geom in body.findall("geom"):
        geom_name = geom.get("name", "")
        if "collision" in geom_name.lower() or geom.get("contype"):
            # Make collision visible as green semi-transparent
            rgba = geom.get("rgba", "1 0 0 0.3")
            if "0 0 0 0" in rgba or rgba.endswith(" 0"):
                geom.set("rgba", "0.2 0.8 0.2 0.5")
            geom.set("group", "1")  # Always visible

    body_xml = ET.tostring(body, encoding="unicode")

    # Create scene with table, object, and coordinate frame
    scene_xml = f"""<mujoco model="{obj_name}_inspection">
  <compiler angle="radian"/>

  <option gravity="0 0 -9.81"/>

  <visual>
    <rgba haze="0.15 0.25 0.35 1"/>
    <quality shadowsize="4096"/>
    <map znear="0.001"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.4 0.6 0.8" rgb2="0 0 0" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".3 .4 .5" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.1"/>
    {chr(10).join(assets_xml)}
  </asset>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" diffuse="1 1 1" specular="0.3 0.3 0.3"/>
    <light pos="1 1 1" dir="-1 -1 -1" diffuse="0.5 0.5 0.5"/>

    <!-- Ground/table at z=0 -->
    <geom name="table" type="plane" size="0.5 0.5 0.01" material="grid" pos="0 0 0"/>

    <!-- Coordinate frame at world origin (where object base should be) -->
    <site name="origin" type="sphere" size="0.005" rgba="1 1 1 1" pos="0 0 0"/>

    <!-- X axis (red) -->
    <site name="x_axis" type="cylinder" size="0.002 0.05" rgba="1 0 0 1" pos="0.05 0 0" euler="0 1.5708 0"/>
    <site name="x_tip" type="box" size="0.008 0.004 0.004" rgba="1 0 0 1" pos="0.1 0 0"/>

    <!-- Y axis (green) -->
    <site name="y_axis" type="cylinder" size="0.002 0.05" rgba="0 1 0 1" pos="0 0.05 0" euler="1.5708 0 0"/>
    <site name="y_tip" type="box" size="0.004 0.008 0.004" rgba="0 1 0 1" pos="0 0.1 0"/>

    <!-- Z axis (blue) -->
    <site name="z_axis" type="cylinder" size="0.002 0.05" rgba="0 0 1 1" pos="0 0 0.05"/>
    <site name="z_tip" type="box" size="0.004 0.004 0.008" rgba="0 0 1 1" pos="0 0 0.1"/>

    <!-- Object body frame indicator (at body origin, which should be at z=half_height) -->
    <site name="body_origin" type="sphere" size="0.008" rgba="1 1 0 0.8" pos="0 0 {body_z}"/>

    <!-- The object -->
    {body_xml}
  </worldbody>
</mujoco>"""

    return scene_xml


def main():
    parser = argparse.ArgumentParser(description="Inspect a prl_assets object")
    parser.add_argument("object", help="Object name to inspect")
    args = parser.parse_args()

    objects_dir = Path(__file__).parent.parent / "src" / "prl_assets" / "objects"
    obj_dir = objects_dir / args.object

    if not obj_dir.exists():
        print(f"Object not found: {args.object}")
        print(f"Available objects: {', '.join(d.name for d in objects_dir.iterdir() if d.is_dir())}")
        return 1

    print(f"Inspecting: {args.object}")
    print("=" * 40)
    print("Legend:")
    print("  - RGB axes = XYZ coordinate frame at world origin")
    print("  - Yellow sphere = body origin (should be at bottom-center)")
    print("  - Green transparent = collision geometry")
    print("  - Textured = visual mesh")
    print("  - Gray plane = table at z=0")
    print()
    print("Controls:")
    print("  - Mouse: rotate/pan/zoom")
    print("  - Esc: quit")
    print("=" * 40)

    try:
        scene_xml = create_inspection_scene(obj_dir, args.object)

        # Write to temp file for debugging
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(scene_xml)
            print(f"Scene XML: {f.name}")

        import mujoco
        import mujoco.viewer

        model = mujoco.MjModel.from_xml_string(scene_xml)
        data = mujoco.MjData(model)

        mujoco.viewer.launch(model, data)
        return 0

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())
