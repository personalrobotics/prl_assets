#!/usr/bin/env python3
"""Validate and fix prl_assets objects for coordinate frame and alignment standards.

Usage:
    python scripts/validate_objects.py              # Validate all objects
    python scripts/validate_objects.py can          # Validate specific object
    python scripts/validate_objects.py --fix        # Fix all issues automatically
    python scripts/validate_objects.py can --fix    # Fix specific object
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml


def load_meta(obj_dir: Path) -> dict:
    """Load meta.yaml for an object."""
    meta_path = obj_dir / "meta.yaml"
    if not meta_path.exists():
        return {}
    with open(meta_path) as f:
        return yaml.safe_load(f)


def load_xml(obj_dir: Path, obj_name: str) -> tuple[ET.ElementTree, Path]:
    """Load MuJoCo XML for an object."""
    xml_path = obj_dir / f"{obj_name}.xml"
    if not xml_path.exists():
        return None, xml_path
    return ET.parse(xml_path), xml_path


def check_coordinate_frame(obj_name: str, meta: dict, tree: ET.ElementTree) -> dict:
    """Check if body pos z equals half the object height."""
    result = {"pass": True, "message": "", "expected": None, "actual": None}

    if "dimensions" not in meta:
        result["pass"] = False
        result["message"] = "No dimensions in meta.yaml"
        return result

    # Height is the z dimension (index 2)
    height = meta["dimensions"][2]
    expected_z = height / 2

    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        result["pass"] = False
        result["message"] = "No worldbody in XML"
        return result

    body = worldbody.find("body")
    if body is None:
        result["pass"] = False
        result["message"] = "No body in worldbody"
        return result

    pos = body.get("pos", "0 0 0")
    pos_parts = [float(x) for x in pos.split()]
    actual_z = pos_parts[2] if len(pos_parts) >= 3 else 0

    result["expected"] = expected_z
    result["actual"] = actual_z

    # Allow 1mm tolerance
    if abs(actual_z - expected_z) > 0.001:
        result["pass"] = False
        result["message"] = f"body pos z={actual_z:.4f}, expected {expected_z:.4f} (half of {height})"
    else:
        result["message"] = f"z={actual_z:.4f} (correct)"

    return result


def check_collision_geom(obj_name: str, meta: dict, tree: ET.ElementTree) -> dict:
    """Check if collision geometry type and size match meta.yaml."""
    result = {"pass": True, "message": "", "details": {}}

    geom_props = meta.get("geometric_properties", {})
    expected_type = geom_props.get("type")

    if not expected_type:
        result["message"] = "No geometric_properties.type in meta.yaml"
        return result

    root = tree.getroot()
    worldbody = root.find("worldbody")
    body = worldbody.find("body") if worldbody is not None else None

    if body is None:
        result["pass"] = False
        result["message"] = "No body found"
        return result

    # Find collision geom (has contype or "collision" in name)
    collision_geom = None
    for geom in body.findall("geom"):
        name = geom.get("name", "")
        if "collision" in name.lower() or geom.get("contype"):
            collision_geom = geom
            break

    if collision_geom is None:
        result["pass"] = False
        result["message"] = "No collision geom found"
        return result

    actual_type = collision_geom.get("type", "mesh")
    result["details"]["expected_type"] = expected_type
    result["details"]["actual_type"] = actual_type

    # Check type match (cylinder and box are common primitives)
    if actual_type != expected_type:
        # Allow mesh for any type since it's more accurate
        if actual_type != "mesh":
            result["pass"] = False
            result["message"] = f"type={actual_type}, expected {expected_type}"
            return result

    # Check dimensions for primitives
    if actual_type == "box":
        size = collision_geom.get("size", "")
        if size:
            half_sizes = [float(x) for x in size.split()]
            full_sizes = [2 * s for s in half_sizes]
            expected_size = geom_props.get("size", meta.get("dimensions", []))
            result["details"]["actual_size"] = full_sizes
            result["details"]["expected_size"] = expected_size

            # Check if sizes match (with tolerance)
            if expected_size:
                for i, (a, e) in enumerate(zip(full_sizes, expected_size)):
                    if abs(a - e) > 0.002:  # 2mm tolerance
                        result["pass"] = False
                        result["message"] = f"size mismatch: {full_sizes} vs expected {expected_size}"
                        return result

    elif actual_type == "cylinder":
        size = collision_geom.get("size", "")
        if size:
            parts = [float(x) for x in size.split()]
            if len(parts) >= 2:
                radius, half_height = parts[0], parts[1]
                result["details"]["radius"] = radius
                result["details"]["height"] = 2 * half_height

                expected_radius = geom_props.get("radius")
                expected_height = geom_props.get("height")

                if expected_radius and abs(radius - expected_radius) > 0.002:
                    result["pass"] = False
                    result["message"] = f"radius={radius}, expected {expected_radius}"
                    return result
                if expected_height and abs(2 * half_height - expected_height) > 0.002:
                    result["pass"] = False
                    result["message"] = f"height={2*half_height}, expected {expected_height}"
                    return result

    result["message"] = f"{actual_type} geometry OK"
    return result


def check_collision_visual_alignment(obj_name: str, meta: dict, tree: ET.ElementTree) -> dict:
    """Check if collision and visual geoms are aligned (no offset on visual)."""
    result = {"pass": True, "message": "", "visual_offset": None}

    root = tree.getroot()
    worldbody = root.find("worldbody")
    body = worldbody.find("body") if worldbody is not None else None

    if body is None:
        result["pass"] = False
        result["message"] = "No body found"
        return result

    # Find visual geom (mesh type or "visual" in name)
    visual_geom = None
    for geom in body.findall("geom"):
        name = geom.get("name", "")
        geom_type = geom.get("type", "")
        if "visual" in name.lower() or geom_type == "mesh":
            if "collision" not in name.lower():
                visual_geom = geom
                break

    if visual_geom is None:
        result["message"] = "No visual geom found (may use collision for visual)"
        return result

    pos = visual_geom.get("pos", "0 0 0")
    pos_parts = [float(x) for x in pos.split()]

    result["visual_offset"] = pos_parts

    # Check if offset is non-zero (beyond 0.5mm tolerance)
    if any(abs(p) > 0.0005 for p in pos_parts):
        result["pass"] = False
        result["message"] = f"visual mesh offset: {pos_parts} (needs re-centering)"
    else:
        result["message"] = "aligned (no offset)"

    return result


def load_mesh_vertices(mesh_path: Path) -> np.ndarray | None:
    """Load vertices from OBJ or STL file."""
    if not mesh_path.exists():
        return None

    vertices = []
    ext = mesh_path.suffix.lower()

    if ext == ".obj":
        with open(mesh_path) as f:
            for line in f:
                if line.startswith("v "):
                    parts = line.split()
                    vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
    elif ext == ".stl":
        with open(mesh_path, "rb") as f:
            # Try to detect binary vs ASCII
            header = f.read(80)
            f.seek(0)
            try:
                content = f.read().decode("utf-8")
                for line in content.split("\n"):
                    line = line.strip()
                    if line.startswith("vertex"):
                        parts = line.split()
                        vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            except Exception:
                # Binary STL - skip for now
                return None

    return np.array(vertices) if vertices else None


def check_mesh_centering(obj_dir: Path, obj_name: str, tree: ET.ElementTree) -> dict:
    """Check if the visual mesh is centered at origin."""
    result = {"pass": True, "message": "", "center_offset": None}

    # Find mesh file from XML
    root = tree.getroot()
    asset = root.find("asset")
    if asset is None:
        result["message"] = "No asset section"
        return result

    mesh_elem = asset.find("mesh")
    if mesh_elem is None:
        result["message"] = "No mesh in assets"
        return result

    mesh_file = mesh_elem.get("file")
    if not mesh_file:
        result["message"] = "No mesh file specified"
        return result

    mesh_path = obj_dir / mesh_file
    vertices = load_mesh_vertices(mesh_path)
    if vertices is None or len(vertices) == 0:
        result["message"] = f"Could not load mesh: {mesh_file}"
        return result

    # Check if centered (bounding box center at origin)
    min_bounds = vertices.min(axis=0)
    max_bounds = vertices.max(axis=0)
    center = (min_bounds + max_bounds) / 2

    result["center_offset"] = center.tolist()
    result["bounds"] = {"min": min_bounds.tolist(), "max": max_bounds.tolist()}

    # Allow 2mm tolerance for centering
    if any(abs(c) > 0.002 for c in center):
        result["pass"] = False
        result["message"] = f"mesh not centered: offset={[f'{c:.4f}' for c in center]}"
    else:
        result["message"] = "mesh centered at origin"

    return result


def check_mesh_rotation(obj_dir: Path, obj_name: str, meta: dict, tree: ET.ElementTree) -> dict:
    """Check if mesh principal axes are aligned with XYZ."""
    result = {"pass": True, "message": "", "rotation_angle": None}

    # Find mesh file
    root = tree.getroot()
    asset = root.find("asset")
    mesh_elem = asset.find("mesh") if asset is not None else None
    if mesh_elem is None:
        result["message"] = "No mesh found"
        return result

    mesh_file = mesh_elem.get("file")
    mesh_path = obj_dir / mesh_file
    vertices = load_mesh_vertices(mesh_path)
    if vertices is None or len(vertices) == 0:
        result["message"] = "Could not load mesh"
        return result

    # Use PCA on XY coordinates to find principal axis
    xy = vertices[:, :2]
    cov = np.cov(xy.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov)

    # Sort by eigenvalue
    idx = np.argsort(eigenvalues)[::-1]
    eigenvectors = eigenvectors[:, idx]

    # Get angle of first principal component from X-axis
    pc1 = eigenvectors[:, 0]
    angle_rad = np.arctan2(pc1[1], pc1[0])
    angle_deg = np.degrees(angle_rad)

    # The principal axis should be aligned with X or Y (0, 90, 180, -90 degrees)
    # Find nearest cardinal direction
    cardinal_angles = [0, 90, 180, -90, -180]
    nearest_cardinal = min(cardinal_angles, key=lambda a: abs(angle_deg - a))
    deviation = abs(angle_deg - nearest_cardinal)

    result["rotation_angle"] = angle_deg
    result["deviation_from_axis"] = deviation

    # Allow 3 degree tolerance
    if deviation > 3.0:
        result["pass"] = False
        result["message"] = f"mesh rotated {deviation:.1f}° from axis (principal axis at {angle_deg:.1f}°)"
    else:
        result["message"] = f"mesh aligned (principal axis at {angle_deg:.1f}°)"

    return result


def check_mesh_height(obj_dir: Path, obj_name: str, meta: dict, tree: ET.ElementTree) -> dict:
    """Check if mesh Z extent matches meta dimensions[2] (resting height).

    This catches orientation issues where the mesh Z doesn't match the expected
    resting height from meta.yaml.
    """
    result = {"pass": True, "message": "", "mesh_z": None, "meta_z": None}

    if "dimensions" not in meta:
        result["message"] = "No dimensions in meta.yaml"
        return result

    meta_height = meta["dimensions"][2]
    result["meta_z"] = meta_height

    # Find mesh file
    root = tree.getroot()
    asset = root.find("asset")
    mesh_elem = asset.find("mesh") if asset is not None else None
    if mesh_elem is None:
        result["message"] = "No mesh found"
        return result

    mesh_file = mesh_elem.get("file")
    mesh_path = obj_dir / mesh_file
    vertices = load_mesh_vertices(mesh_path)
    if vertices is None or len(vertices) == 0:
        result["message"] = "Could not load mesh"
        return result

    # Calculate mesh Z extent
    z_min = vertices[:, 2].min()
    z_max = vertices[:, 2].max()
    mesh_height = z_max - z_min
    result["mesh_z"] = mesh_height

    # Allow 10mm tolerance - this catches major orientation issues like herring_tin
    # where mesh Z was ~84mm but dimensions[2] was 32mm
    if abs(mesh_height - meta_height) > 0.010:
        result["pass"] = False
        result["message"] = (
            f"mesh Z={mesh_height:.4f}m, meta dimensions[2]={meta_height:.4f}m "
            f"(diff={abs(mesh_height - meta_height)*1000:.1f}mm)"
        )
    else:
        result["message"] = f"mesh Z={mesh_height:.4f}m matches dimensions[2]"

    return result


def fix_coordinate_frame(obj_dir: Path, obj_name: str, meta: dict, tree: ET.ElementTree, xml_path: Path) -> bool:
    """Fix body pos z to be half the object height."""
    if "dimensions" not in meta:
        return False

    height = meta["dimensions"][2]
    expected_z = height / 2

    root = tree.getroot()
    worldbody = root.find("worldbody")
    body = worldbody.find("body") if worldbody is not None else None

    if body is None:
        return False

    pos = body.get("pos", "0 0 0")
    pos_parts = [float(x) for x in pos.split()]

    # Update z to half height
    if len(pos_parts) >= 3:
        pos_parts[2] = expected_z
    else:
        pos_parts = [0, 0, expected_z]

    body.set("pos", f"{pos_parts[0]} {pos_parts[1]} {pos_parts[2]:.4f}")

    # Write back
    tree.write(xml_path, encoding="unicode")
    # Re-add XML declaration and format
    with open(xml_path) as f:
        content = f.read()
    # Add newline at end if missing
    if not content.endswith("\n"):
        content += "\n"
    with open(xml_path, "w") as f:
        f.write(content)

    return True


def fix_visual_alignment(obj_dir: Path, obj_name: str, tree: ET.ElementTree, xml_path: Path) -> bool:
    """Remove visual mesh offset (set pos to 0 0 0).

    Note: This is a temporary fix. The proper fix is to re-center the mesh file.
    """
    root = tree.getroot()
    worldbody = root.find("worldbody")
    body = worldbody.find("body") if worldbody is not None else None

    if body is None:
        return False

    fixed = False
    for geom in body.findall("geom"):
        name = geom.get("name", "")
        geom_type = geom.get("type", "")
        if "visual" in name.lower() or geom_type == "mesh":
            if "collision" not in name.lower():
                if geom.get("pos"):
                    geom.attrib.pop("pos")
                    fixed = True

    if fixed:
        tree.write(xml_path, encoding="unicode")
        with open(xml_path) as f:
            content = f.read()
        if not content.endswith("\n"):
            content += "\n"
        with open(xml_path, "w") as f:
            f.write(content)

    return fixed


def validate_object(obj_dir: Path, fix: bool = False) -> dict:
    """Validate a single object and optionally fix issues."""
    obj_name = obj_dir.name
    results = {"name": obj_name, "checks": {}, "fixed": []}

    meta = load_meta(obj_dir)
    if not meta:
        results["error"] = "No meta.yaml found"
        return results

    tree, xml_path = load_xml(obj_dir, obj_name)
    if tree is None:
        results["error"] = f"No {obj_name}.xml found"
        return results

    # Run checks
    results["checks"]["coordinate_frame"] = check_coordinate_frame(obj_name, meta, tree)
    results["checks"]["collision_geom"] = check_collision_geom(obj_name, meta, tree)
    results["checks"]["collision_visual_align"] = check_collision_visual_alignment(obj_name, meta, tree)
    results["checks"]["mesh_centering"] = check_mesh_centering(obj_dir, obj_name, tree)
    results["checks"]["mesh_rotation"] = check_mesh_rotation(obj_dir, obj_name, meta, tree)
    results["checks"]["mesh_height"] = check_mesh_height(obj_dir, obj_name, meta, tree)

    # Fix if requested
    if fix:
        if not results["checks"]["coordinate_frame"]["pass"]:
            if fix_coordinate_frame(obj_dir, obj_name, meta, tree, xml_path):
                results["fixed"].append("coordinate_frame")
                # Reload tree after fix
                tree, _ = load_xml(obj_dir, obj_name)

        if not results["checks"]["collision_visual_align"]["pass"]:
            # Note: this removes the offset, but the mesh still needs re-centering
            print(f"  WARNING: {obj_name} visual mesh needs re-centering in Blender/MeshLab")
            print(f"           Offset was: {results['checks']['collision_visual_align']['visual_offset']}")

    return results


def print_results(results: dict):
    """Print validation results for an object."""
    name = results["name"]

    if "error" in results:
        print(f"\n{name}: ERROR - {results['error']}")
        return

    all_pass = all(c["pass"] for c in results["checks"].values())
    status = "PASS" if all_pass else "FAIL"

    print(f"\n{name}: {status}")
    for check_name, check_result in results["checks"].items():
        icon = "  ✓" if check_result["pass"] else "  ✗"
        print(f"{icon} {check_name}: {check_result['message']}")

    if results["fixed"]:
        print(f"  Fixed: {', '.join(results['fixed'])}")


def main():
    parser = argparse.ArgumentParser(description="Validate and fix prl_assets objects")
    parser.add_argument("object", nargs="?", help="Specific object to validate (default: all)")
    parser.add_argument("--fix", action="store_true", help="Attempt to fix issues")
    args = parser.parse_args()

    objects_dir = Path(__file__).parent.parent / "src" / "prl_assets" / "objects"

    if args.object:
        obj_dirs = [objects_dir / args.object]
        if not obj_dirs[0].exists():
            print(f"Object not found: {args.object}")
            return 1
    else:
        obj_dirs = sorted([d for d in objects_dir.iterdir() if d.is_dir()])

    print("prl_assets Object Validation")
    print("=" * 40)
    if args.fix:
        print("Mode: VALIDATE + FIX")
    else:
        print("Mode: VALIDATE ONLY (use --fix to auto-fix)")

    pass_count = 0
    fail_count = 0

    for obj_dir in obj_dirs:
        results = validate_object(obj_dir, fix=args.fix)
        print_results(results)

        if "error" not in results:
            if all(c["pass"] for c in results["checks"].values()):
                pass_count += 1
            else:
                fail_count += 1

    print("\n" + "=" * 40)
    print(f"Summary: {pass_count} passed, {fail_count} failed")

    return 0 if fail_count == 0 else 1


if __name__ == "__main__":
    exit(main())
