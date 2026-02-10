#!/usr/bin/env python3
"""Re-center mesh files to have origin at bottom-center.

This script moves mesh vertices so that:
- X,Y origin is at the center of the bounding box
- Z origin is at the bottom of the bounding box (z_min = 0)

Usage:
    python scripts/recenter_mesh.py path/to/mesh.obj              # Preview only
    python scripts/recenter_mesh.py path/to/mesh.obj --apply      # Apply changes
    python scripts/recenter_mesh.py path/to/mesh.stl --apply      # Works with STL too
"""

import argparse
import shutil
from pathlib import Path

import numpy as np


def load_obj(filepath: Path) -> tuple[np.ndarray, list]:
    """Load vertices and faces from OBJ file."""
    vertices = []
    faces = []
    other_lines = []  # Preserve texture coords, normals, materials, etc.

    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if line.startswith("v "):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("f "):
                faces.append(line)
            else:
                other_lines.append(line)

    return np.array(vertices), faces, other_lines


def save_obj(filepath: Path, vertices: np.ndarray, faces: list, other_lines: list):
    """Save vertices and faces to OBJ file."""
    with open(filepath, "w") as f:
        # Write header/comments first
        for line in other_lines:
            if line.startswith("#") or line.startswith("mtllib"):
                f.write(line + "\n")

        # Write vertices
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

        # Write other data (normals, texture coords, etc.)
        for line in other_lines:
            if not line.startswith("#") and not line.startswith("mtllib"):
                if line.startswith("vt ") or line.startswith("vn ") or line.startswith("usemtl") or line.startswith("g ") or line.startswith("s "):
                    f.write(line + "\n")

        # Write faces
        for face in faces:
            f.write(face + "\n")


def load_stl_ascii(filepath: Path) -> np.ndarray:
    """Load vertices from ASCII STL file."""
    vertices = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if line.startswith("vertex"):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(vertices)


def load_stl_binary(filepath: Path) -> np.ndarray:
    """Load vertices from binary STL file."""
    with open(filepath, "rb") as f:
        # Skip 80-byte header
        f.read(80)
        # Read number of triangles
        num_triangles = np.frombuffer(f.read(4), dtype=np.uint32)[0]

        vertices = []
        for _ in range(num_triangles):
            # Skip normal (12 bytes)
            f.read(12)
            # Read 3 vertices (36 bytes)
            for _ in range(3):
                v = np.frombuffer(f.read(12), dtype=np.float32)
                vertices.append(v.tolist())
            # Skip attribute byte count (2 bytes)
            f.read(2)

    return np.array(vertices)


def is_binary_stl(filepath: Path) -> bool:
    """Check if STL file is binary format."""
    with open(filepath, "rb") as f:
        # Read first 80 bytes (header)
        header = f.read(80)
        # Check if it starts with "solid" (ASCII format)
        try:
            if header.strip().startswith(b"solid"):
                # Could still be binary with "solid" in header, check further
                f.seek(0)
                content = f.read(200).decode("utf-8", errors="ignore")
                if "facet" in content and "vertex" in content:
                    return False  # ASCII
        except Exception:
            pass
    return True  # Binary


def save_stl_binary(filepath: Path, vertices: np.ndarray, normals: np.ndarray = None):
    """Save vertices to binary STL file.

    Assumes vertices are in groups of 3 (triangles).
    """
    num_triangles = len(vertices) // 3

    with open(filepath, "wb") as f:
        # Write 80-byte header
        header = b"Binary STL re-centered by prl_assets" + b"\0" * (80 - 37)
        f.write(header)

        # Write number of triangles
        f.write(np.array([num_triangles], dtype=np.uint32).tobytes())

        # Write triangles
        for i in range(num_triangles):
            v1 = vertices[i * 3]
            v2 = vertices[i * 3 + 1]
            v3 = vertices[i * 3 + 2]

            # Calculate normal
            edge1 = v2 - v1
            edge2 = v3 - v1
            normal = np.cross(edge1, edge2)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal = normal / norm
            else:
                normal = np.array([0, 0, 1])

            # Write normal
            f.write(normal.astype(np.float32).tobytes())
            # Write vertices
            f.write(v1.astype(np.float32).tobytes())
            f.write(v2.astype(np.float32).tobytes())
            f.write(v3.astype(np.float32).tobytes())
            # Write attribute byte count
            f.write(np.array([0], dtype=np.uint16).tobytes())


def recenter_mesh(filepath: Path, apply: bool = False) -> dict:
    """Re-center a mesh file to have origin at bottom-center.

    Returns dict with before/after bounds and translation applied.
    """
    filepath = Path(filepath)
    ext = filepath.suffix.lower()

    result = {
        "file": str(filepath),
        "format": ext,
        "bounds_before": None,
        "bounds_after": None,
        "translation": None,
        "applied": False,
    }

    # Load mesh
    if ext == ".obj":
        vertices, faces, other_lines = load_obj(filepath)
    elif ext == ".stl":
        if is_binary_stl(filepath):
            vertices = load_stl_binary(filepath)
            result["format"] = ".stl (binary)"
        else:
            vertices = load_stl_ascii(filepath)
            result["format"] = ".stl (ascii)"
        faces = None
        other_lines = None
    else:
        raise ValueError(f"Unsupported format: {ext}")

    if len(vertices) == 0:
        raise ValueError("No vertices found in mesh")

    # Calculate current bounds
    min_bounds = vertices.min(axis=0)
    max_bounds = vertices.max(axis=0)
    result["bounds_before"] = {
        "min": min_bounds.tolist(),
        "max": max_bounds.tolist(),
        "size": (max_bounds - min_bounds).tolist(),
    }

    # Calculate translation to center XYZ at origin
    # (MuJoCo body pos handles the z offset, so mesh should be centered)
    center = (min_bounds + max_bounds) / 2
    translation = -center
    result["translation"] = translation.tolist()

    # Apply translation
    new_vertices = vertices + translation

    # Calculate new bounds
    new_min = new_vertices.min(axis=0)
    new_max = new_vertices.max(axis=0)
    result["bounds_after"] = {
        "min": new_min.tolist(),
        "max": new_max.tolist(),
        "size": (new_max - new_min).tolist(),
    }

    if apply:
        # Backup original
        backup_path = filepath.with_suffix(filepath.suffix + ".backup")
        if not backup_path.exists():
            shutil.copy(filepath, backup_path)
            print(f"  Backup saved to: {backup_path}")

        # Save re-centered mesh
        if ext == ".obj":
            save_obj(filepath, new_vertices, faces, other_lines)
        elif ext == ".stl":
            save_stl_binary(filepath, new_vertices)

        result["applied"] = True

    return result


def print_result(result: dict):
    """Print re-centering results."""
    print(f"\nFile: {result['file']}")
    print(f"Format: {result['format']}")

    bb = result["bounds_before"]
    print(f"\nBefore:")
    print(f"  Min: [{bb['min'][0]:.4f}, {bb['min'][1]:.4f}, {bb['min'][2]:.4f}]")
    print(f"  Max: [{bb['max'][0]:.4f}, {bb['max'][1]:.4f}, {bb['max'][2]:.4f}]")
    print(f"  Size: [{bb['size'][0]:.4f}, {bb['size'][1]:.4f}, {bb['size'][2]:.4f}]")

    t = result["translation"]
    print(f"\nTranslation to apply: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")

    ba = result["bounds_after"]
    print(f"\nAfter:")
    print(f"  Min: [{ba['min'][0]:.4f}, {ba['min'][1]:.4f}, {ba['min'][2]:.4f}]")
    print(f"  Max: [{ba['max'][0]:.4f}, {ba['max'][1]:.4f}, {ba['max'][2]:.4f}]")
    print(f"  Size: [{ba['size'][0]:.4f}, {ba['size'][1]:.4f}, {ba['size'][2]:.4f}]")

    if result["applied"]:
        print(f"\n✓ Changes applied to {result['file']}")
    else:
        print(f"\nDry run - use --apply to save changes")


def main():
    parser = argparse.ArgumentParser(description="Re-center mesh to bottom-center origin")
    parser.add_argument("mesh", help="Path to mesh file (.obj or .stl)")
    parser.add_argument("--apply", action="store_true", help="Apply changes (default: dry run)")
    args = parser.parse_args()

    try:
        result = recenter_mesh(args.mesh, apply=args.apply)
        print_result(result)
        return 0
    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    exit(main())
