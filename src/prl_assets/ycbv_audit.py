#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""Check every YCB-V asset's declared frame against the pinned BOP model it claims to be.

For each asset under ``objects/ycbv/``, the BOP model named by ``dataset.ycbv.bop_id`` is
converted from millimetres to metres, mapped into the asset frame by the declared
``frames.T_asset_bop``, and compared with the asset's visual mesh by surface distance
and by texture colour at the nearest surface points. ICP started from the declared
transform then reports how far the best fit lies from it. Geometry alone cannot tell a
nearly symmetric box from itself turned 180 degrees; its printed texture can, so the
colour check catches a frame that is flipped. The three 180-degree flips of each declared
transform are measured too and recorded as evidence that the checks separate them.

The script exits non-zero if any asset exceeds the tolerances below, and writes the
measurements as JSON (default ``artifacts/ycbv_audit.json``). The checkout wrapper
can also record the current catalog audit, only when every asset passes.

Usage:
    python -m prl_assets.ycbv_audit [--archive ycbv_models.zip] [--out PATH] [--record]
    python -m prl_assets.ycbv_audit --fetch-only   # download and verify, then exit

Without ``--archive`` the pinned archive (525 MB) is downloaded once into
``~/.cache/prl_assets/`` and verified against its SHA-256 before use.
"""

import argparse
import hashlib
import json
import os
import platform
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import numpy as np
import scipy
import trimesh
import yaml
from PIL import Image
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from prl_assets import YCBV_OBJECTS_DIR
from prl_assets.ycbv import BOP_YCB_NAMES
from prl_assets.ycbv_source import ARCHIVE_SHA256, ARCHIVE_URL, fetch_archive, verify_archive

# Two remeshings of one scan differ by about a millimetre; a wrong transform or a
# different object differs by centimetres.
SAMPLES = 20000
MAX_P99_MM = 3.0
MAX_RESIDUAL_DEG = 0.5
MAX_RESIDUAL_MM = 1.0
# Median absolute RGB difference (0-255, averaged over channels) between the two textures
# at nearest surface points. Correct frames measure 1.7-5.3; 180-degree flips 26-62.
MAX_COLOUR_MEDIAN = 12.0

YCBV_DIR = YCBV_OBJECTS_DIR


def _inside(obj_dir: Path, file: str | None, what: str) -> Path:
    if not file:
        raise SystemExit(f"{obj_dir.name}: the {what} names no file")
    path = (obj_dir / file).resolve()
    if not path.is_relative_to(obj_dir.resolve()):
        raise SystemExit(f"{obj_dir.name}: {what} {path} leaves the asset directory")
    return path


def visual_mesh(obj_dir: Path, meta: dict) -> tuple[Path, Path]:
    """The files of the asset's visual mesh geom and its texture.

    The mesh file's vertices are the asset frame only when nothing in the model moves or
    scales them, so a geom offset, a mesh scale or reference pose, or a compiler
    directory is refused rather than ignored.
    """
    tree = ET.parse(obj_dir / meta["mujoco"]["xml_path"])
    for c in tree.iter("compiler"):
        if any(c.get(a) for a in ("meshdir", "texturedir", "assetdir")):
            raise SystemExit(f"{obj_dir.name}: the audit does not resolve compiler meshdir/texturedir/assetdir")
    visual = [g for g in tree.iter("geom") if g.get("type") == "mesh" and g.get("group") == "2"]
    if len(visual) != 1:
        raise SystemExit(f"{obj_dir.name}: expected one visual mesh geom (group 2), found {len(visual)}")
    geom = visual[0]
    if any(geom.get(a) for a in ("pos", "quat", "euler", "axisangle", "xyaxes", "zaxis")):
        raise SystemExit(f"{obj_dir.name}: the visual geom must have no pose offset")
    (mesh,) = [m for m in tree.iter("mesh") if m.get("name") == geom.get("mesh")]
    if any(mesh.get(a) for a in ("scale", "refpos", "refquat")):
        raise SystemExit(f"{obj_dir.name}: the visual mesh must have no scale, refpos or refquat")
    (material,) = [m for m in tree.iter("material") if m.get("name") == geom.get("material")]
    (texture,) = [x for x in tree.iter("texture") if x.get("name") == material.get("texture")]
    return _inside(obj_dir, mesh.get("file"), "visual mesh"), _inside(obj_dir, texture.get("file"), "texture")


def rigid(matrix: list) -> np.ndarray:
    T = np.asarray(matrix, dtype=float)
    R = T[:3, :3]
    ok = (
        T.shape == (4, 4)
        and np.allclose(T[3], [0, 0, 0, 1])
        and np.allclose(R @ R.T, np.eye(3), atol=1e-6)
        and np.isclose(np.linalg.det(R), 1.0, atol=1e-6)
    )
    if not ok:
        raise SystemExit(f"T_asset_bop is not a rigid transform: {matrix}")
    return T


def icp(src: np.ndarray, dst: np.ndarray, T: np.ndarray, iterations: int = 100) -> np.ndarray:
    tree = cKDTree(dst)
    R, t = T[:3, :3].copy(), T[:3, 3].copy()
    for _ in range(iterations):
        p = src @ R.T + t
        q = dst[tree.query(p)[1]]
        mp, mq = p.mean(0), q.mean(0)
        U, _, Vt = np.linalg.svd((p - mp).T @ (q - mq))
        D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
        dR = Vt.T @ D @ U.T
        R, t = dR @ R, dR @ (t - mp) + mq
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = R, t
    return out


def distances_mm(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return 1e3 * np.concatenate([cKDTree(b).query(a)[0], cKDTree(a).query(b)[0]])


def colours(mesh: trimesh.Trimesh, points: np.ndarray, faces: np.ndarray, image: Image.Image) -> np.ndarray:
    """RGB of ``image`` at surface ``points`` on ``faces``, by the mesh's per-vertex UVs."""
    bary = trimesh.triangles.points_to_barycentric(mesh.triangles[faces], points)
    uv = (mesh.visual.uv[mesh.faces[faces]] * bary[:, :, None]).sum(1)
    img = np.asarray(image.convert("RGB"), dtype=float)
    h, w = img.shape[:2]
    x = np.clip(np.round((uv[:, 0] % 1) * (w - 1)), 0, w - 1).astype(int)
    y = np.clip(np.round((1 - uv[:, 1] % 1) * (h - 1)), 0, h - 1).astype(int)
    return img[y, x]


def compare(src, src_rgb, dst, dst_rgb, T) -> dict:
    """Surface distance and colour difference of ``src`` mapped by ``T`` against ``dst``."""
    p = src @ T[:3, :3].T + T[:3, 3]
    d = distances_mm(p, dst)
    nearest = cKDTree(dst).query(p)[1]
    colour = np.abs(src_rgb - dst_rgb[nearest]).mean(1)
    return {"distance": d, "colour_median": float(np.median(colour))}


def audit(archive: zipfile.ZipFile, workdir: Path, obj_dir: Path) -> dict:
    meta = yaml.safe_load((obj_dir / "meta.yaml").read_text())
    bop_id = meta["dataset"]["ycbv"]["bop_id"]
    if BOP_YCB_NAMES.get(bop_id) != meta["dataset"]["ycbv"]["ycb_name"]:
        raise SystemExit(f"{obj_dir.name}: bop_id {bop_id} is {BOP_YCB_NAMES.get(bop_id)}, not the declared ycb_name")
    T = rigid(meta["frames"]["T_asset_bop"])

    ply, png = f"models/obj_{bop_id:06d}.ply", f"models/obj_{bop_id:06d}.png"
    archive.extract(ply, workdir)
    archive.extract(png, workdir)  # the PLY names its texture; trimesh resolves it beside the PLY
    bop = trimesh.load(workdir / ply, process=False)
    bop.apply_scale(1e-3)
    mesh_path, texture_path = visual_mesh(obj_dir, meta)
    # The OBJ carries UVs but no material; the model file names its texture.
    ours = trimesh.load(mesh_path, force="mesh", process=False, skip_materials=True)

    src, src_faces = trimesh.sample.sample_surface_even(bop, SAMPLES, seed=0)
    dst, dst_faces = trimesh.sample.sample_surface_even(ours, SAMPLES, seed=1)
    src_rgb = colours(bop, src, src_faces, bop.visual.material.image)
    with Image.open(texture_path) as image:
        dst_rgb = colours(ours, dst, dst_faces, image)
    measured = compare(src, src_rgb, dst, dst_rgb, T)
    declared, colour_median = measured["distance"], measured["colour_median"]
    flips = {}
    for axis in "xyz":
        F = np.eye(4)
        F[:3, :3] = Rotation.from_euler(axis, 180, degrees=True).as_matrix()
        m = compare(src, src_rgb, dst, dst_rgb, F @ T)
        flips[f"180_about_{axis}"] = {
            "p99_mm": round(float(np.percentile(m["distance"], 99)), 2),
            "colour_median": round(m["colour_median"], 1),
        }
    fit = icp(src, dst, T)
    residual = fit @ np.linalg.inv(T)
    residual_deg = float(np.degrees(np.arccos(np.clip((np.trace(residual[:3, :3]) - 1) / 2, -1, 1))))
    residual_mm = float(1e3 * np.linalg.norm(residual[:3, 3]))
    p99 = float(np.percentile(declared, 99))

    bop_in_asset = bop.vertices @ T[:3, :3].T + T[:3, 3]
    return {
        "name": meta["name"],
        "bop_id": bop_id,
        "ycb_name": meta["dataset"]["ycbv"]["ycb_name"],
        "source_members": {
            ply: hashlib.sha256(archive.read(ply)).hexdigest(),
            png: hashlib.sha256(archive.read(png)).hexdigest(),
        },
        "T_asset_bop": T.tolist(),
        "surface_distance_mm": {
            "mean": round(float(declared.mean()), 2),
            "p99": round(p99, 2),
            "max": round(float(declared.max()), 2),
        },
        "colour_median": round(colour_median, 1),
        # The declared transform turned 180 degrees about each asset axis: what a flipped
        # frame would measure.
        "flipped": flips,
        "best_fit_residual": {"rotation_deg": round(residual_deg, 3), "translation_mm": round(residual_mm, 3)},
        # Extents along the asset frame's axes, metres.
        "asset_extent_m": [round(float(v), 4) for v in ours.extents],
        "bop_extent_in_asset_axes_m": [round(float(v), 4) for v in np.ptp(bop_in_asset, axis=0)],
        "pass": p99 <= MAX_P99_MM
        and colour_median <= MAX_COLOUR_MEDIAN
        and residual_deg <= MAX_RESIDUAL_DEG
        and residual_mm <= MAX_RESIDUAL_MM,
    }


def main(record_path: Path | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--archive", type=Path, help="A local copy of the pinned ycbv_models.zip")
    parser.add_argument("--cache", type=Path, default=Path.home() / ".cache" / "prl_assets")
    parser.add_argument("--out", type=Path, default=Path("artifacts/ycbv_audit.json"))
    parser.add_argument(
        "--record", action="store_true", help="Record the current catalog audit (checkout wrapper only)"
    )
    parser.add_argument("--fetch-only", action="store_true", help="Download and verify the archive, then exit")
    args = parser.parse_args()
    if args.record and record_path is None:
        parser.error("--record requires the repository scripts/audit_ycbv.py wrapper")

    try:
        path = verify_archive(args.archive) if args.archive else fetch_archive(args.cache)
    except (OSError, ValueError) as error:
        parser.exit(1, f"YCB-V source unavailable: {error}\n")
    if args.fetch_only:
        print(f"Verified {path}")
        return 0

    obj_dirs = sorted(meta.parent for meta in YCBV_DIR.rglob("meta.yaml"))
    with zipfile.ZipFile(path) as archive, tempfile.TemporaryDirectory() as tmp:
        results = [audit(archive, Path(tmp), d) for d in obj_dirs]

    report = {
        "source": {"url": ARCHIVE_URL, "sha256": ARCHIVE_SHA256},
        "method": {
            "samples_per_mesh": SAMPLES,
            "distance": "symmetric nearest-neighbour surface distance, both sample sets pooled",
            "tolerances": {
                "p99_mm": MAX_P99_MM,
                "colour_median": MAX_COLOUR_MEDIAN,
                "residual_rotation_deg": MAX_RESIDUAL_DEG,
                "residual_translation_mm": MAX_RESIDUAL_MM,
            },
        },
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "trimesh": trimesh.__version__,
            "pillow": Image.__version__,
        },
        "objects": results,
    }
    ok = sorted(r["bop_id"] for r in results) == sorted(BOP_YCB_NAMES) and all(r["pass"] for r in results)
    report["complete_collection"] = sorted(r["bop_id"] for r in results) == sorted(BOP_YCB_NAMES)
    write_json(args.out, report)
    for r in results:
        d, b = r["surface_distance_mm"], r["best_fit_residual"]
        print(
            f"{'PASS' if r['pass'] else 'FAIL'} {r['name']:12s} bop {r['bop_id']:2d}  "
            f"p99 {d['p99']:.2f} mm  max {d['max']:.2f} mm  colour {r['colour_median']:.1f}  "
            f"best fit {b['rotation_deg']:.3f} deg / {b['translation_mm']:.3f} mm"
        )
    print(f"Wrote {args.out}")
    if args.record:
        if not ok:
            print(f"Not recording: some assets fail; {record_path} is unchanged", file=sys.stderr)
        else:
            write_json(record_path, report)
            print(f"Recorded {record_path}")
    return 0 if ok else 1


def write_json(path: Path, report: dict) -> None:
    """Replace ``path`` atomically, leaving the old file and no temporary on failure."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, suffix=".json.part")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(report, f, indent=2, allow_nan=False)
            f.write("\n")
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


if __name__ == "__main__":
    sys.exit(main())
