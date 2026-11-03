#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Sweep collider recipes for the over-decomposed YCB-V objects (#42).

One unit = one object x one recipe. Each unit builds the object exactly as the importer
would (import_ycbv._build with a recipe override), then records part counts, the
authoring.check outcome, the rendered aperture occupancy, and contacts in an 8-copy pile.
Results are written atomically per unit to OUT/<object>/<setting>.json; existing results
are skipped, so the sweep resumes. Each unit peaks at about 5 GB (the pitcher at the
old 2 mm recipe); size --workers from that. CoACD's output depends on the platform, so
run where the colliders are rebuilt (Linux x86-64, locked dependencies):

    MUJOCO_GL=egl uv run --locked --all-extras python scripts/collider_sweep.py \
        --archive ycbv_models.zip --out runs/sweep --workers 14 [--only mug ...] [--settings hull ...]

The bridge distance runs from the collider's outer surface (samples inside another part
are internal interfaces) to the scan's surface: how far a hull spans a concavity, which
authoring's collision_excess, measured against the visual hull, cannot show.
"""

import argparse
import datetime
import hashlib
import importlib.util
import json
import os
import sys
import tempfile
import time
import traceback
import zipfile
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from pathlib import Path

OBJECTS = [
    "pitcher_base",
    "extra_large_clamp",
    "large_clamp",
    "bleach_cleanser",
    "ycb_bowl",
    "mug",
    "power_drill",
    "mustard_bottle",
    "scissors",
    "banana",
]
THRESHOLDS = [0.01, 0.02, 0.03, 0.05, 0.08]
CAPS = [-1, 32, 16, 8]


def settings():
    yield "current", None
    yield "hull", {"collision": "hull"}
    for t in THRESHOLDS:
        for c in CAPS:
            yield f"t{t}-c{c}", {"collision": "decomposition", "decomposition": {"threshold": t, "max_convex_hull": c}}
    # Second round: finer thresholds (large_clamp's jaw gap) and no decimation
    # (decimated pitcher parts fall short of the surface).
    for t in (0.005, 0.0075):
        for c in (-1, 64, 32):
            yield f"t{t}-c{c}", {"collision": "decomposition", "decomposition": {"threshold": t, "max_convex_hull": c}}
    for t in (0.01, 0.02, 0.03, 0.05):
        for c in (-1, 32, 16):
            yield (
                f"t{t}-c{c}-nodec",
                {
                    "collision": "decomposition",
                    "decomposition": {"threshold": t, "max_convex_hull": c, "decimate": False},
                },
            )


def log(msg):
    print(f"{datetime.datetime.now().isoformat(timespec='seconds')} {msg}", flush=True)


def write_atomic(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=".", suffix=".part")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(value, f, indent=2, allow_nan=False)
            f.write("\n")
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def unit(name, label, override, archive, repo):
    import resource

    import mujoco
    import numpy as np
    import yaml

    from prl_assets import YCBV_OBJECTS_DIR, authoring, import_ycbv, verify_ycbv

    spec = importlib.util.spec_from_file_location("collider_baseline", repo / "scripts" / "collider_baseline.py")
    baseline = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(baseline)

    bop_id = {import_ycbv.canonical_name(i): i for i in import_ycbv.BOP_YCB_NAMES}[name]
    recipe = dict(import_ycbv.RECIPES[bop_id])
    if override:
        recipe = {k: v for k, v in recipe.items() if k not in ("collision", "decomposition")} | override
    started = time.monotonic()
    row = dict(object=name, setting=label, recipe=recipe, pid=os.getpid())
    with tempfile.TemporaryDirectory() as tmp, zipfile.ZipFile(archive) as z:
        stage = Path(tmp) / name
        stage.mkdir()
        try:
            built = import_ycbv._build(z, stage, bop_id, recipe)
        except ValueError as error:
            row.update(passed_check=False, failure=str(error), seconds=time.monotonic() - started)
            row["peak_rss_mb"] = _rss_mb(resource)
            return row
        (stage / ".meta.yaml.pending").rename(stage / "meta.yaml")
        row.update(passed_check=True, parts=built["hulls"], fit=built["fit"])
        if label == "current":
            committed = YCBV_OBJECTS_DIR / name

            def digests(d):
                return {
                    p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in d.iterdir()
                    if p.suffix in (".obj", ".png", ".xml")
                }

            ours, theirs = digests(stage), digests(committed)
            row["reproduces_committed"] = dict(
                identical=sorted(f for f in ours if theirs.get(f) == ours[f]),
                differ=sorted(f for f in ours if f in theirs and theirs[f] != ours[f]),
                only_rebuilt=sorted(set(ours) - set(theirs)),
                only_committed=sorted(set(theirs) - set(ours)),
            )
        model = mujoco.MjModel.from_xml_path(str(stage / f"{name}.xml"))
        row["hull_vertices"] = int(
            sum(model.mesh_graph[model.mesh_graphadr[m]] for m in range(model.nmesh) if model.mesh_graphadr[m] >= 0)
        )
        # How far the collider's surface lies from the scan's actual surface: a hull
        # bridging a concavity (the inside of a banana's curve, a cavity) shows here,
        # while authoring's collision_excess, measured against the visual hull, does not.
        import trimesh

        visual = trimesh.load(stage / f"{name}_visual.obj", force="mesh", process=False)
        parts = [trimesh.load(p, force="mesh") for p in sorted(stage.glob("collision_*.obj"))]
        samples, owner = [], []
        whole = trimesh.util.concatenate(parts)
        points, faces = trimesh.sample.sample_surface(whole, 20000, seed=0)
        part_of_face = np.repeat(np.arange(len(parts)), [len(m.faces) for m in parts])
        # Only the collider's outer surface: a sample inside another part (by more than
        # 0.5 mm) lies on an internal interface between parts.
        inside = np.zeros(len(points), dtype=bool)
        for j, m in enumerate(parts):
            normals, origins = m.face_normals, m.triangles[:, 0]
            depth = -np.einsum("ij,kj->ki", normals, points) + np.einsum("ij,ij->i", normals, origins)
            inside |= (depth.min(axis=1) > 5e-4) & (part_of_face[faces] != j)
        samples = points[~inside]
        from scipy.spatial import cKDTree

        surface = trimesh.sample.sample_surface_even(visual, 200000, seed=1)[0]
        outside = 1e3 * cKDTree(surface).query(samples)[0]  # unsigned; ~1 mm sampling floor
        row["bridge_mm"] = dict(
            p50=float(np.percentile(outside, 50)),
            p95=float(np.percentile(outside, 95)),
            p99=float(np.percentile(outside, 99)),
            max=float(outside.max()),
        )
        renders = Path(tmp) / "renders"
        renders.mkdir()
        projections = verify_ycbv._projection(stage, z, Path(tmp), renders)
        occupied = [p["collision_aperture_occupied_fraction"] for p in projections]
        row["aperture_occupied"] = occupied
        row["aperture_ok"] = all(o is None or o <= 0.1 for o in occupied)
        row["silhouette_iou_min"] = min(p["silhouette_iou"] for p in projections)
        pile = baseline.step_cost(baseline.scene(stage, baseline.COPIES), 0, 1000, 1)
        row["pile"] = dict(
            mean_contacts=pile["mean_contacts"],
            peak_contacts=pile["peak_contacts"],
            us_per_step=pile["us_per_step"]["median"],
            arena_overflows=pile["arena_overflows"],
        )
        meta = yaml.safe_load((stage / "meta.yaml").read_text())
        row["dimensions"] = meta["dimensions"]
    row["seconds"] = time.monotonic() - started
    row["peak_rss_mb"] = _rss_mb(resource)
    return row


def _rss_mb(resource):
    rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return rss / 1024 / 1024 if sys.platform == "darwin" else rss / 1024  # bytes on macOS, KiB on Linux


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--archive", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--workers", type=int, default=1)
    p.add_argument("--only", nargs="*")
    p.add_argument("--settings", nargs="*")
    args = p.parse_args()
    repo = Path.cwd()
    units = [
        (n, label, o)
        for n in (args.only or OBJECTS)
        for label, o in settings()
        if not args.settings or label in args.settings
    ]
    todo = [u for u in units if not (args.out / u[0] / f"{u[1]}.json").exists()]
    log(
        f"start: {len(units)} units, {len(units) - len(todo)} already done, {args.workers} workers, "
        f"load {os.getloadavg()}"
    )
    done = failed = 0
    last = time.monotonic()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        pending = {pool.submit(unit, n, label, o, args.archive, repo): (n, label) for n, label, o in todo}
        while pending:
            finished, _ = wait(pending, timeout=300, return_when=FIRST_COMPLETED)
            for f in finished:
                n, label = pending.pop(f)
                try:
                    row = f.result()
                except Exception:  # a crashed unit is an operational failure, recorded separately
                    failed += 1
                    write_atomic(
                        args.out / "_failures" / f"{n}-{label}.json",
                        dict(object=n, setting=label, error=traceback.format_exc()),
                    )
                    log(f"ERROR {n} {label}")
                    continue
                write_atomic(args.out / n / f"{label}.json", row)
                done += 1
                log(
                    f"[{done + failed}/{len(todo)}] {n} {label}: check {row['passed_check']} parts {row.get('parts')} "
                    f"aperture {row.get('aperture_ok')} pile {row.get('pile', {}).get('peak_contacts')} "
                    f"{row['seconds']:.0f}s rss {row['peak_rss_mb']:.0f}MB"
                )
            if time.monotonic() - last > 300 or not finished:
                log(f"progress: {done} done, {failed} failed, {len(pending)} queued or running, load {os.getloadavg()}")
                last = time.monotonic()
    log(f"done: {done} ok, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
