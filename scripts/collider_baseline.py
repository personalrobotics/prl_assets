#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Measure what every object's collision model costs, before changing any recipe (#38).

For each object: the collider's composition (geoms by type, convex mesh parts, their
vertices and faces), how long its model file takes to load, and the simulation cost
of one copy resting on a plane and of eight copies dropped together. Step costs
include a plane-only scene as the fixed overhead. Solver settings match the seeded
scene in prl_assets.verify_ycbv.

Timings belong to one host: run on a quiet machine as a single process, and compare
results only with runs from the same host. Each timing is repeated and reported as a
median with its spread; none is a pass/fail threshold.

Usage:
    uv run --locked --extra authoring python scripts/collider_baseline.py --out runs/collider_baseline.json
    uv run --locked --extra authoring python scripts/collider_baseline.py --quick mug  # few repeats, smoke test
"""

from __future__ import annotations

import argparse
import datetime
import importlib.metadata
import json
import os
import platform
import socket
import subprocess
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

from prl_assets import authoring

ROOT = Path(__file__).resolve().parent.parent
TIMESTEP = 0.002
COPIES = 8
# The default 16 MB arena overflows for a pile of finely decomposed objects, and MuJoCo
# then drops contacts, which would make the timing meaningless. Overflows are recorded.
ARENA_BYTES = 1 << 30


def log(message: str) -> None:
    print(f"{datetime.datetime.now().isoformat(timespec='seconds')} {message}", flush=True)


def environment() -> dict:
    def git(*args):
        return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()

    cpu = platform.processor()
    if Path("/proc/cpuinfo").exists():
        names = [
            line.split(":", 1)[1].strip()
            for line in Path("/proc/cpuinfo").read_text().splitlines()
            if line.startswith("model name")
        ]
        cpu = names[0] if names else cpu
    return dict(
        commit=git("rev-parse", "HEAD"),
        dirty=bool(git("status", "--porcelain", "--untracked-files=no")),
        host=socket.gethostname(),
        cpu=cpu,
        cpus=os.cpu_count(),
        load_average=list(os.getloadavg()),
        python=platform.python_version(),
        packages={p: importlib.metadata.version(p) for p in ("mujoco", "numpy")},
        command=sys.argv,
        started=datetime.datetime.now().isoformat(timespec="seconds"),
    )


def composition(obj: authoring.Compiled) -> dict:
    m = obj.model
    geoms = authoring.collision_geoms(obj)
    types = {}
    mesh_vertices = hull_vertices = hull_faces = 0
    meshes = set()
    for g in geoms:
        kind = mujoco.mjtGeom(m.geom_type[g]).name.removeprefix("mjGEOM_").lower()
        types[kind] = types.get(kind, 0) + 1
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = int(m.geom_dataid[g])
            meshes.add(mesh)
            mesh_vertices += int(m.mesh_vertnum[mesh])
            # MuJoCo collides a mesh as the convex hull it builds; mesh_graph starts
            # with that hull's vertex and face counts.
            adr = m.mesh_graphadr[mesh]
            if adr >= 0:
                hull_vertices += int(m.mesh_graph[adr])
                hull_faces += int(m.mesh_graph[adr + 1])
    files = [obj.dir / f for f in {s.file for s in obj.spec.meshes if s.file}]
    collider_files = [f for f in files if "collision" in f.name]
    return dict(
        collision_geoms=len(geoms),
        by_type=types,
        mesh_parts=len(meshes),
        mesh_vertices=mesh_vertices,
        hull_vertices=hull_vertices,
        hull_faces=hull_faces,
        collider_file_bytes=sum(f.stat().st_size for f in collider_files if f.exists()),
        collider_files=len(collider_files),
    )


def spread(samples: list[float]) -> dict:
    a = np.asarray(samples)
    return dict(
        median=float(np.median(a)),
        q1=float(np.percentile(a, 25)),
        q3=float(np.percentile(a, 75)),
        min=float(a.min()),
        max=float(a.max()),
        n=len(a),
    )


def load_time(xml: Path, repeats: int) -> dict:
    samples = []
    for _ in range(repeats):
        t = time.perf_counter()
        mujoco.MjModel.from_xml_path(str(xml))
        samples.append(1e3 * (time.perf_counter() - t))
    return spread(samples)


def scene(obj_dir: Path | None, copies: int) -> mujoco.MjModel:
    spec = mujoco.MjSpec()
    spec.option.timestep = TIMESTEP
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10
    spec.memory = ARENA_BYTES
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1])
    sources = []
    if obj_dir is not None:
        meta = yaml.safe_load((obj_dir / "meta.yaml").read_text())
        for i in range(copies):
            source = mujoco.MjSpec.from_file(str(obj_dir / meta["mujoco"]["xml_path"]))
            sources.append(source)  # attached bodies need their source spec alive
            top = next(iter(source.worldbody.bodies))
            body = spec.worldbody.add_frame().attach_body(top, f"{i}/", "")
            if copies > 1:
                # Dropped in a tight column, so they land on and against each other.
                body.pos = [0.03 * (i % 2), 0.03 * (i // 2 % 2), body.pos[2] + 0.12 * (i + 1)]
                yaw = 0.7 * i
                body.quat = [np.cos(yaw / 2), 0.2, 0, np.sin(yaw / 2)]
    model = spec.compile()
    return model


def step_cost(model: mujoco.MjModel, settle: int, steps: int, repeats: int) -> dict:
    samples, contacts, peak, overflows = [], [], 0, 0
    for _ in range(repeats):
        data = mujoco.MjData(model)
        for _ in range(settle):
            mujoco.mj_step(model, data)
        ncon = 0
        t = time.perf_counter()
        for _ in range(steps):
            mujoco.mj_step(model, data)
            ncon += data.ncon
            peak = max(peak, data.ncon)
        samples.append(1e6 * (time.perf_counter() - t) / steps)
        contacts.append(ncon / steps)
        overflows += sum(
            int(data.warning[w].number)
            for w in (mujoco.mjtWarning.mjWARN_CONTACTFULL, mujoco.mjtWarning.mjWARN_CNSTRFULL)
        )
        if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
            raise FloatingPointError("simulation diverged")
    return dict(
        us_per_step=spread(samples),
        mean_contacts=float(np.mean(contacts)),
        peak_contacts=peak,
        arena_overflows=overflows,
        settle_steps=settle,
        timed_steps=steps,
    )


def write_json(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    authoring._write_atomic(path, json.dumps(report, indent=2) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("objects", nargs="*", help="Object names (default: all)")
    parser.add_argument("--out", type=Path, default=ROOT / "artifacts" / "collider_baseline.json")
    parser.add_argument("--quick", action="store_true", help="Few repeats: a smoke test, not a measurement")
    args = parser.parse_args()

    load_repeats, step_repeats, steps = (2, 1, 100) if args.quick else (15, 7, 1000)
    dirs = authoring.object_dirs()
    names = args.objects or sorted(dirs)
    unknown = [n for n in names if n not in dirs]
    if unknown:
        print(f"Unknown objects: {', '.join(unknown)}", file=sys.stderr)
        return 2

    report = dict(
        complete=False,
        environment=environment(),
        method=dict(
            arena_bytes=ARENA_BYTES,
            timestep_s=TIMESTEP,
            integrator="implicitfast",
            cone="elliptic",
            impratio=10,
            copies=COPIES,
            load_repeats=load_repeats,
            step_repeats=step_repeats,
            quick=args.quick,
            resting="one copy at its model-file pose on a plane; timed after settling",
            pile=f"{COPIES} copies dropped in a column onto a plane; timed while they land and settle",
        ),
        plane_only=step_cost(scene(None, 0), 0, steps, step_repeats),
        objects={},
    )
    log(
        f"start: {len(names)} objects, commit {report['environment']['commit'][:7]}, "
        f"load {report['environment']['load_average']}"
    )
    started = time.monotonic()
    for i, name in enumerate(names, 1):
        obj = authoring.load(name)
        row = dict(
            composition=composition(obj), load_ms=load_time(obj.dir / obj.meta["mujoco"]["xml_path"], load_repeats)
        )
        row["resting"] = step_cost(scene(obj.dir, 1), 250, steps, step_repeats)
        row["pile"] = step_cost(scene(obj.dir, COPIES), 0, 2 * steps, step_repeats)
        report["objects"][name] = row
        write_json(args.out, report)  # incomplete until the final write
        c = row["composition"]
        log(
            f"[{i}/{len(names)}] {name}: {c['collision_geoms']} geoms, {c['hull_vertices']} hull vertices, "
            f"load {row['load_ms']['median']:.1f} ms, resting {row['resting']['us_per_step']['median']:.1f} us/step, "
            f"pile {row['pile']['us_per_step']['median']:.1f} us/step, {time.monotonic() - started:.0f} s elapsed"
        )
    report["complete"] = True
    report["environment"]["finished"] = datetime.datetime.now().isoformat(timespec="seconds")
    write_json(args.out, report)
    log(f"done: wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
