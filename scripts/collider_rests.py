#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""How objects rest on their colliders, by sstsr and in MuJoCo (#54).

For each object: the rests ``StablePlacer.place(collision_shape(...))`` finds with at
least ``--min-margin`` degrees of tipping margin, each with its face normal (object
frame) and margin. Each such rest is then simulated, on a ground plane and on the
recycle bin's floor with the bin fixed, under Geodude's settings (2 ms, implicitfast,
elliptic cone, impratio 10): placed at rest, the object must move less than 1 mm and
0.5 deg in 2 s. An object whose name is in ``--tilt`` is also stood on its most stable
rest tilted 5 deg about a horizontal axis, and must settle back within 1 deg.

Run against a directory of objects to compare colliders, e.g. main's and a branch's:

    uv run --locked --all-extras python scripts/collider_rests.py --out artifacts/collider_rests.json \\
        [--objects-dir src/prl_assets/objects/ycbv] [names ...]
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from tsr.placement import StablePlacer

from prl_assets import OBJECTS_DIR
from prl_assets.shape import collision_shape

OBJECTS = ["pudding_box", "wood_block", "foam_brick", "bleach_cleanser", "mustard_bottle"]
BOTTLES = ["bleach_cleanser", "mustard_bottle"]
SETTLE_S = 2.0
STILL_M, STILL_DEG, UPRIGHT_DEG, TILT_DEG = 1e-3, 0.5, 1.0, 5.0


def _model_file(directory: Path) -> Path:
    return directory / yaml.safe_load((directory / "meta.yaml").read_text())["mujoco"]["xml_path"]


def _scene(xml: Path, name: str, floor: str):
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1])
    top = 0.0
    if floor == "bin":
        bin_spec = mujoco.MjSpec.from_file(str(_model_file(OBJECTS_DIR / "recycle_bin")))
        for joint in list(bin_spec.joints):
            bin_spec.delete(joint)  # fixed, as Geodude's bin is
        spec.attach(bin_spec, frame=spec.worldbody.add_frame(), prefix="bin/")
        model = spec.compile()
        g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "bin/bin_bottom")
        top = float(model.geom_pos[g][2] + model.geom_size[g][2])
    source = mujoco.MjSpec.from_file(str(xml))
    spec.attach(source, frame=spec.worldbody.add_frame(), prefix="o/")
    model = spec.compile()
    return model, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"o/{name}"), top


def _settle(model, body, R, height):
    """Place the body at rotation ``R`` with its origin ``height`` up, at rest; return
    its displacement (m) and rotation (deg) after ``SETTLE_S``, and its final rotation."""
    data = mujoco.MjData(model)
    q = model.jnt_qposadr[model.body_jntadr[body]]
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    data.qpos[q : q + 7] = [0.0, 0.0, height, w, x, y, z]
    mujoco.mj_forward(model, data)
    p0, R0 = data.xpos[body].copy(), data.xmat[body].reshape(3, 3).copy()
    for _ in range(round(SETTLE_S / model.opt.timestep)):
        mujoco.mj_step(model, data)
    R1 = data.xmat[body].reshape(3, 3).copy()
    turn = math.degrees(Rotation.from_matrix(R0.T @ R1).magnitude())
    return float(np.linalg.norm(data.xpos[body] - p0)), turn, R1


def measure(directory: Path, name: str, min_margin: float, tilt: bool) -> dict:
    xml = _model_file(directory)
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    shape = collision_shape(model, data, name)
    rests = StablePlacer(1e6, 1e6).place(shape, subject=name, min_margin_deg=min_margin)
    out = {"rests": [], "tilted": None}
    for floor in ("plane", "bin"):
        scene, body, top = _scene(xml, name, floor)
        for i, t in enumerate(rests):
            moved, turned, _ = _settle(scene, body, t.Tw_e[:3, :3], top + t.T_ref_tsr[2, 3])
            if floor == "plane":
                out["rests"].append(
                    {
                        "face_normal": [round(float(v), 4) for v in t.provenance.face_normal],
                        "margin_deg": round(math.degrees(t.stability_margin), 2),
                    }
                )
            out["rests"][i][floor] = {
                "moved_mm": round(1e3 * moved, 3),
                "turned_deg": round(turned, 3),
                "still": moved < STILL_M and turned < STILL_DEG,
            }
    if tilt and rests:
        scene, body, _ = _scene(xml, name, "plane")
        t = rests[0]
        R = Rotation.from_euler("x", TILT_DEG, degrees=True).as_matrix() @ t.Tw_e[:3, :3]
        lowest = min(float((R @ p)[2]) for p in shape.surface())
        _, _, R1 = _settle(scene, body, R, -lowest)
        # The yaw it settles at is free; only the tilt off the rest counts.
        down = R1 @ np.asarray(t.provenance.face_normal)
        off = math.degrees(math.acos(np.clip(-down[2], -1.0, 1.0)))
        out["tilted"] = {"start_deg": TILT_DEG, "settled_off_rest_deg": round(off, 3), "upright": off < UPRIGHT_DEG}
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("names", nargs="*", default=OBJECTS)
    parser.add_argument("--objects-dir", type=Path, default=OBJECTS_DIR / "ycbv")
    parser.add_argument("--min-margin", type=float, default=3.0, help="degrees")
    parser.add_argument("--tilt", nargs="*", default=BOTTLES)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    report = {
        "min_margin_deg": args.min_margin,
        "settle_s": SETTLE_S,
        "still": {"moved_mm": STILL_M * 1e3, "turned_deg": STILL_DEG},
        "objects": {n: measure(args.objects_dir / n, n, args.min_margin, n in args.tilt) for n in args.names},
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n")
    for name, row in report["objects"].items():
        still = all(r["plane"]["still"] and r["bin"]["still"] for r in row["rests"])
        tilted = row["tilted"]
        print(
            f"{name:16s} {len(row['rests'])} rests >= {args.min_margin:g} deg "
            f"{[r['margin_deg'] for r in row['rests']]}, all still: {still}"
            + (f", tilted {TILT_DEG:g} deg settles {tilted['settled_off_rest_deg']} deg off" if tilted else "")
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
