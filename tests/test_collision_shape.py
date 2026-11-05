# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""``prl_assets.shape.collision_shape`` describes each object as MuJoCo collides it, so
sstsr's ``StablePlacer.place`` puts it where it stays.

Two oracles, neither of them the converter: each part's support function must match the
compiled geom's own mesh (``authoring.geom_mesh``) in every direction, and the most stable
rest ``place`` returns must hold when MuJoCo drops the object onto a floor in it.

Writes artifacts/collision_shape.json (override the directory with PRL_ASSETS_ARTIFACTS):
each object's parts, its rest count and the first rest's variant, the support error, and
how far the object tilted and moved in height while settling, when every object was checked.
Reproduce with ``uv run --locked --all-extras pytest tests/test_collision_shape.py -q``.
"""

import json
import os
from pathlib import Path

import numpy as np
import pytest

authoring = pytest.importorskip("prl_assets.authoring")
shape_module = pytest.importorskip("prl_assets.shape")
placement = pytest.importorskip("tsr.placement")
import mujoco  # noqa: E402

NAMES = sorted(authoring.object_dirs())
_ROWS = {}

# The meshes geom_mesh builds inscribe a cylinder in 64 sides and a sphere in a level-4
# icosphere: a support short by r(1 - cos(pi/64)), 0.12 % of the radius, is tessellation.
SUPPORT_TOLERANCE_MM = 0.5
SETTLE_SECONDS = 1.5
DROP = 0.001  # m above the predicted rest, so the object starts free of the floor
TILT_TOLERANCE_DEG = 2.0
HEIGHT_TOLERANCE_MM = 2.0


@pytest.fixture(scope="module", autouse=True)
def _write_artifact():
    yield
    if set(_ROWS) != set(NAMES):  # a partial run (-k) must not replace a full artifact
        return
    out = Path(os.environ.get("PRL_ASSETS_ARTIFACTS", "artifacts"))
    out.mkdir(parents=True, exist_ok=True)
    (out / "collision_shape.json").write_text(json.dumps(dict(sorted(_ROWS.items())), indent=2) + "\n")


def _directions(n: int = 256) -> np.ndarray:
    i = np.arange(n) + 0.5
    z = 1.0 - 2.0 * i / n
    phi = np.pi * (1.0 + 5**0.5) * i
    r = np.sqrt(1.0 - z * z)
    return np.column_stack([r * np.cos(phi), r * np.sin(phi), z])


def _settle(obj, T_floor_obj: np.ndarray) -> tuple[float, float]:
    """Degrees ``obj`` tilts and millimetres its origin moves vertically when dropped
    onto a floor at z = 0 from ``DROP`` above ``T_floor_obj``."""
    scene = mujoco.MjSpec()
    scene.option.timestep = 0.002
    scene.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1])
    source = obj.spec.copy()  # the scene uses it: it must outlive compile()
    body = next(iter(source.worldbody.bodies))
    if body.first_joint() is None:  # a fixture, such as a table, dropped like any object
        body.add_freejoint()
    scene.worldbody.add_frame().attach_body(body, "obj/", "")
    model = scene.compile()
    data = mujoco.MjData(model)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, T_floor_obj[:3, :3].ravel())
    data.qpos[:7] = [*T_floor_obj[:2, 3], T_floor_obj[2, 3] + DROP, *quat]
    for _ in range(round(SETTLE_SECONDS / scene.option.timestep)):
        mujoco.mj_step(model, data)
    b = 1  # the object, the scene's only body
    R = data.xmat[b].reshape(3, 3)
    tilt = np.degrees(np.arccos(np.clip(R[:, 2] @ T_floor_obj[:3, 2], -1.0, 1.0)))
    return float(tilt), float(1e3 * (data.xpos[b][2] - T_floor_obj[2, 3]))


@pytest.mark.parametrize("name", NAMES)
def test_an_object_rests_where_place_puts_it(name):
    obj = authoring.load(name)
    shape = shape_module.collision_shape(obj.model, obj.data, obj.body)

    R, p = obj.data.xmat[obj.body].reshape(3, 3), obj.data.xpos[obj.body]
    vertices = (np.vstack([authoring.geom_mesh(obj, g).vertices for g in authoring.collision_geoms(obj)]) - p) @ R
    error = max(abs(shape.support(n) - float((vertices @ n).max())) for n in _directions())

    if obj.model.nbody == 2:  # one body: its inertial frame is the centre of mass
        assert np.allclose(shape.com, obj.model.body_ipos[obj.body], atol=1e-9), f"{name}: centre of mass moved"

    rests = placement.StablePlacer(table_x=1.0, table_y=1.0).place(shape, subject=name)
    assert rests, f"{name}: place found no rest"
    tilt, height = _settle(obj, rests[0].T_ref_tsr @ rests[0].Tw_e)

    _ROWS[name] = {
        "parts": sorted(type(p).__name__ for p in shape.parts),
        "rests": len(rests),
        "first_rest": rests[0].variant,
        "support_error_mm": round(1e3 * error, 4),
        "settle_tilt_deg": round(tilt, 3),
        "settle_height_mm": round(height, 3),
    }
    assert 1e3 * error <= SUPPORT_TOLERANCE_MM, f"{name}: support differs from the geoms by {1e3 * error:.3f} mm"
    assert tilt <= TILT_TOLERANCE_DEG, f"{name}: tilted {tilt:.2f} deg settling in {rests[0].variant}"
    assert abs(height) <= HEIGHT_TOLERANCE_MM, f"{name}: settled {height:.2f} mm from its predicted height"


def test_a_geom_with_no_part_is_refused_not_dropped():
    spec = mujoco.MjSpec()
    body = spec.worldbody.add_body(name="egg")
    body.add_freejoint()
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.02, 0, 0])
    body.add_geom(name="shell", type=mujoco.mjtGeom.mjGEOM_ELLIPSOID, size=[0.02, 0.02, 0.03], pos=[0, 0, 0.03])
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with pytest.raises(ValueError, match="'shell' is a mjGEOM_ELLIPSOID"):
        shape_module.collision_shape(model, data, "egg")
    with pytest.raises(ValueError, match="no body 'chicken'"):
        shape_module.collision_shape(model, data, "chicken")
