# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""Every object's collision geometry has the shape of its visual mesh.

MuJoCo collides the collision geoms, not the mesh people see, so a model whose collision
geometry is too small, too large, sunk, or solid where the object is open behaves unlike
the object it shows: a bowl that nothing can be put into, a bottle grasped 17 mm short of
its neck (personalrobotics/prl_assets#30). ``prl_assets.authoring.check`` measures each
compiled model against its visual mesh; the tolerances and how an object may widen one are
in ``docs/schema.md``.

Writes artifacts/collision_fit.json (override the directory with PRL_ASSETS_ARTIFACTS):
every object's measurements and the failures found, when every object was checked.
"""

import json
import os
from dataclasses import asdict
from pathlib import Path

import numpy as np
import pytest

authoring = pytest.importorskip("prl_assets.authoring")
mujoco = pytest.importorskip("mujoco")

NAMES = sorted(authoring.object_dirs())
_REPORTS = {}


@pytest.fixture(scope="module", autouse=True)
def _write_artifact():
    yield
    if set(_REPORTS) != set(NAMES):  # a partial run (-k) must not replace a full artifact
        return
    out = Path(os.environ.get("PRL_ASSETS_ARTIFACTS", "artifacts"))
    out.mkdir(parents=True, exist_ok=True)
    rows = {name: {"ok": r.ok, "failures": r.failures, **asdict(r.fit)} for name, r in sorted(_REPORTS.items())}
    (out / "collision_fit.json").write_text(json.dumps(rows, indent=2, allow_nan=False) + "\n")


@pytest.mark.parametrize("name", NAMES)
def test_collision_geometry_matches_the_visual_mesh(name):
    report = authoring.check(name)
    _REPORTS[name] = report
    assert report.ok, str(report)


# Negative controls: each failure the check exists to catch, made deliberately on an object
# that passes, must fail it. Without them a measuring bug could pass every object.


def _edited(name, edit):
    obj = authoring.load(name)
    spec = obj.spec.copy()
    edit(spec)
    return authoring.from_spec(spec, obj)


def _collision(spec):
    return [g for g in spec.geoms if g.contype or g.conaffinity]


def test_an_undersized_collision_box_fails():
    def shrink(spec):
        (g,) = _collision(spec)
        g.size = [g.size[0] - 0.005, g.size[1], g.size[2]]  # 10 mm narrower

    report = authoring.check(_edited("notebook", shrink))
    assert any("(visual_outside)" in f for f in report.failures), str(report)


def test_an_oversized_collision_box_fails():
    def grow(spec):
        (g,) = _collision(spec)
        g.size = [g.size[0] + 0.01, g.size[1], g.size[2]]  # 20 mm wider

    report = authoring.check(_edited("notebook", grow))
    assert any("(collision_excess)" in f for f in report.failures), str(report)


def test_collision_geometry_raised_off_the_support_fails():
    def lift(spec):
        for g in _collision(spec):
            g.pos = [g.pos[0], g.pos[1], g.pos[2] + 0.003]

    report = authoring.check(_edited("notebook", lift))
    assert any("(sink)" in f for f in report.failures), str(report)


def test_a_filled_container_fails():
    """A solid plug inside an open object stops a dropped object short of the floor."""
    obj = authoring.load("plastic_bowl")
    spec = obj.spec.copy()
    body = next(iter(spec.worldbody.bodies))
    (g,) = authoring.visual_geoms(obj)
    rim = np.ptp(authoring.geom_mesh(obj, g).vertices[:, 2])
    body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[0.03, rim / 2, 0],
        pos=[0, 0, rim / 2],
        group=authoring.COLLISION_GROUP,
        contype=1,
        conaffinity=1,
        mass=0,
    )
    report = authoring.check(authoring.from_spec(spec, obj))
    assert any("(floor_gap)" in f for f in report.failures), str(report)


def test_a_stale_size_in_the_metadata_fails():
    """Grasp and place planning read geometric_properties, not the geoms."""
    obj = authoring.load("notebook")
    gp = obj.meta["geometric_properties"]
    meta = {**obj.meta, "geometric_properties": {**gp, "size": [gp["size"][0] + 0.005, *gp["size"][1:]]}}
    report = authoring.check(authoring.from_spec(obj.spec.copy(), obj, meta))
    assert any("geometric_properties.size" in f for f in report.failures), str(report)
    meta = {**obj.meta, "dimensions": [obj.meta["dimensions"][0] + 0.005, *obj.meta["dimensions"][1:]]}
    report = authoring.check(authoring.from_spec(obj.spec.copy(), obj, meta))
    assert any("dimensions" in f for f in report.failures), str(report)


def test_a_tolerance_must_state_its_reason():
    obj = authoring.load("notebook")
    meta = {
        **obj.meta,
        "geometric_properties": {**obj.meta["geometric_properties"], "collision_tolerance_mm": {"sink": 5}},
    }
    with pytest.raises(ValueError, match="reason"):
        authoring.check(authoring.from_spec(obj.spec.copy(), obj, meta))


def test_the_readme_new_object_example_builds_an_object_that_passes(tmp_path):
    """The README's worked example, run as written on the glass's visual mesh in a scratch
    directory: writing a model and metadata, fitting a shell, rewriting the collision
    geoms, and checking the result."""
    readme = (Path(__file__).parent.parent / "README.md").read_text()
    section = readme[readme.index("For a cup, steps 2 to 4") :]
    code = section[
        section.index("```python\n") + len("```python\n") : section.index("\n```", section.index("```python"))
    ]
    cup = tmp_path / "paper_cup"
    cup.mkdir()
    glass = authoring.load("plastic_glass")
    mesh = glass.spec.meshes[0].file
    (cup / "paper_cup_visual.obj").write_bytes((glass.dir / mesh).read_bytes())
    target = 'Path("src/prl_assets/objects/paper_cup")'
    assert target in code, "the README example's object path changed; update this test"
    namespace = {}
    exec(code.replace(target, f"Path({str(cup)!r})"), namespace)
    report = authoring.check(cup)
    assert report.ok, str(report)
    assert report.fit.floor_gap is not None, "the example's cup must be checked as an open object"
