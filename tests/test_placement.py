"""Behavioral contract for placement metadata (docs/schema.md, "Placement").

Parser controls isolate malformed ingress, each writing a rejection artifact. Analytic
models, built here with known openings and tops on a turned, off-centre body, are the
measurers' oracle; they write artifacts/placement_analytic.json. The compiled-model
controls show that a declared opening or support that disagrees with the
model fails ``authoring.check``. The catalog integration shows every open object declares
an opening, and that the recycle bin's opening is the legacy containment Geodude's drop
zones read; it writes artifacts/placement.json. Reproduce
with uv run --locked --all-extras pytest tests/test_placement.py -q.
"""

import copy
import math
import os
import shutil
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from prl_assets import OBJECTS_DIR, authoring
from prl_assets.placement import Placement, PlacementError, parse
from prl_assets.ycbv_audit import write_json

ARTIFACTS = Path(os.environ.get("PRL_ASSETS_ARTIFACTS", "artifacts"))
IDENTITY = np.eye(4).tolist()


def _meta():
    return dict(
        name="probe",
        placement=dict(
            version=1,
            openings=[dict(name="top", shape="circle", radius=0.03, depth=0.08, T_asset_opening=IDENTITY)],
            supports=[dict(name="top", shape="rectangle", size=[0.2, 0.1])],
        ),
    )


def _bad(case):
    meta = _meta()
    section = meta["placement"]
    opening, support = section["openings"][0], section["supports"][0]
    if case in ["bool", "numeric_string", "nan", "zero", "negative"]:
        opening["radius"] = {"bool": True, "numeric_string": "0.03", "nan": float("nan"), "zero": 0, "negative": -1}[
            case
        ]
    elif case == "null_section":
        meta["placement"] = None
    elif case == "missing_version":
        del section["version"]
    elif case == "wrong_version":
        section["version"] = 2
    elif case == "section_key":
        section["stable_poses"] = ["upright"]
    elif case == "empty":
        section["openings"], section["supports"] = [], []
    elif case == "rests":  # how an object rests is the planner's to work out
        section["rests"] = [dict(name="upright", footprint_radius=0.04)]
    elif case == "duplicate":
        section["openings"].append(copy.deepcopy(opening))
    elif case == "name":
        opening["name"] = "Top Opening"
    elif case == "shape":
        opening["shape"] = "ellipse"
    elif case == "shape_parameter":
        opening["size"] = [0.06, 0.06]  # a circle has a radius
    elif case == "rectangle_size":
        support["size"] = [0.2, 0.1, 0.01]
    elif case == "missing_depth":
        del opening["depth"]
    elif case == "unknown_key":
        opening["mass"] = 1.0
    elif case == "reflection":
        opening["T_asset_opening"] = [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    elif case == "string_pose":
        support["T_asset_support"] = [["1", 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    elif case == "bottom_row":
        support["T_asset_support"] = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0.1, 1]]
    else:
        raise AssertionError(case)
    return meta


CASES = [
    "bool",
    "numeric_string",
    "nan",
    "zero",
    "negative",
    "null_section",
    "missing_version",
    "wrong_version",
    "section_key",
    "empty",
    "rests",
    "duplicate",
    "name",
    "shape",
    "shape_parameter",
    "rectangle_size",
    "missing_depth",
    "unknown_key",
    "reflection",
    "string_pose",
    "bottom_row",
]


def test_well_formed_placement_reads_back():
    placement = parse(_meta())
    assert placement.openings[0].size == 0.03 and placement.supports[0].size == (0.2, 0.1)
    np.testing.assert_array_equal(placement.supports[0].frame(), np.eye(4))  # omitted frame: identity
    assert parse(dict(name="probe")) is None


@pytest.mark.parametrize("case", CASES)
def test_malformed_placement_is_rejected(case):
    with pytest.raises(PlacementError) as caught:
        parse(_bad(case))
    assert "probe" in str(caught.value)
    write_json(
        ARTIFACTS / "placement_rejections" / f"{case}.json",
        dict(case=case, rejected=True, diagnostic=str(caught.value)),
    )


def test_a_constructed_placement_is_immutable_and_named_once():
    opening = parse(_meta()).openings[0]
    placement = Placement(openings=[opening])
    assert placement.openings == (opening,)
    for bad in ([opening, opening], ["top"]):
        with pytest.raises(PlacementError):
            Placement(openings=bad)


def _turn(axis, degrees):
    T = np.eye(4)
    quat = np.zeros(4)
    mujoco.mju_axisAngle2Quat(quat, np.eye(3)["xyz".index(axis)], math.radians(degrees))
    matrix = np.zeros(9)
    mujoco.mju_quat2Mat(matrix, quat)
    T[:3, :3] = matrix.reshape(3, 3)
    return T


def _at(x, y, z):
    T = np.eye(4)
    T[:3, 3] = x, y, z
    return T


# Each analytic model is built in a container frame C (z up, its floor's centre at the
# origin) standing turned 30 degrees on the world floor. The asset frame is off the
# interior's centre and turned so its y axis is vertical, as a YCB-V object's BOP frame
# can be: openings and tops are measured along its x axis and the vertical.
T_WORLD_C = _at(0.3, -0.2, 0.0) @ _turn("z", 30)
T_C_ASSET = _at(0.02, 0.01, 0.05) @ _turn("x", 90)


def _model(tmp_path, name, geoms, T_c_asset=T_C_ASSET):
    """An object directory whose collision geoms are ``(type, size, T_C_geom)``, a mesh's
    ``size`` its vertices in its own frame."""
    lines, meshes = [], []
    for i, (kind, size, T_c_geom) in enumerate(geoms):
        T = np.linalg.inv(T_c_asset) @ T_c_geom
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, T[:3, :3].flatten())
        pose = f'pos="{" ".join(map(str, T[:3, 3]))}" quat="{" ".join(map(str, quat))}"'
        if kind == "mesh":
            meshes.append(f'<mesh name="m{i}" vertex="{" ".join(map(str, np.ravel(size)))}"/>')
            lines.append(f'<geom type="mesh" mesh="m{i}" group="3" {pose}/>')
        else:
            lines.append(f'<geom type="{kind}" size="{" ".join(map(str, size))}" group="3" {pose}/>')
    T = T_WORLD_C @ T_c_asset
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, T[:3, :3].flatten())
    directory = tmp_path / name
    directory.mkdir()
    (directory / f"{name}.xml").write_text(
        f'<mujoco model="{name}"><asset>{"".join(meshes)}</asset><worldbody>'
        f'<body name="{name}" pos="{" ".join(map(str, T[:3, 3]))}" quat="{" ".join(map(str, quat))}">'
        f"<freejoint/>{''.join(lines)}</body></worldbody></mujoco>"
    )
    (directory / "meta.yaml").write_text(yaml.safe_dump(dict(name=name, mujoco=dict(xml_path=f"{name}.xml"))))
    return authoring.load(directory)


def _tray(tmp_path):
    """An open box 0.20 by 0.12 m inside, its rim 0.15 m up, its floor 0.01 m thick."""
    return _model(
        tmp_path,
        "tray",
        [
            ("box", (0.11, 0.07, 0.005), _at(0, 0, 0.005)),
            *[("box", (0.005, 0.07, 0.075), _at(sx * 0.105, 0, 0.075)) for sx in (-1, 1)],
            *[("box", (0.11, 0.005, 0.075), _at(0, sy * 0.065, 0.075)) for sy in (-1, 1)],
        ],
    )


def _cup(tmp_path):
    """Sixteen wall segments whose inner faces touch a 0.05 m circle, the rim 0.08 m up,
    on a 0.01 m floor."""
    r, half = 0.05, 0.05 * math.tan(math.pi / 16) + 0.01
    walls = [("box", (0.005, half, 0.04), _turn("z", 22.5 * k) @ _at(r + 0.005, 0, 0.04)) for k in range(16)]
    return _model(tmp_path, "cup", [("cylinder", (r + 0.01, 0.005), _at(0, 0, 0.005)), *walls])


def _frustum(top, bottom, z0, z1):
    return [[sx * x / 2, sy * y / 2, z] for (x, y), z in ((top, z1), (bottom, z0)) for sx in (-1, 1) for sy in (-1, 1)]


def _table(tmp_path, top="flared"):
    """A 0.6 by 0.4 m top 0.75 m up on one leg. The flared top widens to 0.8 by 0.6 m
    below its face, which is the face only; the round top is a disc."""
    if top == "flared":
        slab = ("mesh", _frustum((0.6, 0.4), (0.8, 0.6), -0.05, 0.0), _at(0, 0, 0.75))
    else:
        slab = ("cylinder", (0.3, 0.025), _at(0, 0, 0.725))
    return _model(tmp_path, f"table_{top}", [slab, ("box", (0.025, 0.025, 0.35), _at(0, 0, 0.35))])


def test_measurers_recover_known_openings_and_tops(tmp_path):
    """The measured sizes, depths and frames equal the models' built dimensions, in the
    asset frame of a turned, off-centre body; and a shape that does not describe the
    geometry is refused rather than approximated."""
    expected = {
        "tray": (_tray, "rectangle", [0.20, 0.12], 0.14, _at(0, 0, 0.15)),
        "cup": (_cup, "circle", 0.05, 0.07, _at(0, 0, 0.08)),
    }
    rows = {}
    for name, (build, shape, size, depth, T_c_opening) in expected.items():
        opening = authoring.measure_opening(build(tmp_path), shape)
        T = np.linalg.inv(T_C_ASSET) @ T_c_opening
        np.testing.assert_allclose(np.atleast_1d(opening.size), np.atleast_1d(size), atol=2e-5)
        assert abs(opening.depth - depth) < 2e-5
        np.testing.assert_allclose(opening.frame(), T, atol=2e-5)
        rows[name] = dict(measured=dict(size=opening.size, depth=opening.depth, frame=opening.frame().tolist()))
        rows[name]["built"] = dict(size=size, depth=depth, frame=T.tolist())
    support = authoring.measure_support(_table(tmp_path))
    T = np.linalg.inv(T_C_ASSET) @ _at(0, 0, 0.75)
    np.testing.assert_allclose(support.size, [0.6, 0.4], atol=2e-5)
    np.testing.assert_allclose(support.frame(), T, atol=2e-5)
    rows["table"] = dict(measured=dict(size=support.size, frame=support.frame().tolist()))
    rows["table"]["built"] = dict(size=[0.6, 0.4], frame=T.tolist())

    for sub in "abc":
        (tmp_path / sub).mkdir()
    refusals = {
        "tray_as_circle": lambda: authoring.measure_opening(_tray(tmp_path / "a"), "circle"),
        "cup_as_rectangle": lambda: authoring.measure_opening(_cup(tmp_path / "b"), "rectangle"),
        "round_top": lambda: authoring.measure_support(_table(tmp_path, "round")),
        # No asset axis vertical: its horizontal axes name no rectangle's sides.
        "tilted_asset": lambda: authoring.measure_support(
            _model(tmp_path / "c", "tilted", [("box", (0.3, 0.2, 0.01), _at(0, 0, 0.01))], _turn("x", 30))
        ),
    }
    for case, measure in refusals.items():
        with pytest.raises(ValueError) as caught:
            measure()
        rows[case] = dict(refused=str(caught.value))
    write_json(ARTIFACTS / "placement_analytic.json", rows)


def _copy(tmp_path, name):
    directory = tmp_path / name
    shutil.copytree(authoring.object_dirs()[name], directory)
    return directory


def _edit(directory, change):
    path = directory / "meta.yaml"
    meta = yaml.safe_load(path.read_text())
    change(meta["placement"])
    path.write_text(yaml.safe_dump(meta, sort_keys=False))


def _shift(T, axis, by):
    T = copy.deepcopy(T)
    T[axis][3] += by
    return T


WRONG = {
    # A bin opening 1 cm wider than its walls: a drop zone would reach over the rim.
    "recycle_bin/opening_size": ("recycle_bin", lambda p: p["openings"][0].update(size=[0.248, 0.238])),
    "recycle_bin/opening_depth": ("recycle_bin", lambda p: p["openings"][0].update(depth=0.70)),
    "recycle_bin/opening_frame": (
        "recycle_bin",
        lambda p: p["openings"][0].update(T_asset_opening=_shift(p["openings"][0]["T_asset_opening"], 0, 0.01)),
    ),
    # A round bowl declared rectangular: the rectangle's corners lie outside its wall.
    "plastic_bowl/opening_shape": (
        "plastic_bowl",
        lambda p: (p["openings"][0].pop("radius"), p["openings"][0].update(shape="rectangle", size=[0.16, 0.16])),
    ),
    "ada_table/support_size": ("ada_table", lambda p: p["supports"][0].update(size=[1.9, 0.742])),
}


@pytest.mark.parametrize("case", sorted(WRONG))
def test_a_wrong_placement_fails_against_the_compiled_model(tmp_path, case):
    name, change = WRONG[case]
    directory = _copy(tmp_path, name)
    assert authoring.check(directory).ok
    _edit(directory, change)
    report = authoring.check(directory)
    placement_failures = [f for f in report.failures if "placement" in f]
    assert placement_failures, report.failures
    write_json(
        ARTIFACTS / "placement_rejections" / f"model_{case.replace('/', '_')}.json", dict(failures=report.failures)
    )


def _opening(placement):
    return [
        dict(name=o.name, shape=o.shape, size=o.size, depth=o.depth, rim_in_asset=round(float(o.frame()[2, 3]), 5))
        for o in placement.openings
    ]


def test_rewriting_placement_keeps_every_other_line_and_is_idempotent(tmp_path):
    """The writer replaces only its section, so a following section and the comment that
    heads it survive, and a second write of the same measurement changes nothing."""
    directory = _copy(tmp_path, "recycle_bin")
    path = directory / "meta.yaml"
    tail = "\n# Notes for a later section\nnotes_section:\n  kept: true\n"
    path.write_text(path.read_text() + tail)
    obj = authoring.load(directory)
    placement = authoring.measure_placement(obj, {"top": "rectangle"})
    authoring.write_placement(obj, placement)
    first = path.read_text()
    authoring.write_placement(authoring.load(directory), placement)
    assert path.read_text() == first
    assert first.endswith(tail) and parse(yaml.safe_load(first)) == placement


def test_catalog_placement_matches_legacy_containment():
    """Every open object declares an opening (``check`` holds each section to its model
    in test_collision_fit), and the bin's opening is its legacy containment."""
    rows = []
    for path in sorted(OBJECTS_DIR.rglob("meta.yaml")):
        meta = yaml.safe_load(path.read_text())
        declared = parse(meta)
        legacy = meta.get("geometric_properties") or {}
        if legacy.get("open"):
            assert declared is not None and declared.openings, f"{meta['name']}: open, with no opening"
        if declared is None:
            continue
        rows.append(
            dict(
                name=meta["name"],
                openings=_opening(declared),
                supports=[dict(name=s.name, size=s.size, height=s.frame()[2, 3]) for s in declared.supports],
                legacy={k: legacy[k] for k in ("type", "outer_size", "inner_size", "wall_thickness") if k in legacy},
            )
        )
    # Geodude's drop zones and interior derive the bin's containment from inner_size and
    # outer_size; the measured opening is the same box.
    (bin_,) = [r for r in rows if r["name"] == "recycle_bin"]
    (opening,) = bin_["openings"]
    inner, outer = bin_["legacy"]["inner_size"], bin_["legacy"]["outer_size"]
    # The bin's asset origin is on its base, so the rim's height in its frame is above the support.
    np.testing.assert_allclose([*opening["size"], opening["depth"], opening["rim_in_asset"]], [*inner, outer[2]])
    write_json(ARTIFACTS / "placement.json", rows)
