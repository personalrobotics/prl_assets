"""Behavioral contract for placement metadata (docs/schema.md, "Placement").

Parser controls isolate malformed ingress, each writing a rejection artifact. The
compiled-model controls show that a declared opening, rest or support that disagrees with
the model fails ``authoring.check``. The catalog integration shows every object's section
is exactly what the measurers write, and that the recycle bin's opening is the legacy
containment Geodude's drop zones read; it writes artifacts/placement.json. Reproduce
with uv run --locked --all-extras pytest tests/test_placement.py -q.
"""

import copy
import os
import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml

from prl_assets import OBJECTS_DIR, authoring
from prl_assets.placement import PlacementError, parse
from prl_assets.ycbv_audit import write_json

ARTIFACTS = Path(os.environ.get("PRL_ASSETS_ARTIFACTS", "artifacts"))
IDENTITY = np.eye(4).tolist()


def _meta():
    rest = dict(name="upright", T_support_asset=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.05], [0, 0, 0, 1]])
    return dict(
        name="probe",
        placement=dict(
            version=1,
            rests=[dict(rest, footprint_radius=0.04)],
            openings=[dict(name="top", shape="circle", radius=0.03, depth=0.08, T_asset_opening=IDENTITY)],
            supports=[dict(name="top", shape="rectangle", size=[0.2, 0.1])],
        ),
    )


def _bad(case):
    meta = _meta()
    section = meta["placement"]
    rest, opening, support = section["rests"][0], section["openings"][0], section["supports"][0]
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
    elif case == "no_rests":
        section["rests"] = []
    elif case == "missing_rests":
        del section["rests"]
    elif case == "duplicate":
        section["rests"].append(copy.deepcopy(rest))
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
        rest["mass"] = 1.0
    elif case == "missing_footprint":
        del rest["footprint_radius"]
    elif case == "rest_offset":
        rest["T_support_asset"] = [[1, 0, 0, 0.01], [0, 1, 0, 0], [0, 0, 1, 0.05], [0, 0, 0, 1]]
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
    "no_rests",
    "missing_rests",
    "duplicate",
    "name",
    "shape",
    "shape_parameter",
    "rectangle_size",
    "missing_depth",
    "unknown_key",
    "missing_footprint",
    "rest_offset",
    "reflection",
    "string_pose",
    "bottom_row",
]


def test_well_formed_placement_reads_back():
    placement = parse(_meta())
    assert placement.canonical.height == 0.05
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
    "can/rest_height": (
        "can",
        lambda p: p["rests"][0].update(T_support_asset=_shift(p["rests"][0]["T_support_asset"], 2, 0.01)),
    ),
    "can/footprint": ("can", lambda p: p["rests"][0].update(footprint_radius=0.02)),
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
        dict(
            name=o.name,
            shape=o.shape,
            size=o.size,
            depth=o.depth,
            rim_above_support=round(float(o.frame()[2, 3] + placement.canonical.height), 5),
        )
        for o in placement.openings
    ]


def test_catalog_placement_is_measured_and_matches_legacy_containment():
    rows = []
    for path in sorted(OBJECTS_DIR.rglob("meta.yaml")):
        meta = yaml.safe_load(path.read_text())
        declared = parse(meta)
        assert declared is not None, f"{meta['name']}: no placement section"
        obj = authoring.load(path.parent)
        openings = {o.name: o.shape for o in declared.openings}
        measured = authoring.measure_placement(obj, openings, tuple(s.name for s in declared.supports))
        assert measured == declared, f"{meta['name']}: declared placement is not what the measurers write"
        legacy = meta.get("geometric_properties") or {}
        rows.append(
            dict(
                name=meta["name"],
                rest_height=declared.canonical.height,
                footprint_radius=declared.canonical.footprint_radius,
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
    np.testing.assert_allclose([*opening["size"], opening["depth"], opening["rim_above_support"]], [*inner, outer[2]])
    write_json(ARTIFACTS / "placement.json", rows)
