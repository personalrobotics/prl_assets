"""Behavioral contract for asset grasp descriptions, independent of physics.

Parser controls isolate malformed ingress that cannot economically be exercised by
robot execution: coercion, unknown schema, ambiguous identifiers and nonrigid frames.
Each control writes an inspectable rejection artifact. The catalog integration emits
artifacts/grasp_regions.json and independence measurements. Reproduce with
uv run --locked --all-extras pytest tests/test_grasp_regions.py -q.
"""

import copy
import hashlib
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from prl_assets import OBJECTS_DIR, YCBV_OBJECTS_DIR, authoring
from prl_assets.grasp_regions import GraspRegionError, parse
from prl_assets.ycbv_audit import write_json


def _meta():
    return dict(
        name="probe",
        grasp_regions=dict(version=1, regions=[dict(name="body", primitive="cylinder", radius=0.04, height=0.08)]),
    )


def _bad(case):
    meta = _meta()
    section = meta["grasp_regions"]
    r = section["regions"][0]
    if case in ["bool", "numeric_string", "nan", "inf", "zero", "negative"]:
        r["radius"] = {
            "bool": True,
            "numeric_string": "0.04",
            "nan": float("nan"),
            "inf": float("inf"),
            "zero": 0,
            "negative": -0.1,
        }[case]
    elif case == "duplicate":
        section["regions"].append(copy.deepcopy(r))
    elif case == "primitive":
        r["primitive"] = "capsule"
    elif case == "unknown_key":
        r["mass"] = 1
    elif case == "strategy":
        r["strategies"] = ["sideways"]
    elif case == "empty_strategies":
        r["strategies"] = []
    elif case == "missing_version":
        del section["version"]
    elif case == "wrong_version":
        section["version"] = 2
    elif case == "bool_version":
        section["version"] = True
    elif case == "section_key":
        section["fallback"] = True
    elif case == "null_section":
        meta["grasp_regions"] = None
    elif case == "missing_parameter":
        del r["height"]
    elif case == "name":
        r["name"] = "Body/handle"
    elif case == "duplicate_strategy":
        r["strategies"] = ["side", "side"]
    elif case == "torus":
        r.clear()
        r.update(name="body", primitive="torus", major_radius=0.01, minor_radius=0.02)
    else:
        T = np.eye(4).tolist()
        if case == "reflection":
            T[0][0] = -1
        elif case == "scale":
            T[0][0] = 2
        elif case == "bool_pose":
            T[0][0] = True
        elif case == "string_pose":
            T[0][0] = "1"
        elif case == "nan_pose":
            T[0][3] = float("nan")
        elif case == "bottom_row":
            T[3][2] = 0.1
        elif case == "shape":
            T = T[:3]
        r["T_asset_region"] = T
    return meta


CASES = [
    "bool",
    "numeric_string",
    "nan",
    "inf",
    "zero",
    "negative",
    "duplicate",
    "primitive",
    "unknown_key",
    "strategy",
    "empty_strategies",
    "missing_version",
    "wrong_version",
    "bool_version",
    "section_key",
    "null_section",
    "missing_parameter",
    "name",
    "duplicate_strategy",
    "torus",
    "reflection",
    "scale",
    "bool_pose",
    "string_pose",
    "nan_pose",
    "bottom_row",
    "shape",
]


@pytest.mark.parametrize("case", CASES)
def test_malformed_asset_description_is_rejected(case):
    with pytest.raises(GraspRegionError) as caught:
        parse(_bad(case))
    assert "probe" in str(caught.value)
    out = Path(os.environ.get("PRL_ASSETS_ARTIFACTS", "artifacts")) / "grasp_region_rejections"
    write_json(out / f"{case}.json", dict(case=case, rejected=True, diagnostic=str(caught.value)))


def _record(r):
    return dict(
        name=r.name,
        primitive=r.primitive,
        params=dict(r.params),
        T_asset_region=r.T_asset_region,
        strategies=r.strategies,
        bottom_face_frame=r.bottom_face_frame().tolist(),
        bounds_in_asset=r.bounds_in_asset().tolist(),
    )


def test_primitive_bounds_and_bottom_frames():
    # A right-angle rotation permutes analytic support extents. These oracles do
    # not reproduce the implementation's general rotation/bounds calculation.
    T = [[0, 0, 1, 1], [0, 1, 0, 2], [-1, 0, 0, 3], [0, 0, 0, 1]]
    cases = [
        ("cylinder", dict(radius=2, height=6), [3, 2, 2]),
        ("box", dict(size=[2, 4, 6]), [3, 2, 1]),
        ("sphere", dict(radius=2), [2, 2, 2]),
        ("torus", dict(major_radius=2, minor_radius=1), [1, 3, 3]),
    ]
    records = []
    for primitive, params, half in cases:
        region = dict(name="shape", primitive=primitive, T_asset_region=T, **params)
        r = parse(dict(name=primitive, grasp_regions=dict(version=1, regions=[region])))[0]
        np.testing.assert_allclose(
            r.bounds_in_asset(), [np.array([1, 2, 3]) - half, np.array([1, 2, 3]) + half], atol=1e-12
        )
        origin = r.bottom_face_frame()[:3, 3]
        np.testing.assert_allclose(origin, [-2, 2, 3] if primitive in ["box", "cylinder"] else [1, 2, 3])
        records.append(_record(r))
    write_json(Path(os.environ.get("PRL_ASSETS_ARTIFACTS", "artifacts")) / "grasp_region_math.json", records)


def test_catalog_regions_and_physics_remain_independent(tmp_path):
    rows = []
    for p in sorted(OBJECTS_DIR.rglob("meta.yaml")):
        meta = yaml.safe_load(p.read_text())
        regions = parse(meta)
        rows.append(
            dict(
                name=meta["name"],
                migration_status="grasp_regions" if "grasp_regions" in meta else "legacy only",
                regions=[_record(r) for r in regions],
                aliases=meta.get("perception", {}).get("aliases", []),
                bop_id=meta.get("dataset", {}).get("ycbv", {}).get("bop_id"),
                T_asset_bop=meta.get("frames", {}).get("T_asset_bop"),
            )
        )
    source = YCBV_OBJECTS_DIR / "mug"
    directory = tmp_path / "mug"
    shutil.copytree(source, directory)
    meta = yaml.safe_load((directory / "meta.yaml").read_text())
    regions = parse(meta)
    assert [r.name for r in regions] == ["body", "handle"]
    handle = regions[1]
    T = np.asarray(handle.T_asset_region)
    assert np.linalg.norm(T[:2, 3]) > 0.01 and abs(T[0, 1]) > 0.01
    shift = np.eye(4)
    shift[2, 3] = -handle.params["size"][2] / 2
    np.testing.assert_allclose(handle.bottom_face_frame(), T @ shift, atol=1e-12)
    frozen = copy.deepcopy(meta)
    snapshot = parse(frozen)
    frozen["grasp_regions"]["regions"][0]["radius"] = 0.001
    frozen["grasp_regions"]["regions"][0]["T_asset_region"][0][3] = 1
    assert snapshot == regions
    with pytest.raises(TypeError):
        regions[0].params["radius"] = 1
    returned = regions[0].bottom_face_frame()
    returned[0, 3] = 99
    assert snapshot == regions
    physical_files = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(directory.iterdir()) if p.name != "meta.yaml"
    }
    obj = authoring.load(directory)
    original = obj.model
    smaller = copy.deepcopy(meta)
    smaller["grasp_regions"]["regions"][0]["radius"] = 0.02
    assert authoring.check(authoring.from_spec(obj.spec.copy(), obj, smaller)).ok
    (directory / "meta.yaml").write_text(yaml.safe_dump(smaller))
    modified = authoring.load(directory).model
    for field in [
        "geom_type",
        "geom_size",
        "geom_pos",
        "geom_quat",
        "geom_friction",
        "body_mass",
        "body_inertia",
        "mesh_vert",
        "mesh_face",
    ]:
        np.testing.assert_array_equal(getattr(original, field), getattr(modified, field))
    assert physical_files == {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(directory.iterdir()) if p.name != "meta.yaml"
    }
    tree = ET.parse(directory / "mug.xml")
    next(g for g in tree.iter("geom") if g.get("group") == "3").set("pos", ".001 0 0")
    tree.write(directory / "mug.xml")
    assert parse(yaml.safe_load((directory / "meta.yaml").read_text())) == parse(smaller)
    # Compile the changed geometry as a real resource, rather than merely editing metadata.
    changed = authoring.load(directory).model
    assert not np.array_equal(changed.geom_pos, modified.geom_pos)
    millimetres = copy.deepcopy(meta)
    millimetres["grasp_regions"]["regions"][0]["radius"] = 40
    report = authoring.check(authoring.from_spec(obj.spec.copy(), obj, millimetres))
    assert any("grasp_regions.body" in f for f in report.failures)
    write_json(
        Path(os.environ.get("PRL_ASSETS_ARTIFACTS", "artifacts")) / "grasp_regions.json",
        dict(
            objects=rows,
            independence=dict(
                region_only_preserves_physics=True,
                collision_only_preserves_regions=True,
                smaller_region_passes=True,
                millimetre_region_rejected=True,
                physical_resource_hashes=physical_files,
            ),
        ),
    )
