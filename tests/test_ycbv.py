# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""The YCB-Video collection: discovery, identity, and the frame contract in docs/schema.md.

Writes artifacts/ycbv_collection.json (override the directory with PRL_ASSETS_ARTIFACTS):
each collection object's BOP id, YCB name, model path, and T_asset_bop as the registry
returns them.
"""

import json
import os
from pathlib import Path

import numpy as np
import pytest
import yaml

from prl_assets import OBJECTS_DIR, YCBV_OBJECTS_DIR
from prl_assets.ycbv import BOP_YCB_NAMES

YCBV_METAS = sorted(YCBV_OBJECTS_DIR.rglob("meta.yaml"))


def _rigid(matrix, name):
    T = np.asarray(matrix, dtype=float)
    assert T.shape == (4, 4), f"{name}: not 4x4"
    assert np.allclose(T[3], [0, 0, 0, 1]), f"{name}: bottom row {T[3]}"
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-6), f"{name}: rotation is not orthonormal (scale belongs outside)"
    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-6), f"{name}: rotation is a reflection"


def test_the_collection_has_no_metadata_of_its_own():
    """A meta.yaml in the collection directory would register the directory as an object."""
    assert not (YCBV_OBJECTS_DIR / "meta.yaml").exists()
    assert YCBV_METAS, "the collection holds no objects"


@pytest.mark.parametrize("meta_path", YCBV_METAS, ids=lambda p: p.parent.name)
def test_ycbv_identity_frame_and_symmetries(meta_path: Path):
    meta = yaml.safe_load(meta_path.read_text())
    name = meta["name"]
    ycbv = meta["dataset"]["ycbv"]
    assert BOP_YCB_NAMES.get(ycbv["bop_id"]) == ycbv["ycb_name"], f"{name}: {ycbv} is not a BOP ycbv object"
    assert ycbv["ycb_name"] in meta["perception"]["aliases"], f"{name}: its YCB name must resolve to it"
    _rigid(meta["frames"]["T_asset_bop"], name)

    sym = meta["symmetries"]
    assert sym["frame"] == "bop"
    for flat in sym["discrete"]:
        S = np.asarray(flat, dtype=float).reshape(4, 4)
        S[:3, 3] /= 1000.0
        _rigid(S, f"{name} discrete symmetry")
    for c in sym["continuous"]:
        assert np.linalg.norm(c["axis"]) == pytest.approx(1.0, abs=1e-6) and len(c["offset"]) == 3

    for field in ("geometry", "reference_model", "license", "attribution"):
        assert meta["provenance"].get(field), f"{name}: provenance.{field} is missing"


def test_bop_ids_name_one_object_each():
    ids = [yaml.safe_load(p.read_text())["dataset"]["ycbv"]["bop_id"] for p in YCBV_METAS]
    assert len(ids) == len(set(ids)), sorted(ids)
    assert set(ids) == set(BOP_YCB_NAMES), f"incomplete benchmark catalog: {sorted(ids)}"


def test_the_registry_finds_the_collection_alone_and_within_the_catalog():
    """Both roots load without duplicate or ambiguous registrations; names, YCB names and
    old aliases resolve to the moved objects; neighbouring non-YCB objects keep theirs."""
    AssetManager = pytest.importorskip("asset_manager").AssetManager
    collection = AssetManager(str(YCBV_OBJECTS_DIR))
    catalog = AssetManager(str(OBJECTS_DIR))

    assert set(collection.list()) <= set(catalog.list())
    assert len(collection.list()) == len(YCBV_METAS)
    rows = []
    for name in collection.list():
        path = Path(catalog.get_path(name, "mujoco"))
        assert path == Path(collection.get_path(name, "mujoco"))
        assert path.resolve().is_relative_to(YCBV_OBJECTS_DIR.resolve()) and path.is_file()
        meta = catalog.get(name)
        assert catalog.resolve_alias(meta["dataset"]["ycbv"]["ycb_name"]) == name
        rows.append(
            {
                "name": name,
                **meta["dataset"]["ycbv"],
                "mujoco": path.relative_to(OBJECTS_DIR).as_posix(),
                "T_asset_bop": meta["frames"]["T_asset_bop"],
            }
        )

    # The four objects that moved into the collection keep their names and labels.
    for name in ("cracker_box", "sugar_box", "gelatin_box", "spam_can"):
        assert name in collection.list()
    assert catalog.resolve_alias("potted meat can") == "spam_can"
    assert catalog.resolve_alias("spam") == "spam_can"
    # Category neighbours are not YCB-V objects.
    assert catalog.resolve_alias("bowl") == "plastic_bowl"
    assert "plastic_bowl" not in collection.list()

    out = Path(os.environ.get("PRL_ASSETS_ARTIFACTS", "artifacts"))
    out.mkdir(parents=True, exist_ok=True)
    (out / "ycbv_collection.json").write_text(json.dumps(sorted(rows, key=lambda r: r["bop_id"]), indent=2) + "\n")
