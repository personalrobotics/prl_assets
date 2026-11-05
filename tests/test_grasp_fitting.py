# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""Grasp regions are measured from the visual model, and the measurement is right.

``prl_assets.authoring`` fits each region a recipe (``prl_assets.grasp_recipes``) names to
the visual points in its region of interest: a cylinder about the asset's vertical
through the smallest circle enclosing them, a box from the minimum-area rectangle of
their horizontal hull, canonicalized so that unchanged inputs cannot flip it by a quarter
turn. The oracle is analytic: point sets of known boxes and cylinders, turned and off the
origin, whose regions the fitters must recover within 0.5 mm and 0.5 degrees, including
a near-square box, where only the tie rule decides its axes, and a handle selected from
a body by its region of interest. A sparse or flat region of interest is refused.

On the catalog, every recipe's region must cover every selected point (within the
rounding of written values) and touch it on every face, and ``check`` holds each declared
section to its re-measurement.

Writes ``artifacts/grasp_regions_analytic.json`` (override the directory with
PRL_ASSETS_ARTIFACTS): each analytic case's built and fitted region and each refusal;
and per catalog recipe region, the SHA-256 of the compiled visual vertices, its region of
interest and exclusions, the selected point count, the fitted parameters, the coverage
residual, the face witnesses, and the declared-against-measured differences. Reproduce
with ``uv run --locked --all-extras pytest tests/test_grasp_fitting.py -q``.
"""

import math
import os
from pathlib import Path

import numpy as np
import pytest
import trimesh

from prl_assets.ycbv_audit import write_json

authoring = pytest.importorskip("prl_assets.authoring")
from prl_assets.grasp_recipes import GRASP_RECIPES  # noqa: E402

ARTIFACTS = Path(os.environ.get("PRL_ASSETS_ARTIFACTS", "artifacts"))
ATOL_M = 5e-4
ATOL_DEG = 0.5


def _yawed(yaw_deg, centre):
    T = np.eye(4)
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = centre
    return T


def _points(mesh):
    """Surface samples and every vertex, so that the extremes are exact."""
    return np.vstack([trimesh.sample.sample_surface(mesh, 4000, seed=0)[0], mesh.vertices])


def _box(size, yaw_deg, centre):
    return trimesh.creation.box(size, transform=_yawed(yaw_deg, centre))


def _cylinder(radius, height, centre):
    return trimesh.creation.cylinder(radius, height, sections=128, transform=_yawed(0, centre))


def _turn(region, expected):
    """Degrees between ``region``'s frame and the ``expected`` rotation."""
    R = np.asarray(region.T_asset_region)[:3, :3].T @ expected
    return math.degrees(math.acos(min(1.0, max(-1.0, (np.trace(R) - 1) / 2))))


def _row(region):
    return dict(
        primitive=region.primitive,
        params={k: list(v) if isinstance(v, tuple) else v for k, v in region.params.items()},
        T_asset_region=[list(r) for r in region.T_asset_region],
    )


def _recovered(region, primitive, params, rotation, centre):
    """``region`` is the built one: its primitive, parameters, frame and centre."""
    rotation = _yawed(rotation, (0, 0, 0))[:3, :3] if np.isscalar(rotation) else rotation
    assert region.primitive == primitive
    for key, value in params.items():
        np.testing.assert_allclose(region.params[key], value, atol=ATOL_M)
    np.testing.assert_allclose(np.asarray(region.T_asset_region)[:3, 3], centre, atol=ATOL_M)
    assert _turn(region, rotation) <= ATOL_DEG, (_turn(region, rotation), region.T_asset_region)


def test_fitted_regions_recover_known_primitives_and_the_catalog_is_measured():
    rows = {"analytic": {}, "refused": {}, "catalog": {}}

    # A box turned 30 degrees, off the origin: x is its shorter side.
    points = _points(_box((0.10, 0.04, 0.06), 30 + 90, (0.02, -0.01, 0.03)))
    fitted = authoring.fit_box_region(points, "body", ("face_x",))
    _recovered(fitted, "box", dict(size=(0.04, 0.10, 0.06)), 30, (0.02, -0.01, 0.03))
    rows["analytic"]["box_turned"] = dict(built=dict(size=[0.04, 0.10, 0.06], yaw=30), fitted=_row(fitted))

    # Near-square: its sides differ by 0.5 mm, under the 2 mm tolerance, so x is the side
    # nearer asset x and the yaw lies in (-45, 45]; turned 44 degrees, that is 44.
    points = _points(_box((0.0605, 0.0600, 0.05), 44 - 90, (0.0, 0.03, 0.025)))
    fitted = authoring.fit_box_region(points, "body", ("top",))
    _recovered(fitted, "box", dict(size=(0.0600, 0.0605, 0.05)), 44, (0.0, 0.03, 0.025))
    rows["analytic"]["box_near_square"] = dict(built=dict(size=[0.06, 0.0605, 0.05], yaw=44), fitted=_row(fitted))

    # A cylinder off the origin.
    points = _points(_cylinder(0.03, 0.08, (0.015, 0.02, 0.05)))
    fitted = authoring.fit_cylinder_region(points, "body", ("side",))
    _recovered(fitted, "cylinder", dict(radius=0.03, height=0.08), 0, (0.015, 0.02, 0.05))
    rows["analytic"]["cylinder_off_centre"] = dict(built=dict(radius=0.03, height=0.08), fitted=_row(fitted))

    # A cylinder lying along a horizontal axis turned 20 degrees: the region's z is its
    # axis, its x horizontal, its y up.
    along = math.radians(20)
    u, v = np.array([math.cos(along), math.sin(along), 0]), np.array([-math.sin(along), math.cos(along), 0])
    lying = np.eye(4)
    lying[:3, :3] = np.column_stack([v, [0, 0, 1], u])
    lying[:3, 3] = (0.01, -0.02, 0.012)
    mesh = trimesh.creation.cylinder(0.012, 0.12, sections=128, transform=lying)
    fitted = authoring.fit_cylinder_region(_points(mesh), "body", ("side",), axis="lying")
    _recovered(fitted, "cylinder", dict(radius=0.012, height=0.12), lying[:3, :3], (0.01, -0.02, 0.012))
    rows["analytic"]["cylinder_lying"] = dict(built=dict(radius=0.012, height=0.12, yaw=20), fitted=_row(fitted))

    # A handle beside a body, selected by its region of interest.
    body = _cylinder(0.04, 0.10, (0.0, 0.0, 0.05))
    handle = _box((0.012, 0.03, 0.05), 0, (0.055, 0.0, 0.05))
    points = _points(trimesh.util.concatenate([body, handle]))
    roi = ((0.042, -1.0, -1.0), (1.0, 1.0, 1.0))
    selected = authoring.select_points(points, roi, (), "handle")
    fitted = authoring.fit_box_region(selected, "handle", ("face_x",))
    _recovered(fitted, "box", dict(size=(0.012, 0.03, 0.05)), 0, (0.055, 0.0, 0.05))
    rows["analytic"]["handle_by_roi"] = dict(built=dict(size=[0.012, 0.03, 0.05], yaw=0), fitted=_row(fitted))

    refusals = {
        "sparse": lambda: authoring.select_points(points, ((0.06, 0.0, 0.07), (1, 1, 1)), (), "corner"),
        "flat": lambda: authoring.fit_box_region(
            np.c_[np.random.default_rng(0).random((500, 2)), np.zeros(500)], "f", ("top",)
        ),
    }
    for case, fit in refusals.items():
        with pytest.raises(ValueError) as caught:
            fit()
        rows["refused"][case] = str(caught.value)

    for name in sorted(GRASP_RECIPES):
        obj = authoring.load(name)
        evidence = authoring.grasp_region_evidence(obj)
        rows["catalog"][name] = evidence
        for region in evidence["regions"]:
            assert region["coverage_residual_m"] <= authoring.COVERAGE_TOLERANCE_M, (name, region)
            assert max(region["face_gaps_m"].values()) <= authoring.COVERAGE_TOLERANCE_M, (name, region)
        assert not authoring.check(obj).failures, (name, authoring.check(obj).failures)
    write_json(ARTIFACTS / "grasp_regions_analytic.json", rows)
