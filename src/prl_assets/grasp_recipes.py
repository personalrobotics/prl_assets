# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""How each object's grasp regions are measured: the canonical recipes.

A recipe names the interactions an object offers, a primitive for each and the grasp
strategies it supports, and the part of the visual model each is measured from (its
region of interest, in the asset frame). Every number in a ``grasp_regions`` section is
then measured from the visual model by ``prl_assets.authoring.measure_grasp_regions``,
and ``authoring.check`` holds the declared section to that measurement. Recipes are
authoring data: the runtime schema (``prl_assets.grasp_regions``) never sees them.

A region of interest is chosen from the object's geometry, never from how a robot fared
with it; its comment says why. Objects whose regions were declared before recipes (the
recycling kinds and the mug, prl_assets#40 part 1) have none.
"""

from __future__ import annotations

from dataclasses import dataclass

Bounds = tuple[tuple[float, float, float], tuple[float, float, float]]
"""An axis-aligned box in the asset frame: its least and greatest corner (m)."""

_ANY = float("inf")


@dataclass(frozen=True)
class Recipe:
    """One region: its ``name``, ``primitive`` (``cylinder`` or ``box``) and
    ``strategies``, measured from the visual points inside ``roi`` (all of them when
    ``None``), less those inside each of ``exclude``'s boxes, each with its reason. A
    cylinder stands ``upright`` or is ``lying`` (``authoring.fit_cylinder_region``)."""

    name: str
    primitive: str
    strategies: tuple[str, ...]
    roi: Bounds | None = None
    exclude: tuple[tuple[Bounds, str], ...] = ()
    axis: str = "upright"


def _below(z: float) -> Bounds:
    return ((-_ANY, -_ANY, -_ANY), (_ANY, _ANY, z))


_CAN = (Recipe("body", "cylinder", ("side", "top")),)
_BLOCK = (Recipe("body", "box", ("top", "bottom", "face_x", "face_y")),)

GRASP_RECIPES: dict[str, tuple[Recipe, ...]] = {
    # Cans: the whole can is one cylinder, gripped round its side or across its lid.
    "master_chef_can": _CAN,
    "tomato_soup_can": _CAN,
    "tuna_fish_can": _CAN,
    # A 19 mm marker lying flat, its axis horizontal: only round its side; its ends are
    # too small to grip.
    "large_marker": (Recipe("body", "cylinder", ("side",), axis="lying"),),
    # Blocks and boxes: one box, on any face.
    "foam_brick": _BLOCK,
    "pudding_box": _BLOCK,
    "wood_block": _BLOCK,
    # Bottles: the body below the shoulder, where its section stops narrowing to the
    # neck: 99 by 68 mm up to z = 0.038 m, above which the bleach bottle's handle and
    # neck begin; 95 by 58 mm up to z = 0.048 m, above which the mustard bottle's
    # shoulder narrows to its 47 mm cap. The cap and spout are another interaction, and
    # the body's top face lies inside the bottle, so it is gripped on its faces only.
    "bleach_cleanser": (Recipe("body", "box", ("face_x", "face_y"), roi=_below(0.038)),),
    "mustard_bottle": (Recipe("body", "box", ("face_x", "face_y"), roi=_below(0.048)),),
}
