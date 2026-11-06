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
    ``None``), less those inside each of ``exclude``'s boxes, each with its reason; the
    boxes are in the asset frame turned ``roi_yaw_deg`` about its z, so that a part lying
    on a diagonal is selected apart from what it joins. A cylinder stands ``upright`` or
    is ``lying`` (``authoring.fit_cylinder_region``)."""

    name: str
    primitive: str
    strategies: tuple[str, ...]
    roi: Bounds | None = None
    exclude: tuple[tuple[Bounds, str], ...] = ()
    axis: str = "upright"
    roi_yaw_deg: float = 0.0


_ALL = (-_ANY, _ANY)


def _box(x=_ALL, y=_ALL, z=_ALL) -> Bounds:
    """The box spanning ``x``, ``y`` and ``z``, each a (least, greatest) range."""
    return ((x[0], y[0], z[0]), (x[1], y[1], z[1]))


def _below(z: float) -> Bounds:
    return _box(z=(-_ANY, z))


def _above(z: float) -> Bounds:
    return _box(z=(z, _ANY))


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
    "bleach_cleanser": (
        Recipe("body", "box", ("face_x", "face_y"), roi=_below(0.038)),
        # Its cap: a 31 mm cylinder from z = 0.105 m, where the neck stops narrowing, to
        # the top; gripped round it or across its top, the bottle hangs upright.
        Recipe("cap", "cylinder", ("side", "top"), roi=_above(0.105)),
    ),
    "mustard_bottle": (Recipe("body", "box", ("face_x", "face_y"), roi=_below(0.048)),),
    # Parts of objects lying flat, each where its section is a solid bar, apart from what
    # it joins, gripped from above or from beside it.
    # The banana's middle, 65 mm along its length where it is nearly straight, between
    # the curve toward its stem and its narrowing tip: cut across it, in a frame turned
    # -13 degrees, along its middle's axis.
    "banana": (Recipe("middle", "box", ("top", "face_x"), roi=_box(y=(-0.041, 0.024)), roi_yaw_deg=-13),),
    # The drill's grip, from just above its battery (y = -0.050 m) to just below its
    # trigger (y = -0.002 m), where its section is a 48 by 34 mm bar. As long as it is
    # wide, its x is the side nearer asset x by the tie rule: across the grip, so that
    # face_x takes it from front and back, never through the battery or trigger.
    "power_drill": (Recipe("handle", "box", ("top", "face_x"), roi=_box(y=(-0.050, -0.002))),),
    # A spring clamp's two handles, the open arms opposite its closed jaws: each a bar
    # about 20 mm wide, selected in a frame turned along it (u along the arm), from
    # where it leaves the spring's coil to its end, and within 20 mm of its axis, apart
    # from the other arm.
    "large_clamp": (
        Recipe("handle_a", "box", ("top", "face_x"), roi=_box((0.030, _ANY), (-0.016, 0.024)), roi_yaw_deg=130),
        Recipe("handle_b", "box", ("top", "face_x"), roi=_box((0.040, _ANY), (-0.044, -0.004)), roi_yaw_deg=70),
    ),
    "extra_large_clamp": (
        Recipe("handle_a", "box", ("top", "face_x"), roi=_box((0.040, _ANY), (-0.018, 0.022)), roi_yaw_deg=45),
        Recipe("handle_b", "box", ("top", "face_x"), roi=_box((0.040, _ANY), (-0.040, 0.000)), roi_yaw_deg=-30),
    ),
    # The scissors' shank, between where the handle loops join it (y = -0.010 m) and
    # where the blades thin (y = 0.015 m): its solid 15 mm section. The loops are rings
    # round their holes, and the blades 5 mm thick; gripped from above only.
    "scissors": (Recipe("shank", "box", ("top",), roi=_box(y=(-0.010, 0.015))),),
    # The pitcher's handle: the upright bar of its loop, selected in a frame turned 45
    # degrees along it, beyond the inner edge of the loop (u = 0.072 m), between its bends
    # into the body (z = 0.000 to 0.080 m); gripped from beside it. Its body, 147 mm
    # across, has no other grip.
    "pitcher_base": (
        Recipe("handle", "box", ("face_x", "face_y"), roi=_box((0.072, _ANY), z=(0.000, 0.080)), roi_yaw_deg=45),
    ),
}
