# SPDX-License-Identifier: MIT
"""Versioned placement and containment metadata. Only NumPy is required.

A container's openings and the surfaces other objects may be placed on, each in an
explicit frame: what a placement planner cannot work out from the collision geometry
alone. How an object rests is the planner's to work out, from that geometry. Nothing is
inferred from a primitive type or read from legacy geometry. Returned records are deeply immutable;
frame methods return independent arrays.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

SHAPES = MappingProxyType({"rectangle": "size", "circle": "radius"})
IDENTITY = ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
_NAME = r"[a-z][a-z0-9_]*"


class PlacementError(ValueError):
    """An invalid placement description, with object/entry/field context."""


def _number(value, field, *, positive=True):
    if type(value) not in (int, float):
        raise PlacementError(f"{field}: expected a finite number, not {value!r}")
    try:
        result = float(value)
    except OverflowError as error:
        raise PlacementError(f"{field}: number is not finite") from error
    if not math.isfinite(result) or (positive and result <= 0):
        raise PlacementError(f"{field}: expected a finite {'positive ' if positive else ''}number")
    return result


def _sequence(value, field, length=None):
    if not isinstance(value, (list, tuple)) or (length is not None and len(value) != length):
        raise PlacementError(f"{field}: expected a sequence" + (f" of length {length}" if length is not None else ""))
    return value


def _name(value, field):
    if not isinstance(value, str) or re.fullmatch(_NAME, value) is None:
        raise PlacementError(f"{field}: expected {_NAME}")
    return value


def _rigid(value, field):
    T = tuple(
        tuple(_number(v, field, positive=False) for v in _sequence(row, field, 4)) for row in _sequence(value, field, 4)
    )
    matrix = np.asarray(T)
    R = matrix[:3, :3]
    if not (
        np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=1e-8)
        and np.allclose(R.T @ R, np.eye(3), rtol=0, atol=1e-6)
        and math.isclose(float(np.linalg.det(R)), 1.0, rel_tol=0, abs_tol=1e-6)
    ):
        raise PlacementError(f"{field}: expected a rigid transform with determinant +1")
    return T


def _outline(context, shape, value):
    """A rectangle's full ``size: [x, y]`` or a circle's ``radius``, in metres."""
    if not isinstance(shape, str) or shape not in SHAPES:
        raise PlacementError(f"{context}.shape: expected one of {tuple(SHAPES)}, not {shape!r}")
    field = f"{context}.{SHAPES[shape]}"
    if shape == "rectangle":
        return tuple(_number(v, field) for v in _sequence(value, field, 2))
    return _number(value, field)


@dataclass(frozen=True)
class Opening:
    """A container's usable opening: a rectangle or circle centred on the rim plane, z
    pointing out of the container, with the interior floor ``depth`` below it along -z.
    A rectangle's ``size`` is its full x and y extent; a circle's ``radius`` is the
    largest circle that fits inside the rim."""

    name: str
    shape: str
    size: tuple[float, float] | float
    depth: float
    T_asset_opening: tuple[tuple[float, ...], ...]

    def __post_init__(self):
        context = f"opening {self.name!r}"
        _name(self.name, f"{context}.name")
        object.__setattr__(self, "size", _outline(context, self.shape, self.size))
        object.__setattr__(self, "depth", _number(self.depth, f"{context}.depth"))
        object.__setattr__(self, "T_asset_opening", _rigid(self.T_asset_opening, f"{context}.T_asset_opening"))

    def frame(self) -> np.ndarray:
        """``T_asset_opening`` as a 4 by 4 array."""
        return np.array(self.T_asset_opening)


@dataclass(frozen=True)
class Support:
    """A surface other objects may be placed on: a rectangle centred on the surface, z
    its outward normal. (A support is measured as its top's x-y bounds, so version 1 has
    rectangles only.)"""

    name: str
    shape: str
    size: tuple[float, float] | float
    T_asset_support: tuple[tuple[float, ...], ...]

    def __post_init__(self):
        context = f"support {self.name!r}"
        _name(self.name, f"{context}.name")
        if self.shape != "rectangle":
            raise PlacementError(f"{context}.shape: expected rectangle, not {self.shape!r}")
        object.__setattr__(self, "size", _outline(context, self.shape, self.size))
        object.__setattr__(self, "T_asset_support", _rigid(self.T_asset_support, f"{context}.T_asset_support"))

    def frame(self) -> np.ndarray:
        """``T_asset_support`` as a 4 by 4 array."""
        return np.array(self.T_asset_support)


@dataclass(frozen=True)
class Placement:
    """An object's placement section: at least one opening or support."""

    openings: tuple[Opening, ...] = ()
    supports: tuple[Support, ...] = ()

    def __post_init__(self):
        if not self.openings and not self.supports:
            raise PlacementError("placement: expected at least one opening or support")


_ENTRIES = {
    "openings": (Opening, {"name", "shape", "depth"}, {"T_asset_opening"}),
    "supports": (Support, {"name", "shape"}, {"T_asset_support"}),
}


def _entry(kind, index, row):
    cls, required, optional = _ENTRIES[kind]
    label = f"{kind[:-1]} {row.get('name', index)!r}" if isinstance(row, dict) else f"{kind[:-1]} {index}"
    if not isinstance(row, dict):
        raise PlacementError(f"{label}: expected a mapping")
    shape = row.get("shape")
    if not isinstance(shape, str) or shape not in SHAPES:
        raise PlacementError(f"{label}.shape: expected one of {tuple(SHAPES)}, not {shape!r}")
    required = required | {SHAPES[shape]}
    allowed = required | optional
    if not required <= set(row) or not set(row) <= allowed:
        raise PlacementError(
            f"{label}.fields: missing {sorted(required - set(row))!r}; "
            f"unknown {sorted(repr(k) for k in set(row) - allowed)!r}"
        )
    frame = row.get(next(iter(optional)), IDENTITY)
    size = row[SHAPES[row["shape"]]]
    if kind == "openings":
        return Opening(row["name"], row["shape"], size, row["depth"], frame)
    return Support(row["name"], row["shape"], size, frame)


def parse(meta: dict) -> Placement | None:
    """Validate a loaded metadata dict. A missing ``placement`` section means none
    is declared (``None``); an explicit null or malformed section is an error.

    The section requires ``version`` (integer 1) and ``openings`` or ``supports`` lists,
    together naming at least one entry. Unknown fields are errors, and no legacy geometry
    is read in place of a missing one.
    """
    if not isinstance(meta, dict):
        raise PlacementError("object <unknown>.meta: expected a dictionary")
    name = meta.get("name", "<unnamed>")
    try:
        if "placement" not in meta:
            return None
        section = meta["placement"]
        if not isinstance(section, dict) or "version" not in section or not set(section) <= {"version", *_ENTRIES}:
            raise PlacementError("placement: expected version, and openings or supports")
        if type(section["version"]) is not int or section["version"] != 1:
            raise PlacementError("placement.version: expected integer 1")
        parsed = {}
        for kind in _ENTRIES:
            rows = _sequence(section.get(kind, []), f"placement.{kind}")
            entries = tuple(_entry(kind, i, row) for i, row in enumerate(rows))
            names = [e.name for e in entries]
            if len(set(names)) != len(names):
                raise PlacementError(f"placement.{kind}: duplicate name")
            parsed[kind] = entries
        return Placement(**parsed)
    except PlacementError as error:
        raise PlacementError(f"object {name!r}: {error}") from error
