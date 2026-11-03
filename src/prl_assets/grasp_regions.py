# SPDX-License-Identifier: MIT
"""Versioned, robot-neutral grasp approximations. Only NumPy is required.

All primitive frames are centred. Returned records are deeply immutable; frame and
bounds methods return independent arrays. No legacy geometry is inferred or read.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping

import numpy as np

PARAMETERS = MappingProxyType(
    {
        "cylinder": ("radius", "height"),
        "box": ("size",),
        "sphere": ("radius",),
        "torus": ("major_radius", "minor_radius"),
    }
)
STRATEGIES = MappingProxyType(
    {
        "cylinder": ("side", "top", "bottom"),
        "box": ("top", "bottom", "face_x", "face_y"),
        "sphere": ("all",),
        "torus": ("side", "span"),
    }
)
PRIMITIVES = tuple(PARAMETERS)
IDENTITY = ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))


class GraspRegionError(ValueError):
    """An invalid asset-region description, with object/region/field context."""


def _number(value, field, *, positive=True):
    if type(value) not in (int, float):
        raise GraspRegionError(f"{field}: expected a finite number, not {value!r}")
    try:
        result = float(value)
    except OverflowError as error:
        raise GraspRegionError(f"{field}: number is not finite") from error
    if not math.isfinite(result) or (positive and result <= 0):
        raise GraspRegionError(f"{field}: expected a finite {'positive ' if positive else ''}number")
    return result


def _sequence(value, field, length=None):
    if not isinstance(value, (list, tuple)) or (length is not None and len(value) != length):
        raise GraspRegionError(f"{field}: expected a sequence" + (f" of length {length}" if length is not None else ""))
    return value


@dataclass(frozen=True)
class GraspRegion:
    name: str
    primitive: str
    params: Mapping[str, float | tuple[float, ...]]
    T_asset_region: tuple[tuple[float, ...], ...]
    strategies: tuple[str, ...]

    def __post_init__(self):
        context = f"region {self.name!r}"
        if not isinstance(self.name, str) or re.fullmatch(r"[a-z][a-z0-9_]*", self.name) is None:
            raise GraspRegionError(f"{context}.name: expected [a-z][a-z0-9_]*")
        if not isinstance(self.primitive, str) or self.primitive not in PARAMETERS:
            raise GraspRegionError(f"{context}.primitive: unsupported primitive {self.primitive!r}")
        if not isinstance(self.params, Mapping) or set(self.params) != set(PARAMETERS[self.primitive]):
            raise GraspRegionError(f"{context}.params: expected exactly {PARAMETERS[self.primitive]}")
        params = {}
        for key, value in self.params.items():
            field = f"{context}.{key}"
            params[key] = (
                tuple(_number(v, field) for v in _sequence(value, field, 3)) if key == "size" else _number(value, field)
            )
        if self.primitive == "torus" and params["minor_radius"] >= params["major_radius"]:
            raise GraspRegionError(f"{context}.minor_radius: must be smaller than major_radius")
        field = f"{context}.T_asset_region"
        T = tuple(
            tuple(_number(v, field, positive=False) for v in _sequence(row, field, 4))
            for row in _sequence(self.T_asset_region, field, 4)
        )
        matrix = np.asarray(T)
        R = matrix[:3, :3]
        if not (
            np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=1e-8)
            and np.allclose(R.T @ R, np.eye(3), rtol=0, atol=1e-6)
            and math.isclose(float(np.linalg.det(R)), 1.0, rel_tol=0, abs_tol=1e-6)
        ):
            raise GraspRegionError(f"{field}: expected a rigid transform with determinant +1")
        strategies = tuple(_sequence(self.strategies, f"{context}.strategies"))
        if (
            not strategies
            or any(not isinstance(s, str) or s not in STRATEGIES[self.primitive] for s in strategies)
            or len(set(strategies)) != len(strategies)
        ):
            raise GraspRegionError(f"{context}.strategies: expected distinct members of {STRATEGIES[self.primitive]}")
        object.__setattr__(self, "params", MappingProxyType(params))
        object.__setattr__(self, "T_asset_region", T)
        object.__setattr__(self, "strategies", strategies)
        self.bounds_in_asset()

    def bottom_face_frame(self) -> np.ndarray:
        """T_asset_factory: bottom centre for cylinder/box, centre for sphere/torus.

        This shift is in the region's local z direction. Consumers compose their
        observed T_world_asset with this frame, without another height shift.
        """
        T = np.array(self.T_asset_region)
        height = (
            self.params["height"]
            if self.primitive == "cylinder"
            else self.params["size"][2]
            if self.primitive == "box"
            else 0.0
        )
        T[:3, 3] -= T[:3, 2] * (height / 2)
        return T

    def bounds_in_asset(self) -> np.ndarray:
        """Exact asset-frame AABB, as [minimum_xyz, maximum_xyz] in metres.

        The support extent of a circular cross-section uses the norm of the
        projected axis row; no mesh approximation or sampling is involved.
        """
        T = np.asarray(self.T_asset_region)
        R = T[:3, :3]
        p = self.params
        with np.errstate(over="ignore", invalid="ignore"):
            if self.primitive == "box":
                extent = np.abs(R) @ (np.asarray(p["size"]) / 2)
            elif self.primitive == "cylinder":
                extent = p["radius"] * np.hypot(R[:, 0], R[:, 1]) + p["height"] / 2 * np.abs(R[:, 2])
            elif self.primitive == "sphere":
                extent = np.full(3, p["radius"])
            else:
                extent = p["major_radius"] * np.hypot(R[:, 0], R[:, 1]) + p["minor_radius"]
            bounds = np.stack((T[:3, 3] - extent, T[:3, 3] + extent))
        if not np.isfinite(bounds).all():
            raise GraspRegionError(f"region {self.name!r}.bounds: parameters overflow asset-frame bounds")
        return bounds


def parse(meta: dict) -> tuple[GraspRegion, ...]:
    """Validate a loaded metadata dict. Missing grasp_regions means no regions.

    Legacy geometric_properties is never used as a fallback. Unknown fields and
    explicit nulls are errors, including in a section alongside legacy metadata.
    """
    if not isinstance(meta, dict):
        raise GraspRegionError("object <unknown>.meta: expected a dictionary")
    name = meta.get("name", "<unnamed>")
    try:
        if "grasp_regions" not in meta:
            return ()
        section = meta["grasp_regions"]
        if not isinstance(section, dict) or set(section) != {"version", "regions"}:
            raise GraspRegionError("grasp_regions: expected exactly version and regions")
        if type(section["version"]) is not int or section["version"] != 1:
            raise GraspRegionError("grasp_regions.version: expected integer 1")
        rows = _sequence(section["regions"], "grasp_regions.regions")
        result = []
        names = set()
        for index, row in enumerate(rows):
            label = f"region {row.get('name', index)!r}" if isinstance(row, dict) else f"region {index}"
            if not isinstance(row, dict):
                raise GraspRegionError(f"{label}: expected a mapping")
            primitive = row.get("primitive")
            if not isinstance(primitive, str) or primitive not in PARAMETERS:
                raise GraspRegionError(f"{label}.primitive: unsupported primitive {primitive!r}")
            required = {"name", "primitive", *PARAMETERS[primitive]}
            allowed = required | {"T_asset_region", "strategies"}
            if not required <= set(row) or not set(row) <= allowed:
                raise GraspRegionError(
                    f"{label}.fields: missing {sorted(required - set(row))!r}; "
                    f"unknown {sorted(repr(k) for k in set(row) - allowed)!r}"
                )
            region = GraspRegion(
                row["name"],
                primitive,
                {k: row[k] for k in PARAMETERS[primitive]},
                row.get("T_asset_region", IDENTITY),
                row.get("strategies", STRATEGIES[primitive]),
            )
            if region.name in names:
                raise GraspRegionError(f"{label}.name: duplicate name")
            names.add(region.name)
            result.append(region)
        return tuple(result)
    except GraspRegionError as error:
        raise GraspRegionError(f"object {name!r}: {error}") from error
