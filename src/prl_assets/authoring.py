# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""Measure, fit and build prl_assets objects as MuJoCo compiles them.

Every measurement works on the compiled model, because that is what simulation
collides with: a mesh geom collides as its convex hull, defaults and classes are
applied, and geom offsets are resolved. The visual mesh (geom group 2) is the
reference for the object's shape; the collision geoms (those with contype or
conaffinity) must match it.

Requires the ``authoring`` extra (``mujoco``, ``scipy``)::

    from prl_assets import authoring

    obj = authoring.load("plastic_bowl")
    print(authoring.check(obj))

Typical workflow for a new object: measure its visual mesh with ``visual_points``,
build collision geoms with ``fit_box``, ``stacked_cylinders``, ``revolved_shell`` or
``open_box``, write the model with ``write_model``, then run ``check`` (or
``scripts/validate_objects.py <name>``) until it passes. CI runs the same check on
every object.
"""

from __future__ import annotations

import hashlib
import math
import os
import re
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np
import trimesh
import yaml
from scipy.spatial import ConvexHull, QhullError

from prl_assets import OBJECTS_DIR
from prl_assets.grasp_recipes import GRASP_RECIPES
from prl_assets.grasp_regions import IDENTITY, GraspRegion
from prl_assets.grasp_regions import parse as parse_grasp_regions
from prl_assets.placement import SHAPES, Opening, Placement, Support
from prl_assets.placement import parse as parse_placement

VISUAL_GROUP = 2
COLLISION_GROUP = 3

# Defaults for check(); an object may widen one in
# geometric_properties.collision_tolerance_mm with a stated reason.
DEFAULT_TOLERANCE_MM = {
    "visual_outside": 3.0,  # 99th percentile of the visual surface outside the collision geoms
    "collision_excess": 3.0,  # 99th percentile of the collision surface outside the visual's hull
    "sink": 1.0,  # how far the visual mesh reaches below the collision geometry's bottom
    "resting": 1.0,  # |collision bottom| at the model file's pose, which must rest on z = 0
    "floor_gap": 3.0,  # open objects: a dropped sphere's rest height above the visual inner floor
}
# How far a size in geometric_properties or dimensions may differ from the collision
# geometry's extent: legacy placement readers still treat these as physical sizes.
# New grasp regions have a separate frame/bounds check and are not compared here.
SIZE_TOLERANCE_MM = 2.0

_SAMPLES = 20000
_PROBE_RADIUS = 0.01


# --------------------------------------------------------------------------- loading


def object_dirs(root: Path = OBJECTS_DIR) -> dict[str, Path]:
    """Every object directory under ``root``, at any depth, keyed by directory name.
    Names are unique across collections; a duplicate is an error."""
    dirs: dict[str, Path] = {}
    for meta in sorted(Path(root).rglob("meta.yaml")):
        name = meta.parent.name
        if name in dirs:
            raise ValueError(f"object name {name!r} is used by both {dirs[name]} and {meta.parent}")
        dirs[name] = meta.parent
    return dirs


@dataclass
class Compiled:
    """An object's compiled model, at the pose its model file gives it."""

    name: str
    dir: Path
    meta: dict
    spec: mujoco.MjSpec
    model: mujoco.MjModel
    data: mujoco.MjData

    @property
    def body(self) -> int:
        """The object's top-level body."""
        return next(b for b in range(1, self.model.nbody) if self.model.body_parentid[b] == 0)


def load(obj: str | Path, root: Path = OBJECTS_DIR) -> Compiled:
    """Compile an object by name (searched under ``root``) or by directory."""
    obj_dir = Path(obj) if Path(obj).is_dir() else object_dirs(root)[str(obj)]
    meta = yaml.safe_load((obj_dir / "meta.yaml").read_text()) or {}
    parse_grasp_regions(meta)
    parse_placement(meta)
    spec = mujoco.MjSpec.from_file(str(obj_dir / meta["mujoco"]["xml_path"]))
    return _compiled(meta.get("name", obj_dir.name), obj_dir, meta, spec)


def from_spec(spec: mujoco.MjSpec, like: Compiled, meta: dict | None = None) -> Compiled:
    """``like`` with an edited spec (and optionally edited metadata), to try a change
    before writing it."""
    return _compiled(like.name, like.dir, like.meta if meta is None else meta, spec)


def _compiled(name, obj_dir, meta, spec) -> Compiled:
    parse_grasp_regions(meta)
    parse_placement(meta)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return Compiled(name, obj_dir, meta, spec, model, data)


# ----------------------------------------------------------------------- geometry


def collision_geoms(obj: Compiled) -> list[int]:
    m = obj.model
    return [g for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]]


def visual_geoms(obj: Compiled) -> list[int]:
    m = obj.model
    return [
        g
        for g in range(m.ngeom)
        if m.geom_group[g] == VISUAL_GROUP
        and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
        and not (m.geom_contype[g] or m.geom_conaffinity[g])
    ]


def geom_mesh(obj: Compiled, g: int) -> trimesh.Trimesh:
    """Geom ``g`` as a triangle mesh in world coordinates (a mesh geom as its own triangles)."""
    m, d = obj.model, obj.data
    t, s = int(m.geom_type[g]), m.geom_size[g]
    G = mujoco.mjtGeom
    if t == G.mjGEOM_MESH:
        i = m.geom_dataid[g]
        verts = m.mesh_vert[m.mesh_vertadr[i] : m.mesh_vertadr[i] + m.mesh_vertnum[i]]
        faces = m.mesh_face[m.mesh_faceadr[i] : m.mesh_faceadr[i] + m.mesh_facenum[i]]
        mesh = trimesh.Trimesh(verts, faces, process=False)
    elif t == G.mjGEOM_BOX:
        mesh = trimesh.creation.box(2 * s)
    elif t == G.mjGEOM_CYLINDER:
        mesh = trimesh.creation.cylinder(s[0], 2 * s[1], sections=64)
    elif t == G.mjGEOM_SPHERE:
        mesh = trimesh.creation.icosphere(4, s[0])
    elif t == G.mjGEOM_CAPSULE:
        mesh = trimesh.creation.capsule(2 * s[1], s[0])
    else:
        raise ValueError(f"{obj.name}: geom {g} has unsupported type {mujoco.mjtGeom(t).name}")
    mesh.apply_transform(_pose(d.geom_xpos[g], d.geom_xmat[g]))
    return mesh


def visual_points(obj: Compiled, n: int = _SAMPLES, seed: int = 0) -> np.ndarray:
    """Surface samples of the visual mesh, world coordinates."""
    meshes = [geom_mesh(obj, g) for g in visual_geoms(obj)]
    if not meshes:
        raise ValueError(f"{obj.name}: no visual mesh (group {VISUAL_GROUP})")
    return trimesh.sample.sample_surface(trimesh.util.concatenate(meshes), n, seed=seed)[0]


def to_body(obj: Compiled, points: np.ndarray) -> np.ndarray:
    """World points expressed in the object's body frame."""
    b = obj.body
    return (points - obj.data.xpos[b]) @ obj.data.xmat[b].reshape(3, 3)


def distance_outside(obj: Compiled, points: np.ndarray, geoms: list[int] | None = None) -> np.ndarray:
    """Distance (m) of each world point outside the union of ``geoms`` (default: the
    collision geoms); 0 inside. Mesh geoms count as their convex hull, as MuJoCo collides
    them, and the distance outside a hull is its largest face-plane distance, a lower
    bound that is exact wherever the nearest feature is a face."""
    m, d = obj.model, obj.data
    G = mujoco.mjtGeom
    out = np.full(len(points), np.inf)
    for g in collision_geoms(obj) if geoms is None else geoms:
        t, s = int(m.geom_type[g]), m.geom_size[g]
        if t == G.mjGEOM_MESH:
            dist = _outside_hull(geom_mesh(obj, g).vertices, points)
        else:
            q = (points - d.geom_xpos[g]) @ d.geom_xmat[g].reshape(3, 3)
            if t == G.mjGEOM_BOX:
                dist = np.linalg.norm(np.maximum(np.abs(q) - s, 0), axis=1)
            elif t == G.mjGEOM_CYLINDER:
                radial = np.linalg.norm(q[:, :2], axis=1) - s[0]
                dist = np.linalg.norm(np.maximum(np.stack([radial, np.abs(q[:, 2]) - s[1]], 1), 0), axis=1)
            elif t == G.mjGEOM_SPHERE:
                dist = np.maximum(np.linalg.norm(q, axis=1) - s[0], 0)
            elif t == G.mjGEOM_CAPSULE:
                axis = np.zeros_like(q)
                axis[:, 2] = np.clip(q[:, 2], -s[1], s[1])
                dist = np.maximum(np.linalg.norm(q - axis, axis=1) - s[0], 0)
            else:
                raise ValueError(f"{obj.name}: geom {g} has unsupported type {mujoco.mjtGeom(t).name}")
        out = np.minimum(out, dist)
    return out


def _outside_hull(vertices: np.ndarray, points: np.ndarray) -> np.ndarray:
    eq = ConvexHull(vertices).equations
    return np.maximum((points @ eq[:, :3].T + eq[:, 3]).max(axis=1), 0)


# ---------------------------------------------------------------------- measuring


@dataclass
class Fit:
    """How an object's collision geometry agrees with its visual mesh, in millimetres."""

    resting: float
    collision_extent: list[float]
    # None for an object without a visual mesh: its collision geoms are what it shows.
    visual_outside_p99: float | None = None
    visual_outside_max: float | None = None
    collision_excess_p99: float | None = None
    collision_excess_max: float | None = None
    sink: float | None = None
    floor_gap: float | None = None
    # Open objects: the dropped probe came to rest off the object (rolled off a filled top).
    probe_left_object: bool = False
    visual_extent: list[float] | None = None


def collision_fit(obj: Compiled, n: int = _SAMPLES) -> Fit:
    """Measure ``obj``'s collision geoms against its visual mesh. See ``DEFAULT_TOLERANCE_MM``.
    Without a visual mesh only the resting height and extent are measured."""
    collision = [geom_mesh(obj, g) for g in collision_geoms(obj)]
    col_vertices = np.vstack([c.vertices for c in collision])
    bottom = col_vertices[:, 2].min()
    resting = _mm(1e3 * bottom)
    collision_extent = [_mm(1e3 * v) for v in np.ptp(col_vertices, axis=0)]
    if not visual_geoms(obj):
        return Fit(resting=resting, collision_extent=collision_extent)
    visual = visual_points(obj, n)
    col_points = trimesh.sample.sample_surface(trimesh.util.concatenate(collision), n, seed=1)[0]
    vis_vertices = np.vstack([geom_mesh(obj, g).vertices for g in visual_geoms(obj)])

    outside = 1e3 * distance_outside(obj, visual)
    excess = 1e3 * _outside_hull(vis_vertices, col_points)
    return Fit(
        resting=resting,
        collision_extent=collision_extent,
        visual_outside_p99=_mm(np.percentile(outside, 99)),
        visual_outside_max=_mm(outside.max()),
        collision_excess_p99=_mm(np.percentile(excess, 99)),
        collision_excess_max=_mm(excess.max()),
        sink=_mm(1e3 * (bottom - vis_vertices[:, 2].min())),
        **_floor_probe(obj),
        visual_extent=[_mm(1e3 * v) for v in np.ptp(vis_vertices, axis=0)],
    )


def is_open(obj: Compiled) -> bool:
    """Whether the object declares an open interior that things can be put into."""
    value = (obj.meta.get("geometric_properties") or {}).get("open", False)
    if not isinstance(value, bool):
        raise ValueError(f"{obj.name}: geometric_properties.open must be true or false, not {value!r}")
    return value


def visual_floor(obj: Compiled, xy: tuple[float, float] | None = None) -> float:
    """Height (m) of the first visual surface hit looking straight down at ``xy``
    (default: above the body origin)."""
    m, d = obj.model, obj.data
    x, y = d.xpos[obj.body][:2] if xy is None else xy
    # Off the exact axis: a ray through a mesh vertex can slip between its triangles.
    x, y = x + 5e-4, y + 5e-4
    top = max(geom_mesh(obj, g).vertices[:, 2].max() for g in visual_geoms(obj)) + 0.1
    groups = np.zeros(6, dtype=np.uint8)
    groups[VISUAL_GROUP] = 1
    hit = np.zeros(1, dtype=np.int32)
    dist = mujoco.mj_ray(m, d, np.array([x, y, top]), np.array([0.0, 0.0, -1.0]), groups, 1, -1, hit)
    if dist < 0:
        raise NoSurfaceBelow(f"{obj.name}: no visual surface below ({x:.3f}, {y:.3f})")
    return top - dist


class NoSurfaceBelow(ValueError):
    """No visual surface lies beneath the point looked down from."""


class ProbeLeftObject(ValueError):
    """The dropped probe came to rest where no visual surface lies beneath it."""


def _floor_probe(obj: Compiled) -> dict:
    if not is_open(obj):
        return {}
    try:
        return dict(floor_gap=floor_gap(obj))
    except ProbeLeftObject:
        return dict(probe_left_object=True)


def floor_gap(obj: Compiled, xy: tuple[float, float] | None = None) -> float:
    """Millimetres a 1 cm sphere dropped at ``xy`` (default: above the body origin),
    held in the fixed object, comes to rest above the visual inner floor.

    This is the larger of its height above the visual surface beneath where it comes to
    rest, and above the visual floor beneath the drop point. A filled interior holds the
    probe up by its depth. A probe that rolls downhill inside an open interior (an
    interior need not be centred on the body origin) scores about zero on both; one
    that rolls off a filled top and comes to rest on a handle stays high above the floor
    beneath the drop point. A probe with no visual surface beneath it, such as one that
    rolled off a solid bowl onto the ground, raises ``ProbeLeftObject``."""
    floor_at_drop = visual_floor(obj, xy)
    scene = mujoco.MjSpec()
    scene.option.timestep = 0.002
    scene.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1])
    source = obj.spec.copy()
    scene.worldbody.add_frame().attach_body(next(iter(source.worldbody.bodies)), "obj/", "")
    for joint in list(scene.joints):
        scene.delete(joint)
    x, y = obj.data.xpos[obj.body][:2] if xy is None else xy
    top = max(geom_mesh(obj, g).vertices[:, 2].max() for g in collision_geoms(obj))
    probe = scene.worldbody.add_body(name="probe", pos=[x, y, top + 0.05])
    probe.add_freejoint()
    probe.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[_PROBE_RADIUS, 0, 0], mass=0.02)
    model = scene.compile()
    data = mujoco.MjData(model)
    for _ in range(1500):
        mujoco.mj_step(model, data)
    position = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "probe")]
    try:
        floor_at_rest = visual_floor(obj, (position[0], position[1]))
    except NoSurfaceBelow as error:
        raise ProbeLeftObject(f"{obj.name}: the dropped probe came to rest off the object") from error
    bottom = position[2] - _PROBE_RADIUS
    return _mm(1e3 * max(bottom - floor_at_rest, bottom - floor_at_drop))


# ---------------------------------------------------------------------- placement
#
# Every field of a ``placement`` section (docs/schema.md) is measured here from the
# compiled model at the model file's pose, which rests the object on z = 0. How the
# object itself rests is not declared: a placement planner works it out from the
# collision geometry. ``check``
# compares what an object declares with these measurements; ``measure_placement`` and
# ``placement_yaml`` write them, so nothing in the section is typed by hand.

_RIM_BELOW = 0.003  # opening rays run this far below the rim, inside the walls
_RAYS = 64  # directions sampled around an opening
_ROUNDNESS = 1.25  # a circle's longest ray over its shortest (a square's is 1.41)
_TOP_PLANE = 1e-4  # vertices this close to the highest point lie on a support's top
_ROUND = 5  # lengths to 10 micrometres
_ROUND_ROTATION = 10  # rotation entries: 1e-5 would break the 1e-6 rigid-transform check


def _asset_pose(obj: Compiled) -> np.ndarray:
    return _pose(obj.data.xpos[obj.body], obj.data.xmat[obj.body])


def _collision_vertices(obj: Compiled) -> np.ndarray:
    return np.vstack([geom_mesh(obj, g).vertices for g in collision_geoms(obj)])


def _collision_ray(obj: Compiled, groups: np.ndarray, start, direction) -> float:
    """Distance to the first geom in ``groups`` hit, or NaN."""
    hit = np.zeros(1, dtype=np.int32)
    start, direction = np.asarray(start, dtype=float), np.asarray(direction, dtype=float)
    dist = mujoco.mj_ray(obj.model, obj.data, start, direction, groups, 1, -1, hit)
    return dist if dist >= 0 else math.nan


def _ray_groups(obj: Compiled) -> np.ndarray:
    """The geom groups rays see: the collision geoms' groups, which hold no visual geom
    when the object has one (``check``). A mesh geom collides as its convex hull while a
    ray meets its triangles, so a non-convex collision mesh is refused: its opening would
    be measured where simulation has none."""
    m = obj.model
    groups = np.zeros(mujoco.mjNGROUP, dtype=np.uint8)
    for g in collision_geoms(obj):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = geom_mesh(obj, g)  # a hull part saved and reloaded is convex to round-off
            if mesh.convex_hull.volume > 1.01 * abs(mesh.volume):
                raise ValueError(f"{obj.name}: collision mesh geom {g} is not convex; MuJoCo collides it as its hull")
        if m.geom_group[g] >= mujoco.mjNGROUP:
            raise ValueError(f"{obj.name}: collision geom {g} is in group {m.geom_group[g]}, which rays cannot see")
        groups[m.geom_group[g]] = 1
    return groups


def _rounded(T: np.ndarray) -> list[list[float]]:
    out = [[round(float(v), _ROUND_ROTATION) + 0.0 for v in row] for row in T]
    for row in range(3):
        out[row][3] = round(float(T[row, 3]), _ROUND) + 0.0
    return out


def _in_asset(obj: Compiled, T_world: np.ndarray) -> list[list[float]]:
    return _rounded(np.linalg.inv(_asset_pose(obj)) @ T_world)


def _level_axes(obj: Compiled) -> np.ndarray:
    """The axes openings and supports are measured along, as world-frame columns at the
    model file's pose: z up, x the asset's first horizontal axis, y completing them. An
    asset with no axis vertical there is refused: its horizontal axes do not name the
    sides of a rectangle."""
    R = obj.data.xmat[obj.body].reshape(3, 3)
    vertical = [i for i in range(3) if abs(abs(R[2, i]) - 1.0) < 1e-6]
    if not vertical:
        raise ValueError(f"{obj.name}: no asset axis is vertical at the model file's pose")
    x = R[:, min(i for i in range(3) if i != vertical[0])].copy()
    x[2] = 0.0
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross([0.0, 0.0, 1.0], x), [0.0, 0.0, 1.0]])


def measure_opening(obj: Compiled, shape: str, name: str = "top") -> Opening:
    """``obj``'s top opening, from rays cast in its collision geometry along the
    ``_level_axes``: the rim at the top of the geometry, the interior centred between
    opposing walls just below the rim, and the floor straight below that centre. A
    rectangle's size is the wall-to-wall width along x and y; a circle's radius is the
    shortest of ``_RAYS`` rays to the wall, the largest circle that fits. An opening the
    shape does not describe raises ``ValueError``: a rectangle any of whose rays ends
    more than ``SIZE_TOLERANCE_MM`` from its outline, or a circle whose longest ray is
    more than ``_ROUNDNESS`` times its shortest."""
    if shape not in SHAPES:
        raise ValueError(f"{obj.name}: opening shape must be one of {tuple(SHAPES)}, not {shape!r}")
    axes = _level_axes(obj)
    rim = float(_collision_vertices(obj)[:, 2].max())
    centre = obj.data.xpos[obj.body].copy()
    centre[2] = rim - _RIM_BELOW
    groups = _ray_groups(obj)

    def wall(a: float, b: float) -> float:
        dist = _collision_ray(obj, groups, centre, a * axes[:, 0] + b * axes[:, 1])
        if math.isnan(dist):
            raise ValueError(f"{obj.name}: no wall around the opening along ({a:.3f}, {b:.3f})")
        return dist

    for _ in range(3):  # opposing walls move the centre to the interior's middle
        centre += (wall(1, 0) - wall(-1, 0)) / 2 * axes[:, 0] + (wall(0, 1) - wall(0, -1)) / 2 * axes[:, 1]
    hx, hy = (wall(1, 0) + wall(-1, 0)) / 2, (wall(0, 1) + wall(0, -1)) / 2
    depth = _collision_ray(obj, groups, [*centre[:2], rim], [0.0, 0.0, -1.0])
    if math.isnan(depth):
        raise ValueError(f"{obj.name}: no floor below the opening")
    angles = np.linspace(0, 2 * math.pi, _RAYS, endpoint=False)
    rays = np.array([wall(math.cos(a), math.sin(a)) for a in angles])
    if shape == "rectangle":
        with np.errstate(divide="ignore"):
            outline = np.minimum(hx / np.abs(np.cos(angles)), hy / np.abs(np.sin(angles)))
        worst = int(np.argmax(np.abs(rays - outline)))
        if abs(rays[worst] - outline[worst]) > SIZE_TOLERANCE_MM / 1e3:
            raise ValueError(
                f"{obj.name}: the opening is not a {2 * hx:.4f} by {2 * hy:.4f} m rectangle: at "
                f"{math.degrees(angles[worst]):.1f} degrees its wall is {rays[worst]:.4f} m from the "
                f"centre, not {outline[worst]:.4f} m"
            )
        size = [round(2 * hx, _ROUND), round(2 * hy, _ROUND)]
    else:
        if rays.max() > _ROUNDNESS * rays.min():
            raise ValueError(
                f"{obj.name}: the opening is not round: its wall is {rays.min():.4f} to {rays.max():.4f} m "
                f"from the centre, more than {_ROUNDNESS} times apart"
            )
        size = round(float(rays.min()), _ROUND)
    frame = np.eye(4)
    frame[:3, :3] = axes
    frame[:3, 3] = [*centre[:2], rim]
    return Opening(name, shape, size, round(float(depth), _ROUND), _in_asset(obj, frame))


def measure_support(obj: Compiled, name: str = "top") -> Support:
    """``obj``'s top surface as a rectangle along the ``_level_axes``: the height of its
    collision geometry's highest point, and the bounds of the collision vertices within
    ``_TOP_PLANE`` of it, so a geom that flares below the top adds nothing. A top whose
    outline falls short of its bounding rectangle by more than ``SIZE_TOLERANCE_MM``
    along its perimeter (a round or bevelled top) raises ``ValueError``."""
    axes = _level_axes(obj)
    vertices = _collision_vertices(obj)
    height = float(vertices[:, 2].max())
    top = vertices[vertices[:, 2] > height - _TOP_PLANE] @ axes[:, :2]
    low, high = top.min(axis=0), top.max(axis=0)
    extent = high - low
    try:
        area = ConvexHull(top).volume
    except QhullError:  # fewer than three points, or all on a line: no area
        area = 0.0
    if extent.prod() - area > SIZE_TOLERANCE_MM / 1e3 * 2 * extent.sum():
        raise ValueError(f"{obj.name}: the top is not a rectangle: it fills {area:.4f} of {extent.prod():.4f} m^2")
    frame = np.eye(4)
    frame[:3, :3] = axes
    frame[:3, 3] = axes[:, :2] @ ((low + high) / 2) + [0.0, 0.0, height]
    return Support(name, "rectangle", [round(float(e), _ROUND) for e in extent], _in_asset(obj, frame))


def measure_placement(
    obj: Compiled, openings: dict[str, str] | None = None, supports: tuple[str, ...] = ()
) -> Placement | None:
    """The openings (``{name: shape}``) and supports (names) asked for, each measured;
    ``None`` when none is asked for."""
    if not openings and not supports:
        return None
    return Placement(
        openings=tuple(measure_opening(obj, shape, name) for name, shape in (openings or {}).items()),
        supports=tuple(measure_support(obj, name) for name in supports),
    )


def remeasure_placement(obj: Compiled, declared: Placement | None) -> Placement | None:
    """``measure_placement`` with the openings and supports ``declared`` names, as after a
    collider change; ``None`` when nothing is declared."""
    if declared is None:
        return None
    return measure_placement(
        obj, {o.name: o.shape for o in declared.openings}, tuple(s.name for s in declared.supports)
    )


def placement_section(placement: Placement) -> dict:
    """``placement`` as the mapping its ``meta.yaml`` section loads as."""
    return yaml.safe_load(placement_yaml(placement))["placement"]


def placement_yaml(placement: Placement) -> str:
    """``placement`` as a ``meta.yaml`` section, in the layout of the catalog's files."""

    def n(value: float) -> str:  # fixed point: YAML reads "1e-05" as a string
        text = f"{value:.{_ROUND_ROTATION}f}".rstrip("0").rstrip(".")
        return "0" if text in ("", "-0") else text

    def name(value: str) -> str:  # a name YAML would read as something else is quoted
        return value if yaml.safe_load(value) == value else f'"{value}"'

    def matrix(T) -> str:
        return "[" + ", ".join("[" + ", ".join(n(v) for v in row) + "]" for row in T) + "]"

    def outline(entry) -> str:
        if entry.shape == "rectangle":
            return f"    size: [{n(entry.size[0])}, {n(entry.size[1])}]\n"
        return f"    radius: {n(entry.size)}\n"

    lines = "placement:\n  version: 1\n"
    if placement.openings:
        lines += "  openings:\n"
        for opening in placement.openings:
            lines += f"  - name: {name(opening.name)}\n    shape: {opening.shape}\n" + outline(opening)
            lines += f"    depth: {n(opening.depth)}\n    T_asset_opening: {matrix(opening.T_asset_opening)}\n"
    if placement.supports:
        lines += "  supports:\n"
        for support in placement.supports:
            lines += f"  - name: {name(support.name)}\n    shape: {support.shape}\n" + outline(support)
            lines += f"    T_asset_support: {matrix(support.T_asset_support)}\n"
    return lines


def with_placement(text: str, placement: Placement) -> str:
    """``meta.yaml`` text with its ``placement`` section replaced by ``placement``, or
    one appended, and every other line, comments included, as it was. The section ends
    at the next line that starts in column 0, a comment heading the next section
    included. A result that would not read back as ``placement`` is refused."""
    lines = text.splitlines(keepends=True)
    start = next((i for i, line in enumerate(lines) if line.startswith("placement:")), None)
    if start is None:
        out = "".join(lines).rstrip("\n") + "\n\n" + placement_yaml(placement)
    else:
        end = next((i for i in range(start + 1, len(lines)) if re.match(r"\S", lines[i])), len(lines))
        while end > start + 1 and not lines[end - 1].strip():
            end -= 1
        out = "".join(lines[:start]) + placement_yaml(placement) + "".join(lines[end:])
    if parse_placement(yaml.safe_load(out)) != placement:
        raise ValueError("the written placement section would not read back as measured")
    return out


def write_placement(obj: Compiled, placement: Placement) -> Path:
    """Write ``placement`` into ``obj``'s ``meta.yaml`` (``with_placement``), atomically."""
    path = obj.dir / "meta.yaml"
    _write_atomic(path, with_placement(path.read_text(), placement))
    return path


def _placement_failures(obj: Compiled, declared: Placement) -> list[str]:
    """How ``declared`` differs from what ``obj``'s compiled model measures."""
    failures = []
    tol = SIZE_TOLERANCE_MM / 1e3

    def frames(field, a, b):
        a, b = np.asarray(a), np.asarray(b)
        if np.abs(a[:3, 3] - b[:3, 3]).max() > tol or np.abs(a[:3, :3] - b[:3, :3]).max() > 1e-3:
            failures.append(f"{field} differs from the measured frame {np.round(b, 4).tolist()}")

    def sizes(field, a, b):
        if np.abs(np.atleast_1d(a) - np.atleast_1d(b)).max() > tol:
            failures.append(f"{field} {a} differs from the measured {b}")

    for opening in declared.openings:
        field = f"placement.openings.{opening.name}"
        try:
            found = measure_opening(obj, opening.shape, opening.name)
        except ValueError as error:
            failures.append(f"{field}: {error}")
            continue
        sizes(f"{field}.{SHAPES[opening.shape]}", opening.size, found.size)
        sizes(f"{field}.depth", opening.depth, found.depth)
        frames(f"{field}.T_asset_opening", opening.T_asset_opening, found.T_asset_opening)
    for support in declared.supports:
        field = f"placement.supports.{support.name}"
        found = measure_support(obj, support.name)
        sizes(f"{field}.size", support.size, found.size)
        frames(f"{field}.T_asset_support", support.T_asset_support, found.T_asset_support)
    return failures


# ------------------------------------------------------------------ grasp regions
#
# Each region a recipe (prl_assets.grasp_recipes) names is fitted to the visual points
# in its region of interest, in the asset frame at the model file's pose, z up: the
# samples of ``visual_points`` and every visual vertex, so that the extremes are exact.
# Every number in a recipe object's ``grasp_regions`` section is measured here, and
# ``check`` holds the section to its re-measurement.

COVERAGE_TOLERANCE_M = 1e-5  # the rounding of written values: every point lies within it
TURN_TOLERANCE_DEG = 0.5  # how far a declared region's frame may turn from its measurement
_MIN_ROI_POINTS = 200
_MIN_ROI_EXTENT_M = 1e-3


def select_points(points: np.ndarray, roi, exclude, name: str, yaw_deg: float = 0.0) -> np.ndarray:
    """``points`` (asset frame) inside ``roi``'s box (all of them when ``None``), less
    those inside any of ``exclude``'s ``(box, reason)``, refused when too few or flat to
    measure a region ``name`` from. The boxes are in the asset frame turned ``yaw_deg``
    about its z."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    turned = points @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # R^T p, per row
    keep = np.ones(len(points), dtype=bool)
    if roi is not None:
        keep &= np.all((turned >= roi[0]) & (turned <= roi[1]), axis=1)
    for box, _reason in exclude:
        keep &= ~np.all((turned >= box[0]) & (turned <= box[1]), axis=1)
    return _measurable(points[keep], name)


def _measurable(points: np.ndarray, name: str) -> np.ndarray:
    if len(points) < _MIN_ROI_POINTS:
        raise ValueError(f"region {name!r}: {len(points)} points, fewer than {_MIN_ROI_POINTS} to measure from")
    extent = np.ptp(points, axis=0)
    if extent.min() < _MIN_ROI_EXTENT_M:
        raise ValueError(f"region {name!r}: extent {np.round(extent, 4).tolist()} m is flat")
    return points


def _enclosing_circle(xy: np.ndarray) -> tuple[np.ndarray, float]:
    """The smallest circle enclosing ``xy``: Welzl's algorithm over the points' hull, in
    a fixed order, so the same points give the same circle."""
    hull = xy[ConvexHull(xy).vertices]
    order = np.random.default_rng(0).permutation(len(hull))

    def circle(boundary):
        if not boundary:
            return np.zeros(2), -1.0
        if len(boundary) == 1:
            return boundary[0], 0.0
        if len(boundary) == 2:
            a, b = boundary
            return (a + b) / 2, float(np.linalg.norm(a - b)) / 2
        a, b, c = boundary
        d = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
        sa, sb, sc = a @ a, b @ b, c @ c
        centre = np.array(
            [
                (sa * (b[1] - c[1]) + sb * (c[1] - a[1]) + sc * (a[1] - b[1])) / d,
                (sa * (c[0] - b[0]) + sb * (a[0] - c[0]) + sc * (b[0] - a[0])) / d,
            ]
        )
        return centre, float(np.linalg.norm(a - centre))

    def inside(centre, radius, p):
        return radius >= 0 and np.linalg.norm(p - centre) <= radius + 1e-12

    points = [hull[i] for i in order]
    centre, radius = circle([])
    for i, p in enumerate(points):
        if inside(centre, radius, p):
            continue
        centre, radius = p, 0.0
        for j, q in enumerate(points[:i]):
            if inside(centre, radius, q):
                continue
            centre, radius = circle([p, q])
            for r in points[:j]:
                if not inside(centre, radius, r):
                    centre, radius = circle([p, q, r])
    return np.asarray(centre), radius


def _min_rectangle(xy: np.ndarray) -> tuple[float, tuple[float, float], np.ndarray]:
    """The minimum-area rectangle enclosing ``xy``, canonicalized: (yaw in degrees, its
    sides along the yaw and across it, its centre). The shorter side lies along the yaw,
    which is in (-90, 90]; when the sides differ by less than ``SIZE_TOLERANCE_MM`` the
    side nearer asset x does, and the yaw is in (-45, 45]. Among hull edges that give
    equal areas, the one whose yaw is nearest asset x wins."""
    hull = xy[ConvexHull(xy).vertices]
    best = None
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        angle = math.atan2(b[1] - a[1], b[0] - a[0])
        c, s = math.cos(angle), math.sin(angle)
        local = hull @ np.array([[c, -s], [s, c]])  # the hull in the edge's frame
        lo, hi = local.min(axis=0), local.max(axis=0)
        area = float(np.prod(hi - lo))
        yaw = (math.degrees(angle) + 90.0) % 180.0 - 90.0  # one of the rectangle's axes
        key = (round(area / 1e-9), abs(yaw))
        if best is None or key < best[0]:
            centre = np.array([[c, -s], [s, c]]) @ ((lo + hi) / 2)
            best = (key, yaw, tuple(float(v) for v in hi - lo), centre)
    _, yaw, (along, across), centre = best
    if along > across:  # the shorter side along the yaw
        yaw, along, across = yaw + 90.0, across, along
    if abs(along - across) < SIZE_TOLERANCE_MM / 1e3:  # a near square: the axis nearer x
        yaw = (yaw + 45.0) % 90.0 - 45.0
        if yaw <= -45.0:
            yaw += 90.0
    else:
        yaw = (yaw + 90.0) % 180.0 - 90.0
        if yaw <= -90.0:
            yaw += 180.0
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    local = xy @ np.array([[c, -s], [s, c]])
    lo, hi = local.min(axis=0), local.max(axis=0)
    centre = np.array([[c, -s], [s, c]]) @ ((lo + hi) / 2)
    return yaw, (float(hi[0] - lo[0]), float(hi[1] - lo[1])), centre


def _region_frame(yaw_deg: float, centre) -> list[list[float]]:
    T = np.eye(4)
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    T[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    T[:3, 3] = centre
    return _rounded(T)


def _covering(value: float) -> float:
    """A length rounded up to the written precision, so a rounded region still covers."""
    return math.ceil(value * 10**_ROUND) / 10**_ROUND


def fit_cylinder_region(points: np.ndarray, name: str, strategies, axis: str = "upright") -> GraspRegion:
    """The cylinder that encloses ``points`` (asset frame). Upright, about the asset's
    vertical: through the smallest circle enclosing their xy, over their z range. Lying,
    along the long side of their xy's minimum-area rectangle (``_min_rectangle``), its
    direction's yaw in (-90, 90]: through the smallest circle enclosing them across that
    axis, over their range along it. The region's z is the cylinder's axis; lying, its
    x is horizontal and its y is up."""
    points = _measurable(points, name)
    if axis == "upright":
        centre, radius = _enclosing_circle(points[:, :2])
        z0, z1 = float(points[:, 2].min()), float(points[:, 2].max())
        frame, length = _region_frame(0.0, (*centre, (z0 + z1) / 2)), z1 - z0
    elif axis == "lying":
        yaw, _, _ = _min_rectangle(points[:, :2])
        along = math.radians(yaw + 90.0 if yaw <= 0.0 else yaw - 90.0)  # the long side
        u = np.array([math.cos(along), math.sin(along), 0.0])
        v = np.array([-math.sin(along), math.cos(along), 0.0])
        R = np.column_stack([v, [0.0, 0.0, 1.0], u])
        local = points @ R  # across, up, along
        centre, radius = _enclosing_circle(local[:, :2])
        t0, t1 = float(local[:, 2].min()), float(local[:, 2].max())
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = R @ [*centre, (t0 + t1) / 2]
        frame, length = _rounded(T), t1 - t0
    else:
        raise ValueError(f"region {name!r}: axis {axis!r} is neither upright nor lying")
    return GraspRegion(
        name, "cylinder", {"radius": _covering(radius), "height": _covering(length)}, frame, tuple(strategies)
    )


def fit_box_region(points: np.ndarray, name: str, strategies) -> GraspRegion:
    """The upright box that encloses ``points`` (asset frame): the minimum-area rectangle
    of their xy hull (``_min_rectangle``), over their z range; its x is the shorter side."""
    points = _measurable(points, name)
    yaw, (sx, sy), centre = _min_rectangle(points[:, :2])
    z0, z1 = float(points[:, 2].min()), float(points[:, 2].max())
    size = [_covering(sx), _covering(sy), _covering(z1 - z0)]
    return GraspRegion(name, "box", {"size": size}, _region_frame(yaw, (*centre, (z0 + z1) / 2)), tuple(strategies))


def _fit(recipe, points: np.ndarray) -> GraspRegion:
    if recipe.primitive == "cylinder":
        return fit_cylinder_region(points, recipe.name, recipe.strategies, recipe.axis)
    return fit_box_region(points, recipe.name, recipe.strategies)


def _visual_asset_points(obj: Compiled) -> np.ndarray:
    vertices = np.vstack([geom_mesh(obj, g).vertices for g in visual_geoms(obj)])
    return to_body(obj, np.vstack([visual_points(obj), vertices]))


def _level(obj: Compiled) -> None:
    R = obj.data.xmat[obj.body].reshape(3, 3)
    if not np.allclose(R[:, 2], [0.0, 0.0, 1.0], atol=1e-6):
        raise ValueError(f"{obj.name}: its asset z is not vertical at the model file's pose")


def _measured(obj: Compiled) -> list:
    """Per recipe region of ``obj``: the recipe, the points it selects, and their fit."""
    _level(obj)
    points = _visual_asset_points(obj)
    rows = []
    for recipe in GRASP_RECIPES[obj.name]:
        selected = select_points(points, recipe.roi, recipe.exclude, recipe.name, recipe.roi_yaw_deg)
        rows.append((recipe, selected, _fit(recipe, selected)))
    return rows


def measure_grasp_regions(obj: Compiled) -> tuple[GraspRegion, ...]:
    """``obj``'s grasp regions, measured by its recipe (``GRASP_RECIPES``)."""
    return tuple(fitted for _, _, fitted in _measured(obj))


def _outside(region: GraspRegion, points: np.ndarray) -> tuple[float, dict[str, float]]:
    """How far the farthest of ``points`` lies outside ``region``, and per face, how far
    the nearest point lies inside it: 0 where a point touches the face."""
    T = np.asarray(region.T_asset_region)
    local = (points - T[:3, 3]) @ T[:3, :3]
    if region.primitive == "box":
        half = np.asarray(region.params["size"]) / 2
        gaps = {
            f"{sign}{axis}": float((half[i] - sign_v * local[:, i]).min())
            for i, axis in enumerate("xyz")
            for sign, sign_v in (("+", 1), ("-", -1))
        }
        outside = float(np.max(np.abs(local) - half))
    else:
        r, h = region.params["radius"], region.params["height"] / 2
        radial = np.hypot(local[:, 0], local[:, 1])
        gaps = {
            "radius": float(r - radial.max()),
            "+z": float(h - local[:, 2].max()),
            "-z": float(h + local[:, 2].min()),
        }
        outside = float(max((radial - r).max(), (np.abs(local[:, 2]) - h).max()))
    return max(outside, 0.0), gaps


def _finite(box) -> list:
    """A region-of-interest box with an unbounded side as null: JSON has no infinity."""
    return [[v if math.isfinite(v) else None for v in corner] for corner in box]


def grasp_region_evidence(obj: Compiled) -> dict:
    """What ``obj``'s recipe measures, region by region, for the record: the SHA-256 of
    its compiled visual vertices, each region of interest and its exclusions, the points
    selected, the fitted region, how far any point lies outside it, how near a point comes
    to each face, and how the declared region differs."""
    vertices = np.vstack([geom_mesh(obj, g).vertices for g in visual_geoms(obj)])
    declared = {r.name: r for r in parse_grasp_regions(obj.meta)}
    rows = []
    for recipe, selected, fitted in _measured(obj):
        outside, gaps = _outside(fitted, selected)
        rows.append(
            {
                "name": recipe.name,
                "roi": None if recipe.roi is None else _finite(recipe.roi),
                "roi_frame": "asset" if not recipe.roi_yaw_deg else f"asset turned {recipe.roi_yaw_deg} deg about z",
                "roi_yaw_deg": recipe.roi_yaw_deg,
                "exclude": [{"box": _finite(box), "reason": why} for box, why in recipe.exclude],
                "selected_points": int(len(selected)),
                "fitted": _region_row(fitted),
                "coverage_residual_m": outside,
                "face_gaps_m": gaps,
                "declared_difference": _region_difference(declared.get(recipe.name), fitted),
            }
        )
    digest = hashlib.sha256(np.ascontiguousarray(to_body(obj, vertices), dtype=np.float64).tobytes()).hexdigest()
    return {"visual_vertices_sha256": digest, "regions": rows}


def _corners(region: GraspRegion) -> np.ndarray:
    """A box region's eight corners, or a cylinder region's two end centres, asset frame."""
    T = np.asarray(region.T_asset_region)
    if region.primitive == "box":
        half = np.asarray(region.params["size"]) / 2
        local = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
    else:
        local = np.array([[0.0, 0.0, -1.0], [0.0, 0.0, 1.0]]) * region.params["height"] / 2
    return local @ T[:3, :3].T + T[:3, 3]


def _region_difference(declared: GraspRegion | None, measured: GraspRegion) -> dict | None:
    """How ``declared`` differs from ``measured`` as geometry, whatever frame each names
    it by: a box is the same box turned a half turn about its axis (or a quarter, when
    both are near-square and its strategies grip x and y alike: ``face_x`` with
    ``face_y`` or neither), and a cylinder the same with its axis reversed. ``geometry_m``
    is the farthest a box corner, or a cylinder's end centre, lies from its nearest
    counterpart either way, or the radius difference if larger; ``turn_deg`` the angle
    between their axes (a box: their yaws, modulo its symmetry). None when nothing is
    declared."""
    if declared is None:
        return None
    if declared.primitive != measured.primitive:
        return {"same_kind": False, "geometry_m": math.inf, "turn_deg": math.inf}
    a, b = _corners(declared), _corners(measured)
    nearest = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    gap = float(max(nearest.min(axis=1).max(), nearest.min(axis=0).max()))
    Ra, Rb = np.asarray(declared.T_asset_region)[:3, :3], np.asarray(measured.T_asset_region)[:3, :3]
    if declared.primitive == "cylinder":
        gap = max(gap, abs(declared.params["radius"] - measured.params["radius"]))
        turn = math.degrees(math.acos(min(1.0, abs(float(Ra[:, 2] @ Rb[:, 2])))))
    else:
        square = all(
            abs(r.params["size"][0] - r.params["size"][1]) < SIZE_TOLERANCE_MM / 1e3 for r in (declared, measured)
        ) and ("face_x" in declared.strategies) == ("face_y" in declared.strategies)
        period = 90.0 if square else 180.0
        yaw = math.degrees(math.atan2(Rb[1, 0], Rb[0, 0]) - math.atan2(Ra[1, 0], Ra[0, 0])) % period
        turn = min(yaw, period - yaw)
    same_kind = declared.strategies == measured.strategies
    return {"same_kind": same_kind, "geometry_m": gap, "turn_deg": turn}


def _grasp_region_failures(obj: Compiled, declared: tuple[GraspRegion, ...]) -> list[str]:
    """How ``declared`` differs from its re-measurement by ``obj``'s recipe."""
    recipe = GRASP_RECIPES[obj.name]
    if not declared:
        return ["grasp_regions: a recipe object declares none: write them with scripts/author_grasp_regions.py"]
    if [r.name for r in declared] != [r.name for r in recipe]:
        return [
            f"grasp_regions: regions {[r.name for r in declared]} differ from the recipe's {[r.name for r in recipe]}"
        ]
    try:
        measured = measure_grasp_regions(obj)
    except (ValueError, QhullError) as error:  # a degenerate region of interest
        return [f"grasp_regions: {error}"]
    failures = []
    for d, m in zip(declared, measured):
        diff = _region_difference(d, m)
        field = f"grasp_regions.{d.name}"
        if not diff["same_kind"]:
            failures.append(
                f"{field}: {d.primitive} {d.strategies} differs from the recipe's {m.primitive} {m.strategies}"
            )
        elif diff["geometry_m"] > SIZE_TOLERANCE_MM / 1e3 or diff["turn_deg"] > TURN_TOLERANCE_DEG:
            failures.append(f"{field}: differs from its measurement {_region_row(m)}")
    return failures


def _half_extent(region: GraspRegion) -> np.ndarray:
    """A region's half extent along each of its own axes: ``bounds_in_asset`` in its own
    frame."""
    own = GraspRegion(region.name, region.primitive, region.params, IDENTITY, region.strategies)
    return own.bounds_in_asset()[1]


def _region_row(region: GraspRegion) -> dict:
    row = {"name": region.name, "primitive": region.primitive}
    row.update({k: list(v) if isinstance(v, tuple) else v for k, v in region.params.items()})
    row["T_asset_region"] = [list(r) for r in region.T_asset_region]
    row["strategies"] = list(region.strategies)
    return row


def grasp_regions_yaml(regions: tuple[GraspRegion, ...]) -> str:
    """``regions`` as a ``meta.yaml`` section."""

    def n(value: float) -> str:  # fixed point: YAML reads "1e-05" as a string
        text = f"{value:.{_ROUND_ROTATION}f}".rstrip("0").rstrip(".")
        return "0" if text in ("", "-0") else text

    lines = "grasp_regions:\n  version: 1\n  regions:\n"
    for region in regions:
        lines += f"  - name: {region.name}\n    primitive: {region.primitive}\n"
        for key, value in region.params.items():
            text = "[" + ", ".join(n(v) for v in value) + "]" if isinstance(value, tuple) else n(value)
            lines += f"    {key}: {text}\n"
        matrix = "[" + ", ".join("[" + ", ".join(n(v) for v in row) + "]" for row in region.T_asset_region) + "]"
        lines += f"    T_asset_region: {matrix}\n    strategies: [{', '.join(region.strategies)}]\n"
    return lines


def grasp_regions_section(regions: tuple[GraspRegion, ...]) -> dict:
    """``regions`` as the mapping their ``meta.yaml`` section loads as."""
    return yaml.safe_load(grasp_regions_yaml(regions))["grasp_regions"]


def with_grasp_regions(text: str, regions: tuple[GraspRegion, ...]) -> str:
    """``meta.yaml`` text with its ``grasp_regions`` section replaced by ``regions``, or
    one appended, every other line as it was (``with_placement``'s rule). A result that
    would not read back as ``regions`` is refused."""
    lines = text.splitlines(keepends=True)
    start = next((i for i, line in enumerate(lines) if line.startswith("grasp_regions:")), None)
    if start is None:
        out = "".join(lines).rstrip("\n") + "\n\n" + grasp_regions_yaml(regions)
    else:
        end = next((i for i in range(start + 1, len(lines)) if re.match(r"\S", lines[i])), len(lines))
        while end > start + 1 and not lines[end - 1].strip():
            end -= 1
        out = "".join(lines[:start]) + grasp_regions_yaml(regions) + "".join(lines[end:])
    if parse_grasp_regions(yaml.safe_load(out)) != regions:
        raise ValueError("the written grasp_regions section would not read back as measured")
    return out


def write_grasp_regions(obj: Compiled, regions: tuple[GraspRegion, ...]) -> Path:
    """Write ``regions`` into ``obj``'s ``meta.yaml`` (``with_grasp_regions``), atomically."""
    path = obj.dir / "meta.yaml"
    _write_atomic(path, with_grasp_regions(path.read_text(), regions))
    return path


# ------------------------------------------------------------------------- checks


@dataclass
class Report:
    name: str
    fit: Fit
    failures: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.failures

    def __str__(self) -> str:
        f = self.fit
        lines = [f"{self.name}: {'OK' if self.ok else 'FAIL'}"]
        if f.visual_extent is None:
            lines += [
                "  no visual mesh: the collision geoms are what it shows",
                f"  collision bottom at {f.resting:6.1f} mm  extent {f.collision_extent} mm",
            ]
        else:
            outside = (f.visual_outside_p99, f.visual_outside_max)
            excess = (f.collision_excess_p99, f.collision_excess_max)
            lines += [
                f"  visual outside collision  p99 {outside[0]:6.1f} mm  max {outside[1]:6.1f} mm",
                f"  collision beyond visual   p99 {excess[0]:6.1f} mm  max {excess[1]:6.1f} mm",
                f"  visual below collision        {f.sink:6.1f} mm   collision bottom at {f.resting:6.1f} mm",
                f"  extent visual {f.visual_extent} mm  collision {f.collision_extent} mm",
            ]
        if f.floor_gap is not None:
            lines.append(f"  dropped sphere rests       {f.floor_gap:6.1f} mm above the visual floor")
        lines += [f"  - {msg}" for msg in self.failures]
        return "\n".join(lines)


def tolerances(obj: Compiled) -> dict[str, float]:
    """Defaults, widened by the object's ``geometric_properties.collision_tolerance_mm``."""
    declared = (obj.meta.get("geometric_properties") or {}).get("collision_tolerance_mm") or {}
    if not isinstance(declared, dict):
        raise ValueError(f"{obj.name}: collision_tolerance_mm must be a mapping, not {declared!r}")
    declared = dict(declared)
    reason = declared.pop("reason", None)
    if declared and not (isinstance(reason, str) and reason.strip()):
        raise ValueError(f"{obj.name}: collision_tolerance_mm must state a reason")
    unknown = set(declared) - set(DEFAULT_TOLERANCE_MM)
    if unknown:
        raise ValueError(f"{obj.name}: unknown collision tolerances {sorted(unknown)}")
    for key, value in declared.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not value >= 0:
            raise ValueError(f"{obj.name}: collision tolerance {key} must be a number of mm, not {value!r}")
    return {**DEFAULT_TOLERANCE_MM, **{k: float(v) for k, v in declared.items()}}


def declared_extents(meta: dict) -> dict[str, list[float]]:
    """The full extents (x, y, z; metres) that an object's metadata states, by field."""
    gp = meta.get("geometric_properties") or {}
    out = {}
    if "dimensions" in meta:
        out["dimensions"] = list(meta["dimensions"])
    for key in ("size", "outer_size"):
        if key in gp:
            out[f"geometric_properties.{key}"] = list(gp[key])
    for key in ("radius", "outer_radius"):
        if key in gp:
            d = 2 * gp[key]
            out[f"geometric_properties.{key} (diameter)"] = [d, d, gp["height"]] if "height" in gp else [d, d]
    return out


def _categories(obj: Compiled) -> list[str]:
    category = obj.meta.get("category") or []
    category = [category] if isinstance(category, str) else category
    if not isinstance(category, list) or not all(isinstance(c, str) for c in category):
        raise ValueError(f"{obj.name}: category must be a string or a list of strings, not {category!r}")
    return category


def check(obj: Compiled | str | Path) -> Report:
    """Check an object against the conventions in the README and the tolerances above."""
    if not isinstance(obj, Compiled):
        obj = load(obj)
    regions = parse_grasp_regions(obj.meta)
    placement = parse_placement(obj.meta)
    m = obj.model
    failures = []

    fixture = "fixture" in _categories(obj)
    tol = tolerances(obj)
    free = [j for j in range(m.njnt) if m.jnt_bodyid[j] == obj.body and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    if not fixture and not free:
        failures.append("no free joint on the top-level body (required unless category includes 'fixture')")
    collision = collision_geoms(obj)
    visual = visual_geoms(obj)
    if not collision:
        failures.append("no collision geoms")
    # An object without a visual mesh renders its collision geoms; they are its shape,
    # so there is nothing to compare them with.
    if visual and any(m.geom_group[g] != COLLISION_GROUP for g in collision):
        failures.append(f"collision geoms must be in group {COLLISION_GROUP} when a visual mesh is present")
    # A visual geom left at MuJoCo's default density adds mass, so this also catches it.
    # Mass is split across geoms and written to 6 significant digits.
    if "mass" in obj.meta and not math.isclose(m.body_mass[1:].sum(), obj.meta["mass"], rel_tol=1e-5):
        failures.append(f"compiled mass {m.body_mass[1:].sum():.4f} kg != declared {obj.meta['mass']} kg")
    if not collision:
        return Report(obj.name, Fit(resting=math.nan, collision_extent=[]), failures)

    if regions:
        if not visual:
            failures.append("grasp_regions: visual mesh required for in-frame bounds validation")
        else:
            vertices = to_body(obj, np.vstack([geom_mesh(obj, g).vertices for g in visual]))
            for region in regions:
                # Along the region's own axes, so that a turned region is not held to the
                # corners of an axis-aligned box it does not fill.
                T = np.asarray(region.T_asset_region)
                local = (vertices - T[:3, 3]) @ T[:3, :3]
                if np.any(-_half_extent(region) < local.min(axis=0) - 0.01) or np.any(
                    _half_extent(region) > local.max(axis=0) + 0.01
                ):
                    failures.append(f"grasp_regions.{region.name}.bounds: outside visual bounds plus 10 mm")
    if obj.name in GRASP_RECIPES:
        failures += _grasp_region_failures(obj, regions)
    if is_open(obj) and not (placement and placement.openings):
        failures.append("an open object declares no placement opening: write it with scripts/author_placement.py")
    if placement is not None:
        failures += _placement_failures(obj, placement)
    fit = collision_fit(obj)
    limits = [("resting", abs(fit.resting), "the model file's pose does not rest the object on z = 0")]
    if visual:
        limits += [
            ("visual_outside", fit.visual_outside_p99, "visual mesh lies outside the collision geoms (undersized)"),
            ("collision_excess", fit.collision_excess_p99, "collision geoms extend beyond the visual mesh"),
            ("sink", fit.sink, "visual mesh reaches below the collision geometry and sinks into a support"),
        ]
    if fit.floor_gap is not None:
        limits.append(("floor_gap", abs(fit.floor_gap), "a dropped object does not reach the visual inner floor"))
    if fit.probe_left_object:
        failures.append("a dropped object rolls off the object instead of entering its interior (floor_gap)")
    for key, value, why in limits:
        if value > tol[key]:
            failures.append(f"{why}: {value:.1f} mm > {tol[key]:.1f} mm ({key})")
    for field_name, extent in declared_extents(obj.meta).items():
        actual = fit.collision_extent[: len(extent)]
        if any(abs(1e3 * e - a) > SIZE_TOLERANCE_MM for e, a in zip(extent, actual)):
            stated = [_mm(1e3 * e) for e in extent]
            failures.append(f"{field_name} {stated} mm differs from the collision extent {actual} mm")
    return Report(obj.name, fit, failures)


# ----------------------------------------------------------------------- building


@dataclass
class GeomSpec:
    """A collision primitive in the body frame: MuJoCo type name, half-sizes, pose."""

    type: str
    size: list[float]
    pos: list[float]
    quat: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    extra: dict[str, str] = field(default_factory=dict)  # further MJCF attributes, e.g. solref

    @property
    def volume(self) -> float:
        s = self.size
        if self.type == "box":
            return 8 * s[0] * s[1] * s[2]
        if self.type == "cylinder":
            return math.pi * s[0] ** 2 * 2 * s[1]
        if self.type == "sphere":
            return 4 / 3 * math.pi * s[0] ** 3
        if self.type == "capsule":
            return math.pi * s[0] ** 2 * (2 * s[1] + 4 / 3 * s[0])
        raise ValueError(f"unsupported geom type {self.type!r}")


def hull_of(mesh: str) -> GeomSpec:
    """A mesh geom on the model's asset ``mesh``, which MuJoCo collides as its convex hull:
    exact for a convex object without a cavity whose faces bulge or whose edges are
    rounded too much for a box (a soft bag, a pillow). Use it alone; mass then needs no
    volume. MuJoCo makes one contact per hull pair by default, so check that the object
    rests and stacks stably."""
    return GeomSpec("mesh", [], [0.0, 0.0, 0.0], extra={"mesh": mesh})


def fit_box(points: np.ndarray) -> GeomSpec:
    """The axis-aligned box bounding ``points`` (body frame)."""
    lo, hi = points.min(axis=0), points.max(axis=0)
    return GeomSpec("box", list((hi - lo) / 2), list(np.round((hi + lo) / 2, 4)))


def radius_profile(points: np.ndarray, edges: np.ndarray) -> list[tuple[float, float, float, float]]:
    """Per height band ``[z0, z1)`` of ``edges``: (z0, z1, inner radius, outer radius)
    of ``points`` (body frame) about the body's z axis. Bands with no points are skipped."""
    r = np.linalg.norm(points[:, :2], axis=1)
    out = []
    for z0, z1 in zip(edges[:-1], edges[1:]):
        band = r[(points[:, 2] >= z0) & (points[:, 2] <= z1)]
        if len(band):
            out.append((float(z0), float(z1), float(band.min()), float(band.max())))
    return out


def stacked_cylinders(points: np.ndarray, band: float = 0.005, merge: float = 0.002) -> list[GeomSpec]:
    """Coaxial cylinders that enclose ``points`` (body frame) band by band, merging
    neighbouring bands whose radii differ by less than ``merge`` (m). Suits bottles and
    other solids of revolution with no interior to reach into."""
    z0, z1 = points[:, 2].min(), points[:, 2].max()
    edges = np.linspace(z0, z1, max(1, round((z1 - z0) / band)) + 1)
    runs: list[list[float]] = []
    for lo, hi, _, outer in radius_profile(points, edges):
        if runs and abs(runs[-1][2] - outer) < merge:
            runs[-1][1], runs[-1][2] = hi, max(runs[-1][2], outer)
        else:
            runs.append([lo, hi, outer])
    return [GeomSpec("cylinder", [r, (hi - lo) / 2], [0.0, 0.0, (hi + lo) / 2]) for lo, hi, r in runs]


def revolved_shell(
    profile: list[tuple[float, float]],
    thickness: float | list[float],
    floor: tuple[float, float, float],
    segments: int = 16,
) -> list[GeomSpec]:
    """An open container of revolution as boxes: a floor disc and a ring of ``segments``
    wall boxes for each consecutive pair of ``profile`` points.

    ``profile`` lists (z, outer radius) up the wall from the floor's edge to the rim;
    each wall box's outer face lies on the profile. ``thickness`` is the wall's thickness
    normal to its surface, one value or one per band. ``floor`` is (bottom z, top z,
    radius). ``fit_revolved_shell`` measures all three from a visual mesh.
    """
    bottom, top, radius = floor
    geoms = [GeomSpec("cylinder", [radius, (top - bottom) / 2], [0.0, 0.0, (top + bottom) / 2])]
    bands = list(zip(profile[:-1], profile[1:]))
    widths = [thickness] * len(bands) if isinstance(thickness, (int, float)) else list(thickness)
    if len(widths) != len(bands):
        raise ValueError(f"{len(widths)} thicknesses for {len(bands)} wall bands")
    for ((za, ra), (zb, rb)), thickness in zip(bands, widths):
        length = math.hypot(zb - za, rb - ra)
        tilt = math.atan2(rb - ra, zb - za)  # outward lean of the wall from vertical
        # Mid-wall radius, and the box's width: the outer arc's chord, plus overlap.
        mid_r = (ra + rb) / 2 - thickness / 2 * math.cos(tilt)
        width = 2 * max(ra, rb) * math.tan(math.pi / segments) * 1.05
        zc = (za + zb) / 2 + thickness / 2 * math.sin(tilt)
        for k in range(segments):
            phi = 2 * math.pi * k / segments
            # Box axes: x radial (thickness), y tangential (width), z along the wall.
            # Rotate about z by phi, then lean outward about the local y axis by tilt.
            quat = np.zeros(4)
            mujoco.mju_mulQuat(quat, _axis_quat([0, 0, 1], phi), _axis_quat([0, 1, 0], tilt))
            geoms.append(
                GeomSpec(
                    "box",
                    [thickness / 2, width / 2, length / 2],
                    [mid_r * math.cos(phi), mid_r * math.sin(phi), zc],
                    list(quat),
                )
            )
    return geoms


def fit_revolved_shell(
    points: np.ndarray,
    floor_top: float,
    bands: int = 6,
    segments: int = 16,
    min_thickness: float = 0.003,
) -> list[GeomSpec]:
    """``revolved_shell`` measured from ``points`` (body frame) of an open object of
    revolution about the body's z axis, such as a cup, bowl or plate. ``floor_top`` is
    the inner floor's height (``visual_floor`` gives it). The wall is split into
    ``bands`` equal heights; each band's outer radius and normal thickness come from the
    points, and the floor disc spans the wall's outer radius at the floor."""
    r = np.linalg.norm(points[:, :2], axis=1)
    z = points[:, 2]
    bottom, rim = z.min(), z.max()
    edges = np.linspace(floor_top, rim, bands + 1)
    step = (rim - floor_top) / bands

    def outer(at: float) -> float:
        near = np.abs(z - at) <= max(step / 4, 0.001)
        return float(r[near].max())

    profile = [(float(e), outer(e)) for e in edges]
    thickness = []
    for (za, ra), (zb, rb) in zip(profile[:-1], profile[1:]):
        # Inner radius in the band's middle half, above the floor's own surface.
        mid = (z > za + step / 4) & (z < zb - step / 4)
        horizontal = (ra + rb) / 2 - float(r[mid].min())
        tilt = math.atan2(rb - ra, zb - za)
        thickness.append(max(horizontal * math.cos(tilt), min_thickness))
    # The floor disc takes the radius the object stands on. Where that foot is narrower
    # than the wall at the floor's top (a plate's), one more band covers its flank.
    foot = outer(bottom)
    if floor_top - bottom > 0.002 and profile[0][1] - foot > 0.001:
        profile.insert(0, (float(bottom), foot))
        thickness.insert(0, thickness[0])
    return revolved_shell(profile, thickness, (float(bottom), floor_top, foot), segments)


def tapered_open_box(
    bottom_inner: tuple[float, float],
    top_inner: tuple[float, float],
    height: float,
    wall: tuple[float, float],
    floor: float,
    floor_extra: dict[str, str] | None = None,
) -> list[GeomSpec]:
    """A floor and four walls resting on z = 0, centred on the body's z axis, whose inner
    faces run from the half-extents ``bottom_inner`` (x, y) at the floor's top to
    ``top_inner`` at ``height``. ``wall`` is the (x-wall, y-wall) thickness. Walls lean
    outward when the top is wider, as moulded totes and trays do."""
    geoms = []
    for ax in (0, 1):
        lo, hi, t = bottom_inner[ax], top_inner[ax], wall[ax]
        rise = height - floor
        length = math.hypot(rise, hi - lo)
        tilt = math.atan2(hi - lo, rise)
        # Centre: the inner face's midpoint, moved half a thickness outward along the normal.
        centre = (lo + hi) / 2 + t / 2 * math.cos(tilt)
        zc = floor + rise / 2 - t / 2 * math.sin(tilt)
        # The x walls (normal along x) span the y extent between the y walls' outer faces.
        span = max(bottom_inner[1 - ax], top_inner[1 - ax]) + (wall[1] if ax == 0 else 0.0)
        for side in (1, -1):
            pos = [0.0, 0.0, zc]
            pos[ax] = side * centre
            size = [0.0, 0.0, length / 2]
            size[ax], size[1 - ax] = t / 2, span
            # Lean outward: rotate about the in-plane axis perpendicular to the normal.
            axis = [0.0, 0.0, 0.0]
            axis[1 - ax] = 1.0
            angle = side * tilt * (1 if ax == 0 else -1)
            geoms.append(GeomSpec("box", size, pos, list(_axis_quat(axis, angle))))
    bottom_x, bottom_y = bottom_inner[0] + wall[0], bottom_inner[1] + wall[1]
    geoms.insert(0, GeomSpec("box", [bottom_x, bottom_y, floor / 2], [0.0, 0.0, floor / 2], extra=floor_extra or {}))
    return geoms


def open_box(outer: tuple[float, float, float], wall: float, floor: float) -> list[GeomSpec]:
    """A floor and four vertical walls: an open box of ``outer`` size resting on z = 0."""
    inner = (outer[0] / 2 - wall, outer[1] / 2 - wall)
    return tapered_open_box(inner, inner, outer[2], (wall, wall), floor)


def geoms_xml(name: str, geoms: list[GeomSpec], mass: float, friction=(0.6, 0.005, 0.0001), indent: int = 6) -> str:
    """MJCF for collision ``geoms`` sharing ``mass`` in proportion to volume, following the
    package conventions (group 3, condim 4, mass on the collision geoms). A lone geom takes
    all of it, which lets a mesh geom (``hull_of``) carry the mass without a volume."""
    shares = [1.0] if len(geoms) == 1 else [g.volume for g in geoms]
    total = sum(shares)
    pad = " " * indent
    lines = []
    for i, (g, share) in enumerate(zip(geoms, shares)):
        attrs = [
            f'name="{name}_collision_{i}"' if len(geoms) > 1 else f'name="{name}_collision"',
            f'type="{g.type}"',
        ]
        if g.type != "mesh":
            attrs.append(f'size="{_fmt(g.size[: {"box": 3, "cylinder": 2, "capsule": 2, "sphere": 1}[g.type]])}"')
        attrs.append(f'pos="{_fmt(g.pos)}"')
        if not np.allclose(g.quat, [1, 0, 0, 0]):
            attrs.append(f'quat="{_fmt(g.quat)}"')
        attrs += [f'{k}="{v}"' for k, v in g.extra.items()]
        attrs += [
            f'mass="{mass * share / total:.6g}"',
            f'friction="{_fmt(friction)}"',
            f'group="{COLLISION_GROUP}" condim="4" contype="1" conaffinity="1"',
        ]
        lines.append(f"{pad}<geom {' '.join(attrs)}/>")
    return "\n".join(lines)


def write_model(
    obj_dir: Path,
    name: str,
    visual_mesh: str,
    geoms: list[GeomSpec],
    mass: float,
    texture: str | None = None,
    rgba: tuple[float, float, float, float] = (0.7, 0.7, 0.7, 1.0),
    body_pos: tuple[float, float, float] = (0.0, 0.0, 0.0),
    friction=(0.6, 0.005, 0.0001),
) -> Path:
    """Write ``<obj_dir>/<name>.xml``: a free body at ``body_pos`` with a massless visual
    mesh (group 2) and the given collision geoms carrying ``mass``. Returns its path."""
    if texture:
        material = (
            f'    <texture name="{name}_tex" type="2d" file="{texture}"/>\n'
            f'    <material name="{name}_mat" texture="{name}_tex" specular="0.5" shininess="0.5"/>\n'
        )
    else:
        material = f'    <material name="{name}_mat" rgba="{_fmt(rgba)}" specular="0.5" shininess="0.5"/>\n'
    xml = (
        f'<mujoco model="{name}">\n'
        '  <compiler angle="radian"/>\n\n'
        "  <asset>\n"
        f"{material}"
        f'    <mesh name="{name}_visual" file="{visual_mesh}"/>\n'
        "  </asset>\n\n"
        "  <worldbody>\n"
        f'    <body name="{name}" pos="{_fmt(body_pos)}">\n'
        f'      <freejoint name="{name}_joint"/>\n'
        f"{geoms_xml(name, geoms, mass, friction)}\n"
        f'      <geom name="{name}_visual" type="mesh" mesh="{name}_visual" material="{name}_mat"\n'
        f'            group="{VISUAL_GROUP}" mass="0" contype="0" conaffinity="0"/>\n'
        "    </body>\n"
        "  </worldbody>\n"
        "</mujoco>\n"
    )
    path = Path(obj_dir) / f"{name}.xml"
    _write_atomic(path, xml)
    return path


def replace_collision_geoms(
    xml_path: Path,
    name: str,
    geoms: list[GeomSpec],
    mass: float,
    friction=(0.6, 0.005, 0.0001),
    body_pos: tuple[float, float, float] | None = None,
    comment: str | None = None,
) -> None:
    """Rewrite an existing model's collision geoms (and optionally its body's ``pos``),
    leaving its assets and visual geom untouched. The new geoms go where the first old one
    was; ``comment`` replaces the XML comment directly above it, or is added there."""
    text = Path(xml_path).read_text()
    # Collision is decided from each geom's own attributes; a class could set them.
    if re.search(r"<default\b|\bclass=", text):
        raise ValueError(f"{xml_path}: models with <default> classes are not supported; edit the geoms by hand")
    blocks = [b for b in re.finditer(r"[ \t]*<geom\b[^>]*?/>\n?", text, re.S) if _is_collision_xml(b.group(0))]
    if not blocks:
        raise ValueError(f"{xml_path}: no collision geoms found")
    indent = len(blocks[0].group(0)) - len(blocks[0].group(0).lstrip(" "))
    new = geoms_xml(name, geoms, mass, friction, indent) + "\n"
    if comment is not None:
        new = " " * indent + f"<!-- {comment} -->\n" + new
    for b in reversed(blocks):
        start = b.start()
        # Comments directly above a removed geom (blank lines between allowed) described
        # it; drop them too, except the first geom's when no replacement comment is given.
        above = re.search(r"(?:[ \t]*<!--[^\n]*?-->\n(?:[ \t]*\n)*)+\Z", text[:start])
        if above and (b is not blocks[0] or comment is not None):
            start = above.start()
        text = text[:start] + (new if b is blocks[0] else "") + text[b.end() :]
    text = re.sub(r"\n(?:[ \t]*\n){2,}", "\n\n", text)
    if body_pos is not None:
        text, n = re.subn(
            rf'(<body\b[^>]*\bname="{re.escape(name)}"[^>]*?\bpos=")[^"]*(")',
            rf"\g<1>{_fmt(body_pos)}\g<2>",
            text,
            count=1,
        )
        if not n:
            raise ValueError(f'{xml_path}: no <body name="{name}" pos="..."> to move')
    _write_atomic(Path(xml_path), text)


def _is_collision_xml(geom: str) -> bool:
    attrs = dict(re.findall(r'(\w+)="([^"]*)"', geom))
    return attrs.get("contype", "1") != "0" or attrs.get("conaffinity", "1") != "0"


def write_meta(obj_dir: Path, name: str, **fields) -> Path:
    """Write ``<obj_dir>/meta.yaml`` with ``name``, ``mujoco.xml_path`` and ``fields``
    (description, category, mass, dimensions, geometric_properties, perception, ...)."""
    meta = {"name": name, **fields}
    meta.setdefault("mujoco", {})["xml_path"] = f"{name}.xml"
    path = Path(obj_dir) / "meta.yaml"
    _write_atomic(path, yaml.safe_dump(meta, sort_keys=False))
    return path


def _write_atomic(path: Path, text: str) -> None:
    """Replace ``path`` with ``text``, leaving the old file and no temporary on failure."""
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".part")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(text)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


# -------------------------------------------------------------------------- utils


def _pose(pos, xmat) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(xmat).reshape(3, 3)
    T[:3, 3] = pos
    return T


def _axis_quat(axis, angle: float) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.asarray(axis, dtype=float), angle)
    return q


def _fmt(values) -> str:
    # Rounded to 10 micrometres (and 1e-5 in quaternions), dropping sampling noise.
    return " ".join(f"{round(float(v), 5) + 0.0:.6g}" for v in values)


def _mm(value: float) -> float:
    return round(float(value), 2)
