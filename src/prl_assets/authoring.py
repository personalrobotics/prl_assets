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
from scipy.spatial import ConvexHull

from prl_assets import OBJECTS_DIR
from prl_assets.grasp_regions import parse as parse_grasp_regions
from prl_assets.placement import SHAPES, Opening, Placement, Rest, Support
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
# compiled model at the model file's pose, which rests the object on z = 0. ``check``
# compares what an object declares with these measurements; ``measure_placement`` and
# ``placement_yaml`` write them, so nothing in the section is typed by hand.

_CORNERS = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
_RIM_BELOW = 0.003  # opening rays run this far below the rim, inside the walls
_RAYS = 64  # directions sampled for a circular opening's inscribed radius
_ROUND = 5  # metres to 10 micrometres; rotations to 1e-5


def _subtree_geoms(obj: Compiled) -> list[int]:
    m = obj.model
    return [g for g in range(m.ngeom) if m.body_rootid[m.geom_bodyid[g]] == obj.body]


def _reach(obj: Compiled, g: int, origin: np.ndarray) -> float:
    """How far geom ``g`` reaches horizontally from ``origin``: a mesh by its vertices and
    a box by its corners, exactly; a sphere by its centre and radius, and a capsule or
    cylinder by its two end centres and radius, at any orientation (exact for a capsule
    and an upright cylinder, a tight bound for a tilted one). Anything else by the
    corners of its bounding box."""
    m, d = obj.model, obj.data
    rotation = d.geom_xmat[g].reshape(3, 3)
    centre = d.geom_xpos[g]
    kind = int(m.geom_type[g])
    geom = mujoco.mjtGeom
    radius = float(m.geom_size[g][0])
    if kind == int(geom.mjGEOM_SPHERE):
        ends = centre[None, :]
    elif kind in (int(geom.mjGEOM_CYLINDER), int(geom.mjGEOM_CAPSULE)):
        half = float(m.geom_size[g][1]) * rotation[:, 2]
        ends = np.stack([centre - half, centre + half])
    else:
        if kind == int(geom.mjGEOM_MESH):
            mesh = m.geom_dataid[g]
            start, count = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
            local = m.mesh_vert[start : start + count]
        elif kind == int(geom.mjGEOM_BOX):
            local = _CORNERS * m.geom_size[g]
        else:
            local = m.geom_aabb[g][:3] + _CORNERS * m.geom_aabb[g][3:]
        world = local @ rotation.T + centre
        return float(np.hypot(*(world[:, :2] - origin).T).max())
    return float(np.hypot(*(ends[:, :2] - origin).T).max()) + radius


def _asset_pose(obj: Compiled) -> np.ndarray:
    return _pose(obj.data.xpos[obj.body], obj.data.xmat[obj.body])


def _collision_vertices(obj: Compiled) -> np.ndarray:
    return np.vstack([geom_mesh(obj, g).vertices for g in collision_geoms(obj)])


def _collision_ray(obj: Compiled, start, direction) -> float:
    """Distance to the first collision geom hit, or NaN."""
    groups = np.zeros(6, dtype=np.uint8)
    for g in collision_geoms(obj):
        groups[obj.model.geom_group[g]] = 1
    hit = np.zeros(1, dtype=np.int32)
    start, direction = np.asarray(start, dtype=float), np.asarray(direction, dtype=float)
    dist = mujoco.mj_ray(obj.model, obj.data, start, direction, groups, 1, -1, hit)
    return dist if dist >= 0 else math.nan


def _in_asset(obj: Compiled, T_world: np.ndarray) -> list[list[float]]:
    T = np.linalg.inv(_asset_pose(obj)) @ T_world
    return [[round(float(v), _ROUND) + 0.0 for v in row] for row in T]


def measure_rest(obj: Compiled, name: str = "upright") -> Rest:
    """The rest the model file's pose gives ``obj``: its orientation there, its asset
    origin's height above the collision geometry's lowest point, and the horizontal
    reach of every geom of the object about the asset origin's vertical."""
    T = _asset_pose(obj)
    origin = T[:2, 3].copy()
    T[:2, 3] = 0.0
    T[2, 3] -= float(_collision_vertices(obj)[:, 2].min())
    reach = max(_reach(obj, g, origin) for g in _subtree_geoms(obj))
    rounded = [[round(float(v), _ROUND) + 0.0 for v in row] for row in T]
    return Rest(name, rounded, round(reach, _ROUND))


def measure_opening(obj: Compiled, shape: str, name: str = "top") -> Opening:
    """``obj``'s top opening, from rays cast in its collision geometry: the rim at the top
    of the geometry, the interior centred between opposing walls just below the rim, and
    the floor straight below that centre. A rectangle's size is the wall-to-wall width
    along x and y; a circle's radius is the shortest of ``_RAYS`` rays to the wall, the
    largest circle that fits. A rectangle whose corner rays end short of its corners does
    not describe the opening, and raises ``ValueError``."""
    if shape not in SHAPES:
        raise ValueError(f"{obj.name}: opening shape must be one of {tuple(SHAPES)}, not {shape!r}")
    rim = float(_collision_vertices(obj)[:, 2].max())
    z = rim - _RIM_BELOW
    centre = obj.data.xpos[obj.body][:2].copy()

    def wall(direction) -> float:
        dist = _collision_ray(obj, [*centre, z], [*direction, 0.0])
        if math.isnan(dist):
            raise ValueError(f"{obj.name}: no wall around the opening along {direction}")
        return dist

    for _ in range(3):  # opposing walls move the centre to the interior's middle
        centre += [(wall((1, 0)) - wall((-1, 0))) / 2, (wall((0, 1)) - wall((0, -1))) / 2]
    half = [(wall((1, 0)) + wall((-1, 0))) / 2, (wall((0, 1)) + wall((0, -1))) / 2]
    depth = _collision_ray(obj, [*centre, rim], [0.0, 0.0, -1.0])
    if math.isnan(depth):
        raise ValueError(f"{obj.name}: no floor below the opening")
    if shape == "rectangle":
        size = [round(2 * h, _ROUND) for h in half]
        for sx in (-1, 1):
            for sy in (-1, 1):
                angle = math.atan2(sy * half[1], sx * half[0])
                if wall((math.cos(angle), math.sin(angle))) < math.hypot(*half) - SIZE_TOLERANCE_MM / 1e3:
                    raise ValueError(f"{obj.name}: the opening has no corners where a rectangle has them")
    else:
        angles = np.linspace(0, 2 * math.pi, _RAYS, endpoint=False)
        size = round(min(wall((math.cos(a), math.sin(a))) for a in angles), _ROUND)
    frame = np.eye(4)
    frame[:3, 3] = [*centre, rim]
    return Opening(name, shape, size, round(float(depth), _ROUND), _in_asset(obj, frame))


def measure_support(obj: Compiled, name: str = "top") -> Support:
    """``obj``'s top surface as a rectangle: the height of its collision geometry's
    highest point, and the x-y bounds of the collision geoms that reach it."""
    tops = {g: geom_mesh(obj, g).vertices for g in collision_geoms(obj)}
    height = max(v[:, 2].max() for v in tops.values())
    surface = np.vstack([v for v in tops.values() if v[:, 2].max() > height - 1e-3])
    low, high = surface[:, :2].min(axis=0), surface[:, :2].max(axis=0)
    frame = np.eye(4)
    frame[:3, 3] = [*((low + high) / 2), height]
    size = [round(float(e), _ROUND) for e in high - low]
    return Support(name, "rectangle", size, _in_asset(obj, frame))


def measure_placement(
    obj: Compiled, openings: dict[str, str] | None = None, supports: tuple[str, ...] = ()
) -> Placement:
    """``obj``'s canonical rest, and the openings (``{name: shape}``) and supports
    (names) asked for, each measured."""
    return Placement(
        rests=(measure_rest(obj),),
        openings=tuple(measure_opening(obj, shape, name) for name, shape in (openings or {}).items()),
        supports=tuple(measure_support(obj, name) for name in supports),
    )


def placement_yaml(placement: Placement) -> str:
    """``placement`` as a ``meta.yaml`` section, in the layout of the catalog's files."""

    def n(value: float) -> str:  # fixed point: YAML reads "1e-05" as a string
        text = f"{value:.{_ROUND}f}".rstrip("0").rstrip(".")
        return "0" if text in ("", "-0") else text

    def matrix(T) -> str:
        return "[" + ", ".join("[" + ", ".join(n(v) for v in row) + "]" for row in T) + "]"

    def outline(entry) -> str:
        if entry.shape == "rectangle":
            return f"    size: [{n(entry.size[0])}, {n(entry.size[1])}]\n"
        return f"    radius: {n(entry.size)}\n"

    lines = "placement:\n  version: 1\n  rests:\n"
    for rest in placement.rests:
        lines += (
            f"  - name: {rest.name}\n"
            f"    T_support_asset: {matrix(rest.T_support_asset)}\n"
            f"    footprint_radius: {n(rest.footprint_radius)}\n"
        )
    if placement.openings:
        lines += "  openings:\n"
        for opening in placement.openings:
            lines += f"  - name: {opening.name}\n    shape: {opening.shape}\n" + outline(opening)
            lines += f"    depth: {n(opening.depth)}\n    T_asset_opening: {matrix(opening.T_asset_opening)}\n"
    if placement.supports:
        lines += "  supports:\n"
        for support in placement.supports:
            lines += f"  - name: {support.name}\n    shape: {support.shape}\n" + outline(support)
            lines += f"    T_asset_support: {matrix(support.T_asset_support)}\n"
    return lines


def write_placement(obj: Compiled, placement: Placement) -> Path:
    """Write ``placement`` into ``obj``'s ``meta.yaml``: replace its ``placement``
    section, or append one, leaving every other line as it was, and refuse a result that
    would not read back as ``placement``."""
    path = obj.dir / "meta.yaml"
    lines = path.read_text().splitlines(keepends=True)
    start = next((i for i, line in enumerate(lines) if line.startswith("placement:")), None)
    if start is None:
        text = "".join(lines).rstrip("\n") + "\n\n" + placement_yaml(placement)
    else:
        end = next((i for i in range(start + 1, len(lines)) if re.match(r"[^\s#]", lines[i])), len(lines))
        while end > start + 1 and not lines[end - 1].strip():
            end -= 1
        text = "".join(lines[:start]) + placement_yaml(placement) + "".join(lines[end:])
    if parse_placement(yaml.safe_load(text)) != placement:
        raise ValueError(f"{obj.name}: the written placement section would not read back as measured")
    _write_atomic(path, text)
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

    if len(declared.rests) > 1:
        failures.append("placement.rests: only the canonical rest is measured; declare one")
    rest, measured = declared.canonical, measure_rest(obj)
    frames(f"placement.rests.{rest.name}.T_support_asset", rest.T_support_asset, measured.T_support_asset)
    sizes(f"placement.rests.{rest.name}.footprint_radius", rest.footprint_radius, measured.footprint_radius)
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
        if support.shape != found.shape:
            failures.append(f"{field}.shape: a top surface is measured as a {found.shape}")
            continue
        sizes(f"{field}.size", support.size, found.size)
        frames(f"{field}.T_asset_support", support.T_asset_support, found.T_asset_support)
    return failures


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
            low, high = vertices.min(axis=0) - 0.01, vertices.max(axis=0) + 0.01
            for region in regions:
                bounds = region.bounds_in_asset()
                if np.any(bounds[0] < low) or np.any(bounds[1] > high):
                    failures.append(f"grasp_regions.{region.name}.bounds: outside visual bounds plus 10 mm")
    if placement is None:
        failures.append("no placement section: write it with scripts/author_placement.py")
    else:
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
