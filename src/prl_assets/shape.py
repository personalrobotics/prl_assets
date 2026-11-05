# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""An object's collision geometry as MuJoCo compiled it, for sstsr's ``StablePlacer.place``.

Requires the ``placement`` extra (``mujoco``, ``sstsr``)::

    from tsr.placement import StablePlacer
    from prl_assets.shape import collision_shape

    shape = collision_shape(model, data, "can_0")
    templates = StablePlacer(table_x=0.3, table_y=0.2).place(shape)

The caller need not know what the object is made of: each collision geom becomes the
part that collides like it, and the centre of mass is the compiled model's.
"""

from __future__ import annotations

import mujoco
import numpy as np
from tsr.placement import Box, Capsule, Cylinder, Hull, Shape, Sphere


def collision_shape(model: mujoco.MjModel, data: mujoco.MjData, body: int | str) -> Shape:
    """``body``'s collision geometry and centre of mass, in ``body``'s frame.

    The collision geoms (those with contype or conaffinity) of ``body`` and every body
    below it are taken at their poses in ``data``, which must be forwarded
    (``mj_forward``, or ``mj_kinematics`` then ``mj_comPos`` for the centre of mass): an
    articulated object is described as it stands. A box, cylinder, sphere or capsule
    becomes the same primitive; a mesh becomes its vertices' convex hull, which is what
    MuJoCo collides. The centre of mass
    is ``data.subtree_com``, so a non-uniform object rests where its mass says.

    Raises ``ValueError`` when ``body`` has no collision geom, one of a type that has no
    part (a plane, ellipsoid, height field or SDF), or a mesh that spans no volume.
    """
    b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body) if isinstance(body, str) else int(body)
    if not 0 < b < model.nbody:
        raise ValueError(f"no body {body!r} in the model")
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or str(b)
    subtree = _subtree(model, b)
    T_world_body = _pose(data.xpos[b], data.xmat[b])
    T_body_world = np.linalg.inv(T_world_body)
    parts = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] not in subtree or not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        parts.append(_part(model, g, T_body_world @ _pose(data.geom_xpos[g], data.geom_xmat[g]), name))
    if not parts:
        raise ValueError(f"body {name!r} has no collision geom")
    com = T_body_world[:3, :3] @ data.subtree_com[b] + T_body_world[:3, 3]
    return Shape(parts, com=com)


def _part(model: mujoco.MjModel, g: int, pose: np.ndarray, body: str):
    t, s = int(model.geom_type[g]), model.geom_size[g]
    G = mujoco.mjtGeom
    if t == G.mjGEOM_BOX:
        return Box(2 * s, pose=pose)
    if t == G.mjGEOM_CYLINDER:
        return Cylinder(s[0], 2 * s[1], pose=pose)
    if t == G.mjGEOM_SPHERE:
        return Sphere(s[0], pose=pose)
    if t == G.mjGEOM_CAPSULE:
        return Capsule(s[0], 2 * s[1], pose=pose)
    geom = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or str(g)
    if t == G.mjGEOM_MESH:
        i = model.geom_dataid[g]
        vertices = model.mesh_vert[model.mesh_vertadr[i] : model.mesh_vertadr[i] + model.mesh_vertnum[i]]
        try:
            return Hull(vertices, pose=pose)
        except ValueError as e:
            raise ValueError(f"body {body!r}: collision mesh geom {geom!r}: {e}") from e
    raise ValueError(f"body {body!r}: collision geom {geom!r} is a {G(t).name}, which has no placement part")


def _subtree(model: mujoco.MjModel, root: int) -> set[int]:
    bodies = {root}
    for b in range(root + 1, model.nbody):  # a body's children follow it
        if model.body_parentid[b] in bodies:
            bodies.add(b)
    return bodies


def _pose(pos, xmat) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(xmat).reshape(3, 3)
    T[:3, 3] = pos
    return T
