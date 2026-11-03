# SPDX-License-Identifier: MIT
"""Inspect the installed YCB-V collection, its source projections and a seeded scene.

Requires prl_assets[ycbv] and asset_manager. Run with MUJOCO_GL=egl on a headless
Linux machine. Writes measurements and rendered evidence, including on failure.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import shutil
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import asdict
from pathlib import Path

import mujoco
import numpy as np
import trimesh
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.spatial.transform import Rotation

from prl_assets import OBJECTS_DIR, YCBV_OBJECTS_DIR, authoring
from prl_assets.ycbv import BOP_YCB_NAMES
from prl_assets.ycbv_source import sha256, verify_archive

# Silhouettes are rendered at 320x240. The four retained remeshings may differ
# slightly at edges. Texture/frame agreement is checked separately by the audit.
MIN_PROJECTION_IOU = 0.97
MAX_FLOOR_PENETRATION_M = 0.001


def _quat(rotation):
    x, y, z, w = Rotation.from_matrix(rotation).as_quat()
    return [w, x, y, z]


def _scene():
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10
    spec.worldbody.add_light(pos=[0, -1, 2], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
    return spec


def _attach(spec, directory, prefix):
    meta = yaml.safe_load((directory / "meta.yaml").read_text())
    source = mujoco.MjSpec.from_file(str(directory / meta["mujoco"]["xml_path"]))
    # MjSpec retains resource bytes while composing, but to_xml does not retain
    # the child's original directory. Make saved inspection scenes reloadable.
    for resource in [*source.meshes, *source.textures]:
        if resource.file:
            path = (directory / resource.file).resolve()
            if not path.is_relative_to(directory.resolve()):
                raise ValueError(f"resource leaves asset directory: {path}")
            resource.file = str(path)
    top = next(iter(source.worldbody.bodies))
    return spec.worldbody.add_frame().attach_body(top, prefix, "")


def _render(model, data, *, distance, lookat, azimuth=135, elevation=-25, collision=False, axes=False, foreground=True):
    camera = mujoco.MjvCamera()
    camera.lookat = lookat
    camera.distance = distance
    camera.azimuth = azimuth
    camera.elevation = elevation
    option = mujoco.MjvOption()
    option.geomgroup[:] = 0
    option.geomgroup[3 if collision else 2] = 1
    if axes:
        option.frame = mujoco.mjtFrame.mjFRAME_BODY
    with mujoco.Renderer(model, height=240, width=320) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=option)
        rgb = renderer.render().copy()
        mask = None
        if foreground:
            # We need a binary foreground mask, not per-part identities. Depth
            # avoids blended colour IDs at boundaries of many collision parts.
            renderer.enable_depth_rendering()
            far = model.stat.extent * model.vis.map.zfar
            mask = renderer.render().copy() < 0.99 * far
    return rgb, mask


def _projection(directory, archive, temporary, out):
    meta = yaml.safe_load((directory / "meta.yaml").read_text())
    identity = meta["dataset"]["ycbv"]
    member = f"models/obj_{identity['bop_id']:06d}.ply"
    ply = temporary / Path(member).name
    ply.write_bytes(archive.read(member))
    # Trimesh loads the PLY's sibling texture, so keep that reference local too.
    png = f"models/obj_{identity['bop_id']:06d}.png"
    (temporary / Path(png).name).write_bytes(archive.read(png))
    source = trimesh.load(ply, process=False)
    source.apply_scale(0.001)
    # MuJoCo caches assets by filename and modification time. Give each
    # reference a unique path, including the deliberately modified control.
    source_obj = temporary / f"{directory.name}-source.obj"
    source_obj.write_text(trimesh.exchange.obj.export_obj(source, include_texture=False, include_normals=False))
    reference = _scene()
    reference.add_mesh(name="source", file=str(source_obj))
    reference.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="source", group=2, rgba=[0.8, 0.8, 0.8, 1])
    reference_model = reference.compile()
    reference_data = mujoco.MjData(reference_model)
    mujoco.mj_forward(reference_model, reference_data)
    ours = _scene()
    body = _attach(ours, directory, "asset/")
    # World coordinates equal the BOP metre frame. Ignore the XML's resting
    # placement and apply the actual inverse asset/BOP transform to the body.
    transform = np.linalg.inv(np.asarray(meta["frames"]["T_asset_bop"], dtype=float))
    body.pos = transform[:3, 3]
    body.quat = _quat(transform[:3, :3])
    model = ours.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    centre = source.bounds.mean(axis=0)
    distance = float(max(source.extents) * 3)
    rows = []
    for view, (azimuth, elevation) in enumerate(((135, -25), (0, 0), (90, 0), (90, -89))):
        args = dict(distance=distance, lookat=centre, azimuth=azimuth, elevation=elevation)
        _, expected = _render(reference_model, reference_data, **args)
        _, observed = _render(model, data, **args)
        _, collision_mask = _render(model, data, **args, collision=True)
        holes = ndimage.binary_fill_holes(expected) & ~expected
        holes = ndimage.binary_erosion(holes, iterations=2)
        count = int(holes.sum())
        occupied = float(np.count_nonzero(holes & collision_mask) / count) if count >= 25 else None
        union = int(np.count_nonzero(expected | observed))
        iou = float(np.count_nonzero(expected & observed) / union) if union else 0.0
        overlay = np.zeros((240, 320, 3), dtype=np.uint8)
        overlay[expected] = [255, 80, 80]
        overlay[observed] += np.array([0, 100, 100], dtype=np.uint8)
        Image.fromarray(overlay).save(out / f"{directory.name}-projection-{view}.png")
        rows.append(
            dict(
                azimuth_deg=azimuth,
                elevation_deg=elevation,
                silhouette_iou=iou,
                image_size_px=[320, 240],
                fovy_deg=float(model.vis.global_.fovy),
                lookat_bop_m=centre.tolist(),
                camera_distance_m=distance,
                source_aperture_pixels=count,
                collision_aperture_occupied_fraction=occupied,
            )
        )
    return rows


def _inspect(directory, out):
    obj = authoring.load(directory)
    fit = authoring.check(obj)
    collision = authoring.collision_geoms(obj)
    mass = float(obj.model.body_mass[obj.body])
    inertia = obj.model.body_inertia[obj.body]
    friction = obj.model.geom_friction[collision]
    physical_valid = bool(
        mass > 0
        and np.isfinite(mass)
        and np.isfinite(inertia).all()
        and (inertia > 0).all()
        and np.isfinite(friction).all()
        and (friction >= 0).all()
        and obj.model.ntex > 0
    )
    # The world floor is at z=0. These exact signed distances also report the
    # resting contact that a visual/collision overlay cannot quantify.
    spec = obj.spec.copy()
    spec.worldbody.add_geom(
        name="measurement_floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1], contype=0, conaffinity=0, group=5
    )
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    plane = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "measurement_floor")
    solid_ids = [g for g in range(model.ngeom) if model.geom_contype[g] or model.geom_conaffinity[g]]
    clearance = min(mujoco.mj_geomDistance(model, data, plane, g, 1.0, None) for g in solid_ids)
    points = authoring.visual_points(obj)
    centre = (points.min(0) + points.max(0)) / 2
    distance = float(max(np.ptp(points, axis=0)) * 3)
    panels = []
    for azimuth, elevation in ((135, -25), (90, -80)):
        rgb, _ = _render(
            model, data, distance=distance, lookat=centre, azimuth=azimuth, elevation=elevation, foreground=False
        )
        image, _ = _render(
            model,
            data,
            distance=distance,
            lookat=centre,
            azimuth=azimuth,
            elevation=elevation,
            collision=True,
            axes=True,
            foreground=False,
        )
        # Overlay the collision silhouette/axes on the textured view; save the
        # separate geometry image too, since alpha overlays can hide cavities.
        Image.fromarray(image).save(out / f"{obj.name}-collision-{azimuth}-{elevation}.png")
        panels.extend([Image.fromarray(rgb), Image.blend(Image.fromarray(rgb), Image.fromarray(image), 0.5)])
    tile = Image.new("RGB", (640, 510), "#202020")
    for index, image in enumerate(panels):
        tile.paste(image, ((index % 2) * 320, (index // 2) * 240))
    draw = ImageDraw.Draw(tile)
    dims = " x ".join(f"{x * 1000:.1f}" for x in obj.meta["dimensions"])
    draw.text((8, 482), f"{obj.name}: {dims} mm; floor {clearance * 1000:.3f} mm", fill="white")
    tile.save(out / f"{obj.name}-inspection.png")
    return dict(
        name=obj.name,
        collision_geoms=len(collision),
        compiled_mass_kg=mass,
        principal_inertia_kg_m2=inertia.tolist(),
        physical_parameters_valid=physical_valid,
        dimensions_m=obj.meta["dimensions"],
        resting_floor_clearance_m=float(clearance),
        collision_check=asdict(fit),
    ), tile


def _filled_handle_control(archive, temporary, out):
    """A deliberately filled mug handle must fail the rendered aperture check."""
    directory = temporary / "negative-control-mug"
    shutil.copytree(YCBV_OBJECTS_DIR / "mug", directory)
    meta = yaml.safe_load((directory / "meta.yaml").read_text())
    tree = ET.parse(directory / meta["mujoco"]["xml_path"])
    body = tree.find("worldbody/body")
    for geom in list(body.findall("geom")):
        if geom.get("group") == "3":
            body.remove(geom)
    visual = trimesh.load(directory / "mug_visual.obj", process=False, force="mesh")
    (directory / "filled.obj").write_text(trimesh.exchange.obj.export_obj(visual.convex_hull, include_texture=False))
    ET.SubElement(tree.find("asset"), "mesh", name="filled_control", file="filled.obj")
    ET.SubElement(body, "geom", type="mesh", mesh="filled_control", group="3", mass="0.118")
    tree.write(directory / meta["mujoco"]["xml_path"])
    projections = _projection(directory, archive, temporary, out)
    detected = all(p["silhouette_iou"] >= MIN_PROJECTION_IOU for p in projections) and any(
        p["source_aperture_pixels"] >= 25 and p["collision_aperture_occupied_fraction"] > 0.9 for p in projections
    )
    return dict(
        deliberate_failure="mug handle replaced by a solid convex hull", projections=projections, detected=detected
    )


def _mixed_scene(out):
    spec = _scene()
    spec.visual.global_.offwidth = 1000
    spec.visual.global_.offheight = 600
    spec.worldbody.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1], group=0)
    rng = np.random.default_rng(28)
    names = ["cracker_box", "mustard_bottle", "mug", "large_clamp"] * 2
    bodies = []
    for index, name in enumerate(names):
        body = _attach(spec, YCBV_OBJECTS_DIR / name, f"{index}/")
        body.pos = [(index % 4 - 1.5) * 0.35, (index // 4 - 0.5) * 0.4, body.pos[2] + 0.08]
        yaw = rng.uniform(-np.pi, np.pi)
        body.quat = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        bodies.append(body.name)
    model = spec.compile()
    data = mujoco.MjData(model)
    for _ in range(2000):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table")
    rows = []
    for name in bodies:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        solids = [
            g
            for g in range(model.ngeom)
            if model.geom_bodyid[g] == bid and (model.geom_contype[g] or model.geom_conaffinity[g])
        ]
        clearance = min(mujoco.mj_geomDistance(model, data, floor, g, 2.0, None) for g in solids)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, bid, velocity, 0)
        rows.append(
            dict(
                instance=name,
                position_m=data.xpos[bid].tolist(),
                quaternion_wxyz=data.xquat[bid].tolist(),
                floor_clearance_m=float(clearance),
                linear_speed_m_s=float(np.linalg.norm(velocity[3:])),
            )
        )
    # Bundle only the referenced scene resources, so the inspection XML can
    # be reopened after the temporary verification installation is removed.
    tree = ET.fromstring(spec.to_xml())
    for element in tree.iter():
        if element.get("file"):
            source = Path(element.get("file")).resolve()
            if not source.is_relative_to(OBJECTS_DIR.resolve()):
                raise ValueError(f"scene resource leaves installed assets: {source}")
            relative = Path("scene-assets") / source.relative_to(OBJECTS_DIR.resolve())
            destination = out / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
            element.set("file", relative.as_posix())
    for filename in ("NOTICE.md", "LICENSE-CC-BY-4.0.txt", "LICENSE-YCB-VIDEO-MIT.txt"):
        shutil.copyfile(YCBV_OBJECTS_DIR / filename, out / "scene-assets" / filename)
    ET.indent(tree)
    authoring._write_atomic(out / "mixed-scene.xml", ET.tostring(tree, encoding="unicode") + "\n")
    np.savez(out / "mixed-scene-state.npz", qpos=data.qpos, qvel=data.qvel)
    restored_model = mujoco.MjModel.from_xml_path(str(out / "mixed-scene.xml"))
    restored = mujoco.MjData(restored_model)
    restored.qpos[:] = data.qpos
    restored.qvel[:] = data.qvel
    mujoco.mj_forward(restored_model, restored)
    if not np.allclose(restored.xpos, data.xpos, atol=1e-9):
        raise ValueError("saved scene and state do not reproduce the inspected placement")
    option = mujoco.MjvOption()
    option.geomgroup[:] = 0
    option.geomgroup[0] = option.geomgroup[2] = 1
    camera = mujoco.MjvCamera()
    camera.lookat = [0, 0, 0.08]
    camera.distance, camera.azimuth, camera.elevation = 1.9, 110, -45
    with mujoco.Renderer(model, height=600, width=1000) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=option)
        Image.fromarray(renderer.render()).save(out / "mixed-scene.png")
    ok = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()) and all(
        -MAX_FLOOR_PENETRATION_M <= row["floor_clearance_m"] <= 0.005 and row["linear_speed_m_s"] < 0.05 for row in rows
    )
    return dict(
        seed=28,
        timestep_s=0.002,
        steps=2000,
        integrator="implicitfast",
        cone="elliptic",
        impratio=10,
        floor_penetration_limit_m=MAX_FLOOR_PENETRATION_M,
        instances=rows,
        passed=ok,
    )


def verify(archive_path, out):
    from asset_manager import AssetManager

    from prl_assets.ycbv_audit import audit, write_json

    out.mkdir(parents=True, exist_ok=False)
    collection = AssetManager(YCBV_OBJECTS_DIR)
    catalog = AssetManager(OBJECTS_DIR)
    ids = [collection.get(name)["dataset"]["ycbv"]["bop_id"] for name in collection.list()]
    if sorted(ids) != sorted(BOP_YCB_NAMES):
        raise ValueError("installed collection must contain each of the 21 BOP identities once")
    if not set(collection.list()) <= set(catalog.list()) or catalog.resolve_alias("bowl") != "plastic_bowl":
        raise ValueError("full-catalog discovery or existing bowl identity changed")
    rows, tiles = [], []
    with zipfile.ZipFile(verify_archive(archive_path)) as archive, tempfile.TemporaryDirectory() as temporary:
        for name in sorted(collection.list(), key=lambda n: collection.get(n)["dataset"]["ycbv"]["bop_id"]):
            directory = YCBV_OBJECTS_DIR / name
            print(f"Inspecting {name}", flush=True)
            row, tile = _inspect(directory, out)
            row["source_audit"] = audit(archive, Path(temporary), directory)
            row["projections"] = _projection(directory, archive, Path(temporary), out)
            row["files"] = {
                str(p.relative_to(directory)): sha256(p) for p in sorted(directory.rglob("*")) if p.is_file()
            }
            row["passed"] = (
                row["physical_parameters_valid"]
                and row["collision_check"]["failures"] == []
                and row["source_audit"]["pass"]
                and all(
                    p["silhouette_iou"] >= MIN_PROJECTION_IOU
                    and (
                        p["collision_aperture_occupied_fraction"] is None
                        or p["collision_aperture_occupied_fraction"] <= 0.1
                    )
                    for p in row["projections"]
                )
                and abs(row["resting_floor_clearance_m"]) <= 0.001
            )
            write_json(out / f"{name}.json", row)
            rows.append(row)
            tiles.append(tile)
        negative_control = _filled_handle_control(archive, Path(temporary), out)
    sheet = Image.new("RGB", (640 * 3, 510 * 7), "#202020")
    for index, tile in enumerate(tiles):
        sheet.paste(tile, ((index % 3) * 640, (index // 3) * 510))
    sheet.save(out / "contact-sheet.png")
    scene = _mixed_scene(out)
    result = dict(
        environment={
            name: importlib.metadata.version(name)
            for name in ("prl_assets", "asset_manager", "numpy", "mujoco", "trimesh", "scipy", "pillow")
        },
        installed_root=str(OBJECTS_DIR),
        catalog_count=len(catalog.list()),
        collection_count=len(ids),
        minimum_projection_iou=MIN_PROJECTION_IOU,
        foreground_definition="depth < 0.99 * far clipping plane",
        aperture_check=dict(erosion_pixels=2, minimum_eroded_area_pixels=25, maximum_collision_occupied_fraction=0.1),
        objects=rows,
        mixed_scene=scene,
        filled_handle_negative_control=negative_control,
        passed=all(row["passed"] for row in rows) and scene["passed"] and negative_control["detected"],
    )
    write_json(out / "validation.json", result)
    return result["passed"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("artifacts/installed-ycbv"))
    args = parser.parse_args()
    if args.out.resolve().is_relative_to(OBJECTS_DIR.resolve()):
        parser.error("inspection outputs must be outside the installed assets")
    if args.out.exists():
        parser.error("use a new output directory to preserve previous inspection evidence")
    return 0 if verify(args.archive, args.out.resolve()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
