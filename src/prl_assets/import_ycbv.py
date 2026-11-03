# SPDX-License-Identifier: MIT
"""Import selected YCB-V assets from the pinned source; retain existing identities.

Requires prl_assets[ycbv]. New assets are checked in invisible staging directories;
meta.yaml is published last so recursive registries see only completed objects.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import math
import os
import shutil
import tempfile
import zipfile
from contextlib import contextmanager
from dataclasses import asdict
from pathlib import Path

from prl_assets.ycbv import BOP_YCB_NAMES
from prl_assets.ycbv_source import ARCHIVE_SHA256, ARCHIVE_URL, fetch_archive, sha256, verify_archive

COACD_VERSION = "1.0.14"
MASS_SOURCE = "https://personalrobotics.cs.washington.edu/publications/calli2015ycb.pdf#page=5"
# Extend only after checking each object's collision fit and open-space probes.
RECIPES = {5: {"mass": 0.603, "material": "plastic", "category": ["food_package", "graspable"]}}
DECOMPOSITION = dict(
    threshold=0.002,
    real_metric=True,
    seed=0,
    preprocess_mode="auto",
    preprocess_resolution=50,
    resolution=2000,
    mcts_nodes=20,
    mcts_iterations=150,
    mcts_max_depth=3,
    merge=True,
    decimate=True,
    max_ch_vertex=128,
)


def canonical_name(bop_id: int) -> str:
    """Stable catalog identities, including the two names that avoid existing ambiguity."""
    if bop_id == 9:
        return "spam_can"
    if bop_id == 13:
        return "ycb_bowl"  # 'bowl' already resolves to the distinct plastic_bowl.
    return BOP_YCB_NAMES[bop_id][4:]


def _json(path: Path, value) -> None:
    text = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=".json-", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(text)
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


@contextmanager
def _lock(root: Path):
    # POSIX authoring platforms; flock releases the lock even after a crash. Keep
    # the inode so concurrent importers cannot accidentally acquire different locks.
    import fcntl

    lock_path = root / ".import.lock"
    if lock_path.is_symlink():
        raise ValueError("import lock must not be a symlink")
    with lock_path.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield


def _existing(root: Path) -> dict[int, Path]:
    import yaml

    result = {}
    names = set()
    for p in sorted(root.rglob("meta.yaml")):
        if not p.resolve().is_relative_to(root):
            raise ValueError(f"metadata leaves output collection: {p}")
        try:
            meta = yaml.safe_load(p.read_text())
        except yaml.YAMLError as error:
            raise ValueError(f"invalid YAML in {p}: {error}") from error
        if not isinstance(meta, dict) or not isinstance(meta.get("name"), str):
            raise ValueError(f"invalid asset metadata: {p}")
        if meta["name"] in names:
            raise ValueError(f"duplicate canonical name {meta['name']}")
        names.add(meta["name"])
        dataset = meta.get("dataset", {})
        if not isinstance(dataset, dict):
            raise ValueError(f"invalid dataset metadata: {p}")
        identity = dataset.get("ycbv")
        if identity is None:
            continue
        if not isinstance(identity, dict):
            raise ValueError(f"invalid YCB-V identity: {p}")
        bop_id = identity.get("bop_id")
        if type(bop_id) is not int or BOP_YCB_NAMES.get(bop_id) != identity.get("ycb_name"):
            raise ValueError(f"invalid YCB-V identity: {p}")
        if bop_id in result:
            raise ValueError(f"duplicate BOP identity {bop_id}: {p}")
        result[bop_id] = p.parent
    return result


def _obj(mesh, path: Path, textured: bool) -> None:
    import trimesh

    text = trimesh.exchange.obj.export_obj(
        mesh,
        include_normals=False,
        include_color=False,
        include_texture=textured,
        write_texture=False,
        digits=9,
        header=None,
    )
    # MuJoCo binds the texture in MJCF. Avoid dangling MTL references from OBJ export.
    text = "\n".join(line for line in text.splitlines() if not line.startswith(("mtllib ", "usemtl "))) + "\n"
    path.write_text(text)


def _build(archive: zipfile.ZipFile, stage: Path, bop_id: int) -> dict:
    import xml.etree.ElementTree as ET

    import coacd
    import mujoco
    import numpy as np
    import trimesh
    import yaml
    from PIL import Image

    from prl_assets import authoring

    if importlib.metadata.version("coacd") != COACD_VERSION:
        raise ValueError(f"reproducible conversion requires coacd=={COACD_VERSION}")
    name = canonical_name(bop_id)
    recipe = RECIPES[bop_id]
    source = stage / ".source"
    source.mkdir()
    members = [f"models/obj_{bop_id:06d}.{ext}" for ext in ("ply", "png")]
    hashes = {}
    for member in members:
        # Only fixed, whitelisted members are read. Never extract archive paths.
        if archive.namelist().count(member) != 1:
            raise ValueError(f"expected one archive member {member}")
        data = archive.read(member)
        destination = source / Path(member).name
        destination.write_bytes(data)
        hashes[member] = sha256(destination)
    model_info = json.loads(archive.read("models/models_info.json"))[str(bop_id)]
    ply = source / f"obj_{bop_id:06d}.ply"
    header = ply.read_bytes().split(b"end_header", 1)[0].decode("ascii")
    textures = [line.split(maxsplit=2)[2] for line in header.splitlines() if line.startswith("comment TextureFile ")]
    if textures != [f"obj_{bop_id:06d}.png"]:
        raise ValueError("source PLY must reference only its expected sibling texture")
    mesh = trimesh.load(ply, process=False)
    if not isinstance(mesh, trimesh.Trimesh) or not np.isfinite(mesh.vertices).all():
        raise ValueError("source must be a finite triangle mesh")
    if mesh.visual.kind != "texture" or not np.isfinite(mesh.visual.uv).all():
        raise ValueError("source must have finite texture coordinates")
    mesh.apply_scale(1e-3)
    _obj(mesh, stage / f"{name}_visual.obj", True)
    with Image.open(source / f"obj_{bop_id:06d}.png") as image:
        image.convert("RGB").resize((1024, 1024), Image.Resampling.LANCZOS).save(stage / f"{name}_texture.png")
    collision_input = trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces, process=True)
    coacd.set_log_level("warn")
    parts = coacd.run_coacd(coacd.Mesh(collision_input.vertices, collision_input.faces), **DECOMPOSITION)
    hulls = [trimesh.Trimesh(vertices=v, faces=f, process=True).convex_hull for v, f in parts]
    if not hulls or any(not math.isfinite(h.volume) or h.volume <= 0 for h in hulls):
        raise ValueError("decomposition must produce finite, positive-volume hulls")
    hulls.sort(key=lambda h: tuple(np.round(h.centroid, 9)))
    total_volume = sum(h.volume for h in hulls)
    vertices = np.vstack([h.vertices for h in hulls])
    bottom = float(vertices[:, 2].min())
    extents = np.ptp(vertices, axis=0).tolist()
    model = ET.Element("mujoco", model=name)
    ET.SubElement(model, "compiler", angle="radian")
    assets = ET.SubElement(model, "asset")
    ET.SubElement(assets, "texture", name="visual_tex", type="2d", file=f"{name}_texture.png")
    ET.SubElement(assets, "material", name="visual_mat", texture="visual_tex")
    ET.SubElement(assets, "mesh", name="visual_mesh", file=f"{name}_visual.obj")
    body = ET.SubElement(ET.SubElement(model, "worldbody"), "body", name=name, pos=f"0 0 {-bottom:.9g}")
    ET.SubElement(body, "freejoint", name=f"{name}_joint")
    ET.SubElement(
        body,
        "geom",
        name="visual",
        type="mesh",
        mesh="visual_mesh",
        material="visual_mat",
        group="2",
        mass="0",
        contype="0",
        conaffinity="0",
    )
    for index, hull in enumerate(hulls):
        filename = f"collision_{index:03d}.obj"
        _obj(hull, stage / filename, False)
        ET.SubElement(assets, "mesh", name=f"collision_{index}", file=filename)
        ET.SubElement(
            body,
            "geom",
            name=f"{name}_collision_{index}",
            type="mesh",
            mesh=f"collision_{index}",
            mass=f"{recipe['mass'] * hull.volume / total_volume:.12g}",
            friction="0.6 0.005 0.0001",
            group="3",
            condim="4",
        )
    ET.indent(model)
    (stage / f"{name}.xml").write_text(ET.tostring(model, encoding="unicode") + "\n")
    meta = dict(
        name=name,
        description=f"YCB-V {BOP_YCB_NAMES[bop_id]}",
        category=recipe["category"],
        mass=recipe["mass"],
        dimensions=extents,
        material=recipe["material"],
        geometric_properties=dict(type="box", size=extents),
        mujoco=dict(xml_path=f"{name}.xml", scale=1.0, friction=[0.6, 0.005, 0.0001]),
        perception=dict(aliases=[BOP_YCB_NAMES[bop_id], "YCB " + name.replace("_", " ")]),
        dataset=dict(ycbv=dict(bop_id=bop_id, ycb_name=BOP_YCB_NAMES[bop_id])),
        frames=dict(T_asset_bop=np.eye(4).tolist()),
        symmetries=dict(
            frame="bop",
            discrete=model_info.get("symmetries_discrete", []),
            continuous=model_info.get("symmetries_continuous", []),
        ),
        provenance=dict(
            geometry=(
                "Pinned BOP PLY converted mm to m without alignment or simplification; texture resized to 1024x1024"
            ),
            reference_model=f"models/obj_{bop_id:06d}.ply in the pinned BOP ycbv_models.zip",
            license="CC-BY-4.0",
            attribution="YCB Object and Model Set; see collection NOTICE.md",
            archive_sha256=ARCHIVE_SHA256,
            source_members=hashes,
            physical_properties=dict(
                mass_source=MASS_SOURCE,
                mass_note="Reference YCB object mass, not a measurement of a lab copy",
                inertia="Estimated by MuJoCo from uniformly dense convex parts; overlaps are counted",
                friction="Unmeasured nominal contact parameters",
            ),
            conversion=dict(
                coacd=COACD_VERSION,
                settings=DECOMPOSITION,
                texture_size=[1024, 1024],
                environment={
                    name: importlib.metadata.version(name) for name in ("numpy", "trimesh", "mujoco", "pillow", "scipy")
                },
            ),
        ),
    )
    spec = mujoco.MjSpec.from_file(str(stage / f"{name}.xml"))
    compiled = authoring._compiled(name, stage, meta, spec)
    report = authoring.check(compiled)
    if not report.ok:
        raise ValueError(f"{name} failed collision validation: {report.failures}")
    shutil.rmtree(source)
    # No discoverable metadata exists until publication. A partial staged import
    # is therefore invisible even if output is inside the catalog being scanned.
    (stage / ".meta.yaml.pending").write_text(yaml.safe_dump(meta, sort_keys=False))
    return dict(
        name=name,
        bop_id=bop_id,
        action="imported",
        hulls=len(hulls),
        fit=asdict(report.fit),
        files={
            ("meta.yaml" if p.name == ".meta.yaml.pending" else p.name): sha256(p)
            for p in sorted(stage.iterdir())
            if p.is_file()
        },
    )


def import_objects(objects: list[str], output: Path, archive: Path) -> dict:
    """Validate all requested assets, then publish complete new directories only.

    Existing identities are read-only. Failed publication removes only directories
    created by this invocation; already-valid assets are never overwritten.
    """
    allowed = {canonical_name(i): i for i in BOP_YCB_NAMES}
    if not objects or len(set(objects)) != len(objects) or any(name not in allowed for name in objects):
        raise ValueError("select distinct canonical YCB-V names (no paths or unknown names)")
    archive = verify_archive(archive)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    with _lock(output), tempfile.TemporaryDirectory(dir=output, prefix=".ycbv-stage-") as temporary:
        existing = _existing(output)
        rows = []
        staged = []
        published = []
        with zipfile.ZipFile(archive) as z:
            for name in objects:
                bop_id = allowed[name]
                if bop_id in existing:
                    directory = existing[bop_id]
                    import xml.etree.ElementTree as ET

                    import yaml

                    from prl_assets import authoring

                    meta = yaml.safe_load((directory / "meta.yaml").read_text())
                    simulator = meta.get("mujoco")
                    if not isinstance(simulator, dict) or not isinstance(simulator.get("xml_path"), str):
                        raise ValueError(f"invalid MuJoCo metadata: {directory}")
                    model_path = (directory / simulator["xml_path"]).resolve()
                    if not model_path.is_relative_to(directory.resolve()):
                        raise ValueError(f"MuJoCo path leaves {directory}")
                    try:
                        tree = ET.parse(model_path)
                    except ET.ParseError as error:
                        raise ValueError(f"invalid MuJoCo XML in {model_path}: {error}") from error
                    if list(tree.iter("include")):
                        raise ValueError("retained YCB-V assets must not have XML includes")
                    for element in tree.iter():
                        if element.tag == "compiler" and any(
                            element.get(k) for k in ("meshdir", "texturedir", "assetdir")
                        ):
                            raise ValueError("retained YCB-V assets must use local resource paths")
                        if element.get("file") and not (directory / element.get("file")).resolve().is_relative_to(
                            directory.resolve()
                        ):
                            raise ValueError(f"resource path leaves {directory}")
                    for resource in directory.rglob("*"):
                        if resource.is_file() and not resource.resolve().is_relative_to(directory.resolve()):
                            raise ValueError(f"linked resource leaves {directory}")
                    report = authoring.check(directory)
                    if not report.ok:
                        raise ValueError(f"existing asset {directory} fails validation: {report.failures}")
                    rows.append(
                        dict(
                            name=directory.name,
                            bop_id=bop_id,
                            action="retained",
                            files={
                                str(p.relative_to(directory)): sha256(p)
                                for p in sorted(directory.rglob("*"))
                                if p.is_file()
                            },
                        )
                    )
                    continue
                if (output / name).exists() or (output / name).is_symlink():
                    raise ValueError(f"refusing to overwrite existing path {output / name}")
                if bop_id not in RECIPES:
                    raise ValueError(f"no validated import recipe for {name}")
                stage = Path(temporary) / name
                stage.mkdir()
                rows.append(_build(z, stage, bop_id))
                staged.append((stage, output / name))
        try:
            for stage, destination in staged:
                if destination.exists() or destination.is_symlink():
                    raise FileExistsError(destination)
                os.rename(stage, destination)
                published.append(destination)
                os.rename(destination / ".meta.yaml.pending", destination / "meta.yaml")
        except OSError:
            for destination in reversed(published):
                shutil.rmtree(destination)
            raise
    return dict(source=dict(url=ARCHIVE_URL, sha256=ARCHIVE_SHA256), objects=rows)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--objects", nargs="+", required=True, choices=sorted(canonical_name(i) for i in BOP_YCB_NAMES))
    p.add_argument(
        "--output", type=Path, required=True, help="YCB-V collection directory; existing assets are retained"
    )
    p.add_argument("--archive", type=Path)
    p.add_argument("--cache", type=Path, default=Path.home() / ".cache/prl_assets")
    p.add_argument("--report", type=Path, default=Path("artifacts/ycbv_import.json"))
    args = p.parse_args()
    args.output = args.output.resolve()
    args.report = args.report.resolve()
    if args.report.is_relative_to(args.output):
        p.error("the report must be outside the asset collection")
    if len(set(args.objects)) != len(args.objects):
        p.error("select distinct object names")
    try:
        archive = verify_archive(args.archive) if args.archive else fetch_archive(args.cache)
        result = import_objects(args.objects, args.output, archive)
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        p.exit(1, f"YCB-V import failed: {error}\n")
    try:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        _json(args.report, result)
    except OSError as error:
        p.exit(1, f"Assets are valid, but the report could not be written: {error}\n")
    print(f"Validated {len(result['objects'])} objects; report: {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
