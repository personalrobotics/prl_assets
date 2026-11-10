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
# Reference masses: Calli et al. (2015), Table 2. These are not measurements
# of a lab copy. Closed, approximately convex products use a convex envelope;
# handles, bent shapes and open interiors retain separate convex pieces.
#
# Each decomposition is the coarsest one that still passes authoring.check (including
# the floor probe of open interiors), keeps the reference silhouette's apertures (jaw
# gaps, handle and scissor loops) open in verify_ycbv, and bridges the scan's
# concavities by at most about 13 mm at the 99th percentile. It was chosen from a sweep
# of thresholds, part caps and decimation (#42). The default 2 mm threshold traced scan
# noise and produced up to 567 parts per object. A box product is a box primitive, and
# a hull may be cut flat at its base (base_cut_mm), where the scan's flat face is not (#54).
_ROUNDED_EDGES = "the scan rounds the box's edges; the box keeps flat faces for resting and face grasps"
RECIPES = {
    1: dict(mass=0.414, material="metal", category=["food_package", "graspable"], collision="hull"),
    4: dict(mass=0.349, material="metal", category=["food_package", "graspable"], collision="hull"),
    # A bottle's scanned base is a recessed ring whose points differ by 1-2 mm, so its hull
    # rests on one tilted triangle with the centre of mass near its edge (2.6 deg). Cut
    # flat 0.5 mm up, it stands on its whole base (15-17 deg; #54).
    5: dict(mass=0.603, material="plastic", category=["food_package", "graspable"], collision="hull", base_cut_mm=0.5),
    # A thin can at the bin-drop impact speed needs the 4 ms contact time
    # constant used by the bin floor; default softness lets it pass through.
    6: dict(mass=0.171, material="metal", category=["food_package", "graspable"], collision="hull", solref="0.004 1"),
    # Boxes are boxes, as the retained cracker, sugar and gelatin boxes are: the scan's
    # faces are warped by up to 4 mm, so its hull rested flat at a 0.6 deg margin (#54).
    7: dict(
        mass=0.187,
        material="cardboard",
        category=["food_package", "graspable"],
        collision="box",
        collision_tolerance_mm=dict(collision_excess=5.5, reason=_ROUNDED_EDGES),
    ),
    10: dict(mass=0.066, material="unspecified", category=["food", "graspable"], decomposition=dict(threshold=0.02)),
    # The benchmark pitcher scan is capped across the top; it has no usable
    # open interior. Preserve its handle, but do not advertise it as fillable.
    # Decimated parts fall 3.7 mm short of its surface; undecimated ones do not.
    11: dict(
        mass=0.178,
        material="plastic",
        category=["container", "tableware", "graspable"],
        decomposition=dict(threshold=0.01, max_convex_hull=16, decimate=False),
    ),
    12: dict(mass=1.131, material="plastic", category=["household", "graspable"], collision="hull", base_cut_mm=0.5),
    # 18 parts also pass, at a 2.35 mm floor gap against the 3 mm limit; 28 leave margin.
    13: dict(
        mass=0.147,
        material="unspecified",
        category=["container", "tableware", "graspable"],
        open=True,
        opening="circle",  # placement.openings.top
        decomposition=dict(threshold=0.0075, max_convex_hull=32),
    ),
    14: dict(
        mass=0.118,
        material="unspecified",
        category=["container", "tableware", "graspable"],
        open=True,
        opening="circle",
        decomposition=dict(threshold=0.01, max_convex_hull=32),
    ),
    15: dict(mass=0.895, material="unspecified", category=["tool", "graspable"], decomposition=dict(threshold=0.02)),
    16: dict(
        mass=0.729,
        material="wood",
        category=["block", "graspable"],
        collision="box",
        collision_tolerance_mm=dict(collision_excess=3.5, reason=_ROUNDED_EDGES),
    ),
    17: dict(
        mass=0.082,
        material="unspecified",
        category=["tool", "graspable"],
        decomposition=dict(threshold=0.01, max_convex_hull=16),
    ),
    18: dict(mass=0.0158, material="plastic", category=["tool", "graspable"], collision="hull"),
    # The jaw gap stays open at 0.0075 / 32 parts (6% occupied); 0.01 leaves 12%.
    19: dict(
        mass=0.125,
        material="unspecified",
        category=["tool", "graspable"],
        decomposition=dict(threshold=0.0075, max_convex_hull=32),
    ),
    20: dict(
        mass=0.202,
        material="unspecified",
        category=["tool", "graspable"],
        decomposition=dict(threshold=0.01, max_convex_hull=16),
    ),
    21: dict(mass=0.028, material="foam", category=["block", "graspable"], collision="box"),
}
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


def _check_contained(directory: Path) -> Path:
    """Refuse an existing asset whose model or resources resolve outside its directory.
    Returns the resolved model path."""
    import xml.etree.ElementTree as ET

    import yaml

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
        if element.tag == "compiler" and any(element.get(k) for k in ("meshdir", "texturedir", "assetdir")):
            raise ValueError("retained YCB-V assets must use local resource paths")
        if element.get("file") and not (directory / element.get("file")).resolve().is_relative_to(directory.resolve()):
            raise ValueError(f"resource path leaves {directory}")
    for resource in directory.rglob("*"):
        if resource.is_file() and not resource.resolve().is_relative_to(directory.resolve()):
            raise ValueError(f"linked resource leaves {directory}")
    return model_path


def _cut_below(hull, height: float):
    """``hull`` without the part less than ``height`` above its lowest point, so it rests
    on a flat face: its vertices above that plane and its edges' crossings of it, hulled
    again. trimesh's own slicing needs shapely, which the importer does not depend on."""
    import numpy as np
    import trimesh

    v = np.asarray(hull.vertices)
    z = float(v[:, 2].min()) + height
    a, b = hull.edges_unique.T
    above_a, above_b = v[a, 2] - z, v[b, 2] - z
    crossing = above_a * above_b < 0
    t = above_a[crossing] / (above_a[crossing] - above_b[crossing])
    points = v[a[crossing]] + t[:, None] * (v[b[crossing]] - v[a[crossing]])
    points[:, 2] = z  # exactly on the plane, so the cut is one face
    return trimesh.Trimesh(vertices=np.vstack([v[v[:, 2] >= z], points]), process=True).convex_hull


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
    text = "\n".join(line for line in text.splitlines() if not line.startswith(("mtllib ", "usemtl "))).rstrip() + "\n"
    path.write_text(text)


def _build(archive: zipfile.ZipFile, stage: Path, bop_id: int, recipe: dict | None = None) -> dict:
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
    recipe = RECIPES[bop_id] if recipe is None else recipe
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
    settings = {**DECOMPOSITION}
    if recipe.get("open"):
        # Preserve thin walls and open interiors in the manifold preprocessing.
        settings["preprocess_resolution"] = 100
    settings.update(recipe.get("decomposition", {}))
    method = recipe.get("collision", "decomposition")
    box = None
    if method == "box":
        # Upright, so the model file's pose rests on a face: yawed about z only, the
        # minimum-area rectangle of the scan's xy over its z range.
        yaw, (sx, sy), (cx, cy) = authoring._min_rectangle(np.asarray(mesh.vertices)[:, :2])
        z0, z1 = float(mesh.vertices[:, 2].min()), float(mesh.vertices[:, 2].max())
        box = dict(size=(sx / 2, sy / 2, (z1 - z0) / 2), pos=(cx, cy, (z0 + z1) / 2), yaw=math.radians(yaw))
        solid = trimesh.creation.box(extents=(sx, sy, z1 - z0))
        solid.apply_transform(trimesh.transformations.rotation_matrix(box["yaw"], (0, 0, 1)))
        solid.apply_translation(box["pos"])
        hulls = [solid]
    elif method == "hull":
        hulls = [collision_input.convex_hull]
        if "base_cut_mm" in recipe:
            hulls = [_cut_below(hulls[0], recipe["base_cut_mm"] / 1e3)]
    else:
        coacd.set_log_level("warn")
        parts = coacd.run_coacd(coacd.Mesh(collision_input.vertices, collision_input.faces), **settings)
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
    if box is not None:
        # The box's own pose in the body: the body is lifted so its bottom rests on z = 0.
        ET.SubElement(
            body,
            "geom",
            name=f"{name}_collision",
            type="box",
            size=" ".join(f"{v:.9g}" for v in box["size"]),
            pos=" ".join(f"{v:.9g}" for v in box["pos"]),
            quat=" ".join(f"{v:.9g}" for v in (math.cos(box["yaw"] / 2), 0.0, 0.0, math.sin(box["yaw"] / 2))),
            mass=f"{recipe['mass']:.12g}",
            friction="0.6 0.005 0.0001",
            group="3",
            condim="4",
            **({"solref": recipe["solref"]} if "solref" in recipe else {}),
        )
        hulls = []  # no mesh files: MuJoCo collides the primitive
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
            **({"solref": recipe["solref"]} if "solref" in recipe else {}),
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
        geometric_properties=dict(
            type="box",
            size=extents,
            **({"open": True} if recipe.get("open") else {}),
            **{k: recipe[k] for k in ("collision_tolerance_mm",) if k in recipe},
        ),
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
                **(
                    {"contact": f"MuJoCo solref {recipe['solref']}; 4 ms stiffness for thin-can bin impacts"}
                    if "solref" in recipe
                    else {}
                ),
            ),
            conversion=dict(
                coacd=COACD_VERSION,
                method=method,
                settings=settings
                if method == "decomposition"
                else ({"base_cut_mm": recipe["base_cut_mm"]} if "base_cut_mm" in recipe else {}),
                texture_size=[1024, 1024],
                environment={
                    name: importlib.metadata.version(name) for name in ("numpy", "trimesh", "mujoco", "pillow", "scipy")
                },
            ),
        ),
    )
    spec = mujoco.MjSpec.from_file(str(stage / f"{name}.xml"))
    compiled = authoring._compiled(name, stage, meta, spec)
    # Placement is measured from the model just built, like every catalog object's.
    placement = authoring.measure_placement(compiled, {"top": recipe["opening"]} if "opening" in recipe else None)
    if placement is not None:
        meta["placement"] = authoring.placement_section(placement)  # compiled.meta is this dict
    # An object with a grasp recipe gets its regions measured from the visual model too.
    regions = authoring.measure_grasp_regions(compiled) if name in authoring.GRASP_RECIPES else ()
    if regions:
        meta["grasp_regions"] = authoring.grasp_regions_section(regions)
    report = authoring.check(compiled)
    if not report.ok:
        raise ValueError(f"{name} failed collision validation: {report.failures}")
    shutil.rmtree(source)
    # No discoverable metadata exists until publication. A partial staged import
    # is therefore invisible even if output is inside the catalog being scanned.
    rest = yaml.safe_dump({k: v for k, v in meta.items() if k not in ("placement", "grasp_regions")}, sort_keys=False)
    text = authoring.with_placement(rest, placement) if placement else rest
    (stage / ".meta.yaml.pending").write_text(authoring.with_grasp_regions(text, regions) if regions else text)
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
                    from prl_assets import authoring

                    _check_contained(directory)
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


def _dump_like(meta: dict, previous: str) -> str:
    """``meta`` as YAML, keeping the blank lines that separated top-level sections in
    ``previous``, so a rebuild's diff shows only the values that changed."""
    import yaml

    lines = previous.splitlines()
    spaced = {
        lines[i + 1].split(":", 1)[0]
        for i in range(len(lines) - 1)
        if not lines[i].strip() and lines[i + 1][:1].isalpha()
    }
    out = []
    for line in yaml.safe_dump(meta, sort_keys=False).splitlines():
        if line[:1].isalpha() and line.split(":", 1)[0] in spaced:
            out.append("")
        out.append(line)
    return "\n".join(out) + "\n"


def _keep_section(text: str, previous: str, key: str) -> str:
    """``text`` with its top-level ``key`` section replaced by ``previous``'s, verbatim. A
    section ends at the next line that starts in column 0, as ``authoring.with_placement``
    takes it, less trailing blank lines."""
    import re

    def bounds(lines):
        start = next((i for i, line in enumerate(lines) if line.startswith(f"{key}:")), None)
        if start is None:
            return None
        end = next((i for i in range(start + 1, len(lines)) if re.match(r"\S", lines[i])), len(lines))
        while end > start + 1 and not lines[end - 1].strip():
            end -= 1
        return start, end

    new, old = text.splitlines(keepends=True), previous.splitlines(keepends=True)
    here, there = bounds(new), bounds(old)
    if here is None or there is None:
        return text
    return "".join(new[: here[0]] + old[there[0] : there[1]] + new[here[1] :])


def _collision_parts(path: Path) -> tuple[bytes, frozenset]:
    """A model file without its collision meshes, collision geoms and resting position,
    and the set of attribute sets its collision geoms carry beyond their geometry: name,
    mesh, mass, and a primitive's type, size and pose. A collider may change between a
    mesh and a primitive; an attribute tuned by hand, such as solref or a mesh geom's
    pose, may not."""
    import xml.etree.ElementTree as ET

    from prl_assets.authoring import COLLISION_GROUP

    tree = ET.parse(path).getroot()
    attributes = set()
    for parent in tree.iter():
        for child in list(parent):
            if child.tag == "mesh" and child.get("name", "").startswith("collision_"):
                parent.remove(child)
            elif child.tag == "geom" and child.get("group") == str(COLLISION_GROUP):
                # A primitive's type, size and pose are fitted; a mesh's pose is not.
                geometry = (
                    ("name", "mesh", "mass")
                    if "mesh" in child.attrib
                    else ("name", "mass", "type", "size", "pos", "quat")
                )
                attributes.add(frozenset((k, v) for k, v in child.attrib.items() if k not in geometry))
                parent.remove(child)
    for body in tree.iter("body"):
        body.attrib.pop("pos", None)
    ET.indent(tree)  # a primitive has no mesh asset, so the stripped files' layout differs
    return ET.tostring(tree), frozenset(attributes)


def rebuild_collisions(objects: list[str], output: Path, archive: Path) -> dict:
    """Rebuild existing assets' collision models from their current recipes.

    Only the collision parts are replaced: the collision mesh files, the collision
    geoms and resting body position in the model file, ``dimensions`` and a legacy box
    ``geometric_properties.size`` that tracked them, ``provenance.conversion``, and any
    ``placement`` section, re-measured from the new collider with the openings and
    supports it declares. Every other file and metadata section, including hand-authored ones such
    as ``grasp_regions``, is kept. The regenerated visual mesh must equal the committed
    one, so the collider is derived from the geometry that is shown, and the model file
    must agree outside its collision geoms, including any collision-geom attribute a
    person tuned by hand.

    Every requested object is rebuilt and validated before any is replaced. Staged and
    retired copies hold their metadata under hidden names, so a registry never sees
    them, and a failure while replacing restores every object already replaced.
    """
    import mujoco
    import yaml

    from prl_assets import authoring
    from prl_assets.grasp_regions import parse as parse_grasp_regions
    from prl_assets.placement import parse as parse_placement

    allowed = {canonical_name(i): i for i in BOP_YCB_NAMES}
    if not objects or len(set(objects)) != len(objects) or any(name not in allowed for name in objects):
        raise ValueError("select distinct canonical YCB-V names (no paths or unknown names)")
    archive = verify_archive(archive)
    output = Path(output).resolve()
    rows, staged = [], []
    with _lock(output), tempfile.TemporaryDirectory(dir=output, prefix=".ycbv-stage-") as temporary:
        existing = _existing(output)
        with zipfile.ZipFile(archive) as z:
            for name in objects:
                bop_id = allowed[name]
                if bop_id not in existing or bop_id not in RECIPES:
                    raise ValueError(f"{name} is not an imported asset with a recipe")
                directory = existing[bop_id]
                model_path = _check_contained(directory)
                if any(p.is_symlink() for p in directory.rglob("*")):
                    raise ValueError(f"{name}: refusing to rebuild an asset that contains symbolic links")
                built = Path(temporary) / f"{name}.built"
                built.mkdir()
                _build(z, built, bop_id)
                visual = f"{name}_visual.obj"
                if (built / visual).read_bytes() != (directory / visual).read_bytes():
                    raise ValueError(f"{name}: the regenerated visual mesh differs from the committed one")
                rebuilt, rebuilt_attributes = _collision_parts(built / f"{name}.xml")
                committed, committed_attributes = _collision_parts(model_path)
                if rebuilt != committed:
                    raise ValueError(f"{name}: the model file differs outside its collision geometry")
                if rebuilt_attributes != committed_attributes:
                    raise ValueError(
                        f"{name}: the committed collision geoms carry attributes the recipe does not "
                        "reproduce; add them to the recipe before rebuilding"
                    )

                previous = (directory / "meta.yaml").read_text()
                old_meta = yaml.safe_load(previous)
                new_meta = yaml.safe_load((built / ".meta.yaml.pending").read_text())
                meta = dict(old_meta)
                gp = dict(meta.get("geometric_properties") or {})
                if gp.get("type") == "box" and gp.get("size") == old_meta.get("dimensions"):
                    gp["size"] = new_meta["dimensions"]
                # An imported object's widened checks are its recipe's, and go with it.
                gp.pop("collision_tolerance_mm", None)
                if "collision_tolerance_mm" in new_meta["geometric_properties"]:
                    gp["collision_tolerance_mm"] = new_meta["geometric_properties"]["collision_tolerance_mm"]
                meta["geometric_properties"] = gp
                recipe = name in authoring.GRASP_RECIPES
                if not recipe and "grasp_regions" in old_meta and new_meta["dimensions"] != old_meta.get("dimensions"):
                    # Regions declared by hand describe the object's shape; refuse rather
                    # than leave them describing a different one. A recipe object's regions
                    # are measured from its visual model, which a rebuild leaves as it was,
                    # and check re-measures them.
                    raise ValueError(
                        f"{name}: the rebuild changes dimensions, and the object declares "
                        "grasp_regions; re-author them with the new collider"
                    )
                meta["dimensions"] = new_meta["dimensions"]
                meta["provenance"] = {**old_meta["provenance"], "conversion": new_meta["provenance"]["conversion"]}

                stage = Path(temporary) / name
                shutil.copytree(directory, stage)
                (stage / "meta.yaml").unlink()
                for old in stage.glob("collision_*.obj"):
                    old.unlink()
                for new in built.glob("collision_*.obj"):
                    shutil.copyfile(new, stage / new.name)
                shutil.copyfile(built / f"{name}.xml", stage / model_path.relative_to(directory.resolve()))
                spec = mujoco.MjSpec.from_file(str(stage / model_path.relative_to(directory.resolve())))
                compiled = authoring._compiled(name, stage, meta, spec)
                placement = authoring.remeasure_placement(compiled, parse_placement(old_meta))
                if placement is not None:
                    meta["placement"] = authoring.placement_section(placement)  # compiled.meta is this dict
                rest = _dump_like({k: v for k, v in meta.items() if k != "placement"}, previous)
                text = authoring.with_placement(rest, placement) if placement else rest
                # A rebuild keeps grasp regions, written by hand or by recipe, as written.
                text = _keep_section(text, previous, "grasp_regions")
                if parse_grasp_regions(yaml.safe_load(text)) != parse_grasp_regions(old_meta):
                    raise ValueError(f"{name}: the kept grasp_regions section would not read back as declared")
                (stage / ".meta.yaml.pending").write_text(text)
                report = authoring.check(compiled)
                if not report.ok:
                    raise ValueError(f"{name} failed collision validation: {report.failures}")
                staged.append((name, bop_id, directory, stage, old_meta, new_meta, report))

        # Publish. The old object becomes invisible (hidden metadata) before it moves, and
        # the new one visible only once it is in place; on any failure, including an
        # interruption, every object already replaced is put back.
        done = []
        try:
            for name, bop_id, directory, stage, old_meta, new_meta, report in staged:
                retired = Path(temporary) / f"{name}.retired"
                os.rename(directory / "meta.yaml", directory / ".meta.yaml.retired")
                done.append((directory, retired, False))
                os.rename(directory, retired)
                done[-1] = (directory, retired, True)
                os.rename(stage, directory)
                os.rename(directory / ".meta.yaml.pending", directory / "meta.yaml")
                rows.append(
                    dict(
                        name=name,
                        bop_id=bop_id,
                        action="collision rebuilt",
                        hulls=len(list(directory.glob("collision_*.obj"))),
                        previous_hulls=len(list(retired.glob("collision_*.obj"))),
                        dimensions_changed=new_meta["dimensions"] != old_meta.get("dimensions"),
                        fit=asdict(report.fit),
                        files={p.name: sha256(p) for p in sorted(directory.iterdir()) if p.is_file()},
                    )
                )
        except BaseException:
            for directory, retired, moved in reversed(done):
                if moved:
                    if directory.exists():
                        shutil.rmtree(directory)
                    os.rename(retired, directory)
                os.rename(directory / ".meta.yaml.retired", directory / "meta.yaml")
            raise
    return dict(source=dict(url=ARCHIVE_URL, sha256=ARCHIVE_SHA256), objects=rows)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--objects", nargs="+", required=True, choices=sorted(canonical_name(i) for i in BOP_YCB_NAMES))
    p.add_argument(
        "--output", type=Path, required=True, help="YCB-V collection directory; existing assets are retained"
    )
    p.add_argument(
        "--rebuild-collision",
        action="store_true",
        help="Rebuild existing assets' collision models from their recipes, keeping everything else",
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
        run = rebuild_collisions if args.rebuild_collision else import_objects
        result = run(args.objects, args.output, archive)
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
