"""Real-archive import, independent frame/texture audit, and non-destructive failures.

Opt in with PRL_ASSETS_YCBV_ARCHIVE=/path/to/the/pinned/ycbv_models.zip.
Writes artifacts/ycbv_import_validation.json, including the independent measurements.
"""

import json
import os
import runpy
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

import pytest

from prl_assets import YCBV_OBJECTS_DIR
from prl_assets.ycbv_source import sha256


def test_real_archive_import_and_failed_requests_preserve_assets(tmp_path):
    source = os.environ.get("PRL_ASSETS_YCBV_ARCHIVE")
    if not source:
        pytest.skip("real-archive authoring check; run in the cached YCB-V audit job")
    pytest.importorskip("coacd")
    archive = Path(source).expanduser().resolve(strict=True)
    collection = tmp_path / "collection"
    collection.mkdir()
    shutil.copytree(YCBV_OBJECTS_DIR / "cracker_box", collection / "cracker_box")

    def inventory():
        return {
            str(p.relative_to(collection)): sha256(p)
            for p in sorted(collection.rglob("*"))
            if p.is_file() and p.name != ".import.lock"
        }

    before = inventory()
    report = tmp_path / "import.json"

    def invoke(objects, source_archive=archive):
        return subprocess.run(
            [
                sys.executable,
                "-m",
                "prl_assets.import_ycbv",
                "--objects",
                *objects,
                "--output",
                str(collection),
                "--archive",
                str(source_archive),
                "--report",
                str(report),
            ],
            capture_output=True,
            text=True,
            timeout=300,
        )

    done = invoke(["cracker_box", "mustard_bottle"])
    assert done.returncode == 0, done.stdout + done.stderr
    result = json.loads(report.read_text())
    retained, imported = result["objects"]
    assert retained["action"] == "retained" and imported["action"] == "imported"
    after = inventory()
    assert all(after[p] == digest for p, digest in before.items()), "an existing object's files changed"
    for name, digest in imported["files"].items():
        assert sha256(collection / "mustard_bottle" / name) == digest, name

    # The pre-existing audit is independent of conversion/collision fitting. It
    # catches scale, origin, rotation, and texture-coordinate export mistakes.
    audit = runpy.run_path(str(Path(__file__).resolve().parents[1] / "scripts/audit_ycbv.py"))["audit"]
    with zipfile.ZipFile(archive) as z, tempfile.TemporaryDirectory() as temp:
        measured = audit(z, Path(temp), collection / "mustard_bottle")
    assert measured["pass"], measured

    invalid = tmp_path / "bad.zip"
    invalid.write_bytes(b"not the pinned dataset")
    failures = []
    for names, path in [
        (["mustard_bottle"], invalid),
        (["../escape"], archive),
        (["mustard_bottle", "mustard_bottle"], archive),
    ]:
        previous = inventory()
        outcome = invoke(names, path)
        assert outcome.returncode != 0, names
        assert inventory() == previous, "a rejected request changed a valid asset"
        failures.append(dict(objects=names, rejected=True, assets_unchanged=True))
    # A retained object's linked resources are also ingress: do not compile or
    # hash a file outside its asset directory just because the XML names it.
    obj_dir = collection / "cracker_box"
    xml_path = obj_dir / "cracker_box.xml"
    original_xml = xml_path.read_text()
    import xml.etree.ElementTree as ET

    tree = ET.fromstring(original_xml)
    reference = next(e for e in tree.iter("mesh") if e.get("file"))
    original_reference = reference.get("file")
    reference.set("file", "../outside.obj")
    xml_path.write_text(ET.tostring(tree, encoding="unicode"))
    previous = inventory()
    rejected = invoke(["cracker_box"])
    assert rejected.returncode != 0 and inventory() == previous
    xml_path.write_text(original_xml)
    failures.append(dict(case="retained mesh path escape", rejected=True, assets_unchanged=True))

    mesh_path = obj_dir / original_reference
    original_mesh = mesh_path.read_bytes()
    outside = tmp_path / "outside.obj"
    outside.write_bytes(original_mesh)
    mesh_path.unlink()
    mesh_path.symlink_to(outside)
    previous = inventory()
    rejected = invoke(["cracker_box"])
    assert rejected.returncode != 0 and inventory() == previous
    mesh_path.unlink()
    mesh_path.write_bytes(original_mesh)
    failures.append(dict(case="retained mesh symlink escape", rejected=True, assets_unchanged=True))

    assert not (tmp_path / "escape").exists()
    assert not list(collection.glob(".ycbv-stage-*")), "staging files leaked"

    output = Path(os.environ.get("PRL_ASSETS_ARTIFACTS", "artifacts"))
    output.mkdir(parents=True, exist_ok=True)
    (output / "ycbv_import_validation.json").write_text(
        json.dumps(
            dict(import_result=result, independent_audit=measured, rejected_requests=failures),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    )


def test_collision_rebuild_reproduces_the_committed_colliders_and_is_all_or_nothing(tmp_path):
    """--rebuild-collision regenerates only an object's collider (#42). On the platform
    the committed colliders were built on, rebuilding a committed object reproduces it
    byte for byte: a decomposed mug, a box primitive and a hull cut flat at its base
    (#54), with the latter two's grasp regions as written. A collision attribute tuned by
    hand still blocks a rebuild. A request in which one object fails replaces none of
    them and leaves no staged or retired copy that a registry could discover."""
    source = os.environ.get("PRL_ASSETS_YCBV_ARCHIVE")
    if not source:
        pytest.skip("real-archive authoring check; run in the cached YCB-V audit job")
    pytest.importorskip("coacd")
    import platform

    from prl_assets import authoring

    archive = Path(source).expanduser().resolve(strict=True)
    collection = tmp_path / "collection"
    collection.mkdir()
    for name in ("mug", "banana", "pudding_box", "bleach_cleanser"):
        shutil.copytree(YCBV_OBJECTS_DIR / name, collection / name)

    def inventory():
        return {
            str(p.relative_to(collection)): sha256(p)
            for p in sorted(collection.rglob("*"))
            if p.is_file() and p.name != ".import.lock"
        }

    def rebuild(*objects):
        return subprocess.run(
            [
                sys.executable,
                "-m",
                "prl_assets.import_ycbv",
                "--rebuild-collision",
                "--objects",
                *objects,
                "--output",
                str(collection),
                "--archive",
                str(archive),
                "--report",
                str(tmp_path / "rebuild.json"),
            ],
            capture_output=True,
            text=True,
            timeout=600,
        )

    committed = inventory()
    result = rebuild("mug", "pudding_box", "bleach_cleanser")
    assert result.returncode == 0, result.stderr
    if platform.system() == "Linux" and platform.machine() == "x86_64":
        assert inventory() == committed, "rebuilding the committed objects changed them"

    model = collection / "pudding_box" / "pudding_box.xml"
    model.write_text(model.read_text().replace('type="box"', 'type="box" solref="0.004 1"'))
    before = inventory()
    result = rebuild("pudding_box")
    assert result.returncode != 0 and "attributes the recipe does not reproduce" in result.stderr, result.stderr
    assert inventory() == before

    # A visual mesh that no longer matches the archive makes the banana fail; the mug,
    # requested first and valid, must not be replaced either.
    before = inventory()
    banana = collection / "banana" / "banana_visual.obj"
    banana.write_text(banana.read_text() + "# edited\n")
    before = inventory()
    result = rebuild("mug", "banana")
    assert result.returncode != 0 and "visual mesh differs" in result.stderr, result.stderr
    assert inventory() == before
    assert set(authoring.object_dirs(collection)) == {"mug", "banana", "pudding_box", "bleach_cleanser"}
