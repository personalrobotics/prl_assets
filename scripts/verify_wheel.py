#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Verify one built wheel outside the checkout, using the committed dependency lock.

Run from the repository: uv run --locked --all-extras python scripts/verify_wheel.py
WHEEL ARCHIVE OUTPUT. Requires uv and a headless OpenGL backend (MUJOCO_GL=egl).
The exact wheel, requirement exports and inspection results remain in OUTPUT.
"""

from __future__ import annotations

import argparse
import email.parser
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = (
    "asset-manager @ git+https://github.com/personalrobotics/asset_manager.git@f421eb375fe76a0d9c69dcf0dd08ef48d718a969"
)


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def run(args, *, cwd=ROOT, env=None, stdout=None):
    subprocess.run([str(x) for x in args], cwd=cwd, env=env, stdout=stdout, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wheel", type=Path)
    parser.add_argument("archive", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    try:
        wheel = args.wheel.resolve(strict=True)
        archive = args.archive.resolve(strict=True)
    except OSError as error:
        parser.error(str(error))
    if not wheel.is_file() or not archive.is_file():
        parser.error("wheel and reference archive must be regular files")
    # Never write inspection outputs into the maintained package or modify
    # the wheel while verifying it.
    output = args.output.resolve()
    if output.is_relative_to((ROOT / "src/prl_assets").resolve()):
        parser.error("verification output must be outside the maintained package")
    if output.exists():
        parser.error("use a new output directory to preserve previous verification evidence")
    output.mkdir(parents=True)
    before = digest(wheel)
    with zipfile.ZipFile(wheel) as bundle:
        for name in bundle.namelist():
            path = PurePosixPath(name)
            if path.is_absolute() or ".." in path.parts or "\\" in name:
                raise ValueError(f"unsafe wheel path: {name}")
            if "__pycache__" in path.parts or path.name.endswith((".pyc", ".pending")) or path.name == ".import.lock":
                raise ValueError(f"temporary build/import file in wheel: {name}")
        (metadata,) = [n for n in bundle.namelist() if n.endswith(".dist-info/METADATA")]
        headers = email.parser.Parser().parsestr(bundle.read(metadata).decode())
        if headers["License-Expression"] != "MIT AND CC-BY-4.0":
            raise ValueError("wheel must retain both code and scan license terms")
        for name in (
            "NOTICE.md",
            "LICENSE-CC-BY-4.0.txt",
            "LICENSE-YCB-VIDEO-MIT.txt",
            "catalog_audit.json",
            "overlap_audit.json",
        ):
            if len(bundle.read("prl_assets/objects/ycbv/" + name)) < 100:
                raise ValueError(f"missing or empty packaged notice/audit: {name}")
        source_root = ROOT / "src"
        force_include = {
            "prl_assets/objects/ycbv/catalog_audit.json": ROOT / "docs/ycbv_catalog.json",
            "prl_assets/objects/ycbv/overlap_audit.json": ROOT / "docs/ycbv_audit.json",
        }
        expected_metadata = {
            p.relative_to(source_root).as_posix() for p in (source_root / "prl_assets/objects").rglob("meta.yaml")
        }
        installed_metadata = {
            n for n in bundle.namelist() if n.startswith("prl_assets/objects/") and n.endswith("/meta.yaml")
        }
        if expected_metadata != installed_metadata:
            raise ValueError("wheel must preserve every catalog registration from the source tree")
        source_files_checked = 0
        for name in bundle.namelist():
            if name.startswith("prl_assets/") and not name.endswith("/"):
                source = force_include.get(name, source_root / name)
                source = source.resolve()
                if (
                    not source.is_relative_to(ROOT)
                    or not source.is_file()
                    or hashlib.sha256(bundle.read(name)).hexdigest() != digest(source)
                ):
                    raise ValueError(f"wheel resource differs from the maintained source: {name}")
                source_files_checked += 1
        if headers.get_payload().rstrip() != (ROOT / "README.md").read_text().rstrip():
            raise ValueError("wheel description differs from the maintained README")
        unpacked_size = sum(info.file_size for info in bundle.infolist())
    base = output / "base-requirements.txt"
    full = output / "verification-requirements.txt"
    for path, extras in ((base, []), (full, ["--all-extras"])):
        run(
            ["uv", "export", "--locked", "--no-emit-project", "--no-default-groups", *extras, "--output-file", path],
            stdout=subprocess.DEVNULL,
        )
    with tempfile.TemporaryDirectory(prefix="prl-assets-installed-") as temporary:
        work = Path(temporary)
        venv = work / "environment"
        run(["uv", "venv", "--python", sys.executable, venv])
        python = venv / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
        run(["uv", "pip", "install", "--python", python, "--require-hashes", "-r", base])
        run(["uv", "pip", "install", "--python", python, "--no-deps", wheel])
        # No editable checkout, PYTHONPATH, optional authoring libraries or
        # implicit dataset cache can make this base installation succeed.
        check = """import importlib.util, pathlib, sys
import prl_assets
from prl_assets.grasp_regions import parse
assert parse({'name': 'legacy', 'geometric_properties': {'type': 'box'}}) == ()
assert pathlib.Path(prl_assets.__file__).resolve().is_relative_to(pathlib.Path(sys.prefix).resolve())
assert importlib.util.find_spec('mujoco') is None
assert importlib.util.find_spec('scipy') is None
assert len(list(prl_assets.YCBV_OBJECTS_DIR.rglob('meta.yaml'))) == 21
print('PASS base wheel imports and includes all 21 identities without authoring dependencies')
"""
        run([python, "-I", "-c", check], cwd=work)
        run(["uv", "pip", "install", "--python", python, "--require-hashes", "-r", full])
        run(["uv", "pip", "install", "--python", python, "--no-deps", "--no-build-isolation", REGISTRY])
        regions_check = """import sys
from pathlib import Path
from asset_manager import AssetManager
from prl_assets import OBJECTS_DIR
from prl_assets.grasp_regions import parse
from prl_assets.ycbv_audit import write_json
catalog = AssetManager(OBJECTS_DIR)
rows = []
for name in catalog.list():
    meta = catalog.get(name)
    regions = parse(meta)
    rows.append(dict(name=name, migration_status='grasp_regions' if 'grasp_regions' in meta else 'legacy only',
        aliases=meta.get('perception', {}).get('aliases', []),
        bop_id=meta.get('dataset', {}).get('ycbv', {}).get('bop_id'),
        T_asset_bop=meta.get('frames', {}).get('T_asset_bop'),
        regions=[dict(name=r.name, primitive=r.primitive, params=dict(r.params),
            strategies=r.strategies, T_asset_region=r.T_asset_region,
            bottom_face_frame=r.bottom_face_frame().tolist(), bounds_in_asset=r.bounds_in_asset().tolist()) for r in regions]))
assert [r.name for r in parse(catalog.get('mug'))] == ['body', 'handle']
write_json(Path(sys.argv[1]), dict(objects=rows))
print('PASS installed asset grasp regions and preserved catalog identities')
"""
        run([python, "-I", "-c", regions_check, output / "grasp_regions.json"], cwd=work)
        env = os.environ.copy()
        env.pop("PYTHONPATH", None)
        env.setdefault("MUJOCO_GL", "egl")
        run(
            [python, "-I", "-m", "prl_assets.verify_ycbv", "--archive", archive, "--out", output / "inspection"],
            cwd=work,
            env=env,
        )
        package = venv / (
            "Lib/site-packages/prl_assets"
            if os.name == "nt"
            else f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/prl_assets"
        )
        installed_size = sum(p.stat().st_size for p in package.rglob("*") if p.is_file())
    if digest(wheel) != before:
        raise ValueError("wheel changed during verification")
    copied = output / wheel.name
    shutil.copyfile(wheel, copied)
    if digest(copied) != before:
        raise ValueError("published copy differs from the verified wheel")
    manifest = dict(
        wheel=wheel.name,
        sha256=before,
        wheel_bytes=wheel.stat().st_size,
        wheel_uncompressed_bytes=unpacked_size,
        installed_package_bytes=installed_size,
        license_expression=headers["License-Expression"],
        source_files_checked=source_files_checked,
        base_installation_passed=True,
        installed_collection_passed=True,
        dependency_lock_sha256=digest(ROOT / "uv.lock"),
        inspection="inspection/validation.json",
    )
    # The successful manifest is the commit marker for this verification run.
    fd, staged = tempfile.mkstemp(dir=output, suffix=".json.tmp")
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        os.replace(staged, output / "wheel-manifest.json")
    finally:
        Path(staged).unlink(missing_ok=True)
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
