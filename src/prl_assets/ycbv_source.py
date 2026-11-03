# SPDX-License-Identifier: MIT
"""Pinned BOP source shared by the YCB-V importer and frame audit."""

import hashlib
import os
import tempfile
import urllib.request
from pathlib import Path

ARCHIVE_URL = (
    "https://huggingface.co/datasets/bop-benchmark/ycbv/resolve/"
    "5c2c4aa229800355648cd268040aa814f8dc94f0/ycbv_models.zip"
)
ARCHIVE_SHA256 = "1e517a9b513cadb8eaf406ac3ce3c5f1d6fc4a80161c866b4e2a73d13de5e554"
DOWNLOAD_TIMEOUT_S = 60


def sha256(path: Path) -> str:
    """SHA-256 of a local file without loading the archive into memory."""
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def verify_archive(path: Path) -> Path:
    """Accept only the immutable published archive, before parsing or extracting it."""
    path = Path(path).resolve(strict=True)
    if not path.is_file() or sha256(path) != ARCHIVE_SHA256:
        raise ValueError(f"{path} does not match the pinned SHA-256 {ARCHIVE_SHA256}")
    return path


def fetch_archive(cache: Path) -> Path:
    """Reuse a verified cache or atomically publish a fully verified download."""
    cache = Path(cache).resolve()
    path = cache / "ycbv_models.zip"
    if path.is_file() and sha256(path) == ARCHIVE_SHA256:
        return path
    cache.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(dir=cache, suffix=".part")
    try:
        with os.fdopen(fd, "wb") as out, urllib.request.urlopen(ARCHIVE_URL, timeout=DOWNLOAD_TIMEOUT_S) as response:
            for chunk in iter(lambda: response.read(1 << 20), b""):
                out.write(chunk)
        verify_archive(Path(temporary))
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)
    return path
