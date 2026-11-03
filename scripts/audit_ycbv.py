#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Checkout entry point; the installed audit lives in prl_assets.ycbv_audit."""
from pathlib import Path

from prl_assets.ycbv_audit import audit, main  # audit remains available to the import integration check

if __name__ == "__main__":
    raise SystemExit(main(record_path=Path(__file__).resolve().parents[1] / "docs/ycbv_catalog.json"))
