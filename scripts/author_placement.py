#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

"""Measure and write objects' ``placement`` sections (docs/schema.md).

Each object's canonical rest is measured from its compiled model, with the openings and
supports it asks for, and written into its ``meta.yaml`` by
``prl_assets.authoring.write_placement``: the section is replaced, or appended, and
nothing else in the file changes. An object that already declares a section is
re-measured with the openings and supports it declares, so a rerun after a collider
change refreshes it, and a rerun on an unchanged object changes nothing.

Usage:
    uv run --extra authoring python scripts/author_placement.py can mug       # refresh, or a rest only
    uv run --extra authoring python scripts/author_placement.py recycle_bin --opening top=rectangle
    uv run --extra authoring python scripts/author_placement.py ada_table --support top
    uv run --extra authoring python scripts/author_placement.py --all         # refresh every object

Exits 2 for an unknown object name.
"""

import argparse
import sys

from prl_assets import authoring
from prl_assets.placement import parse


def main() -> int:
    parser = argparse.ArgumentParser(description="Measure and write objects' placement sections")
    parser.add_argument("objects", nargs="*", help="Object names")
    parser.add_argument("--all", action="store_true", help="every object")
    parser.add_argument("--opening", action="append", default=[], metavar="NAME=SHAPE", help="rectangle or circle")
    parser.add_argument("--support", action="append", default=[], metavar="NAME", help="a top surface")
    args = parser.parse_args()

    known = authoring.object_dirs()
    names = sorted(known) if args.all else args.objects
    unknown = [n for n in names if n not in known]
    if unknown or not names:
        print(f"Unknown objects: {', '.join(unknown)}. Available: {', '.join(sorted(known))}")
        return 2
    asked = dict(item.split("=", 1) for item in args.opening)
    for name in names:
        obj = authoring.load(name)
        declared = parse(obj.meta)
        openings = dict(asked)
        supports = tuple(args.support)
        if declared is not None:
            openings = {o.name: o.shape for o in declared.openings} | openings
            supports = tuple(dict.fromkeys((*(s.name for s in declared.supports), *supports)))
        placement = authoring.measure_placement(obj, openings, supports)
        before = obj.dir.joinpath("meta.yaml").read_text()
        path = authoring.write_placement(obj, placement)
        print(f"{name}: {'unchanged' if path.read_text() == before else 'written'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
