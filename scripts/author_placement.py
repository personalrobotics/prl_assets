#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

"""Measure and write objects' ``placement`` sections (docs/schema.md).

The openings and supports an object asks for are measured from its compiled model and
written into its ``meta.yaml`` by
``prl_assets.authoring.write_placement``: the section is replaced, or appended, and
nothing else in the file changes. An object that already declares a section is
re-measured with the openings and supports it declares, so a rerun after a collider
change refreshes it, and a rerun on an unchanged object changes nothing.

Usage:
    uv run --extra authoring python scripts/author_placement.py mug           # refresh what mug declares
    uv run --extra authoring python scripts/author_placement.py recycle_bin --opening top=rectangle
    uv run --extra authoring python scripts/author_placement.py ada_table --support top
    uv run --extra authoring python scripts/author_placement.py --all         # refresh every object

How an object rests is not declared: a placement planner works it out from the
collision geometry. Exits 2 for an unknown object name, and 1 when an object asks for
no opening or support.
"""

import argparse
import sys

from prl_assets import authoring
from prl_assets.placement import SHAPES, parse


def _opening(text: str) -> tuple[str, str]:
    name, _, shape = text.partition("=")
    if not name or shape not in SHAPES:
        raise argparse.ArgumentTypeError(f"expected NAME=SHAPE, SHAPE one of {', '.join(SHAPES)}; not {text!r}")
    return name, shape


def main() -> int:
    parser = argparse.ArgumentParser(description="Measure and write objects' placement sections")
    parser.add_argument("objects", nargs="*", help="Object names")
    parser.add_argument("--all", action="store_true", help="every object")
    parser.add_argument("--opening", action="append", default=[], type=_opening, metavar="NAME=SHAPE")
    parser.add_argument("--support", action="append", default=[], metavar="NAME", help="a top surface")
    args = parser.parse_args()
    if not args.all and not args.objects:
        parser.error("name the objects, or pass --all")

    known = authoring.object_dirs()
    names = sorted(known) if args.all else args.objects
    unknown = [n for n in names if n not in known]
    if unknown:
        print(f"Unknown objects: {', '.join(unknown)}. Available: {', '.join(sorted(known))}")
        return 2
    status = 0
    for name in names:
        obj = authoring.load(name)
        declared = parse(obj.meta)
        openings = {} if declared is None else {o.name: o.shape for o in declared.openings}
        supports = () if declared is None else tuple(s.name for s in declared.supports)
        openings |= dict(args.opening)
        supports = tuple(dict.fromkeys((*supports, *args.support)))
        placement = authoring.measure_placement(obj, openings, supports)
        if placement is None:
            if not args.all:
                print(f"{name}: declares no opening or support; pass --opening or --support")
                status = 1
            continue
        before = obj.dir.joinpath("meta.yaml").read_text()
        path = authoring.write_placement(obj, placement)
        print(f"{name}: {'unchanged' if path.read_text() == before else 'written'}")
    return status


if __name__ == "__main__":
    sys.exit(main())
