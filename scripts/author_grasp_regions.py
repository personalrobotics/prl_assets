#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

"""Measure and write objects' ``grasp_regions`` sections from their recipes.

An object's recipe (``prl_assets.grasp_recipes.GRASP_RECIPES``) names its regions, their
primitives and strategies, and the part of the visual model each is measured from;
``prl_assets.authoring.measure_grasp_regions`` measures them and
``authoring.write_grasp_regions`` writes the section into its ``meta.yaml``: replaced, or
appended, and nothing else in the file changes. A rerun on an unchanged object changes
nothing. This script only runs the recipes; they live in the package, where ``check``
reads them too.

Usage:
    uv run --extra authoring python scripts/author_grasp_regions.py tomato_soup_can
    uv run --extra authoring python scripts/author_grasp_regions.py --all            # every recipe
    uv run --extra authoring python scripts/author_grasp_regions.py --all --dry-run  # print, write nothing

Exits 2 for a name that has no recipe.
"""

import argparse
import sys

from prl_assets import authoring
from prl_assets.grasp_recipes import GRASP_RECIPES


def main() -> int:
    parser = argparse.ArgumentParser(description="Measure and write objects' grasp_regions sections")
    parser.add_argument("objects", nargs="*", help="Object names")
    parser.add_argument("--all", action="store_true", help="every object with a recipe")
    parser.add_argument("--dry-run", action="store_true", help="print the sections, write nothing")
    args = parser.parse_args()
    if not args.all and not args.objects:
        parser.error("name the objects, or pass --all")
    names = sorted(GRASP_RECIPES) if args.all else args.objects
    unknown = [n for n in names if n not in GRASP_RECIPES]
    if unknown:
        print(f"No recipe for: {', '.join(unknown)}. Recipes: {', '.join(sorted(GRASP_RECIPES))}")
        return 2
    for name in names:
        obj = authoring.load(name)
        regions = authoring.measure_grasp_regions(obj)
        if args.dry_run:
            print(f"# {name}\n{authoring.grasp_regions_yaml(regions)}")
            continue
        before = obj.dir.joinpath("meta.yaml").read_text()
        path = authoring.write_grasp_regions(obj, regions)
        print(f"{name}: {'unchanged' if path.read_text() == before else 'written'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
