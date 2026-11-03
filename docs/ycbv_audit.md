# YCB-Video overlap audit

Before the YCB-V collection was added, four catalog objects looked like YCB-V objects
by product name, and two others shared a category with one. This records what was
checked, what was decided, and what moved. The measurements are in
[`ycbv_audit.json`](ycbv_audit.json); the frame contract is in [the schema](schema.md).

## Method

`scripts/audit_ycbv.py` reads the BOP YCB-V models from the pinned
`ycbv_models.zip` (SHA-256 verified before use), converts each reference model from
millimetres to metres, maps it into the asset frame by the asset's declared
`frames.T_asset_bop`, and compares it with the asset's visual mesh by symmetric
surface distance over 20,000 samples per mesh, and by texture colour at the nearest
surface points. It then runs ICP from the declared transform and reports the rotation
and translation of the best fit relative to it.

An asset passes when the 99th-percentile distance is at most 3 mm, the median colour
difference is at most 12 (mean absolute RGB difference, 0–255), and the best fit lies
within 0.5° and 1 mm of the declared transform. Two remeshings of one scan differ by
about a millimetre.

Geometry alone does not determine the frame of a nearly symmetric box. Turned 180°
about z, `gelatin_box` measures 1.46 mm and `spam_can` 1.61 mm, both inside the
distance limit; their printed labels differ, at colour 34.7 and 26.7. The audit
therefore measures each declared transform's three 180° flips as well and records them
under `flipped` in the JSON: every flip fails on colour (26.7–61.7, against 1.7–5.3 for
the declared frames). Declaring the flipped frame for `gelatin_box` fails the audit.
Declaring the identity for `sugar_box` gives a 68 mm 99th percentile and a 90° residual,
and labelling `cracker_box` as BOP object 7 gives 92 mm.

To reproduce (writes `artifacts/ycbv_audit.json`; `--record` also replaces the
current full-catalog `docs/ycbv_catalog.json`, and only if all 21 assets pass; the
earlier overlap artifact remains unchanged):

```bash
uv run --extra dev python scripts/audit_ycbv.py [--record]
```

## Decisions

| Catalog object | BOP id / YCB name | p99 distance | Colour | Best fit vs. declared | Decision |
| --- | --- | --- | --- | --- | --- |
| `cracker_box` | 2 / `003_cracker_box` | 2.26 mm | 2.7 | 0.015°, 0.02 mm | Same object, identity frame |
| `sugar_box` | 3 / `004_sugar_box` | 1.52 mm | 1.7 | 0.029°, 0.07 mm | Same object, rotated 90° about y |
| `gelatin_box` | 8 / `009_gelatin_box` | 0.96 mm | 4.7 | 0.008°, 0.05 mm | Same object, rotated −13.64° about z |
| `spam_can` | 9 / `010_potted_meat_can` | 1.23 mm | 5.3 | 0.008°, 0.08 mm | Same object, identity frame |
| `plastic_bowl` | not 13 / `024_bowl` | — | — | — | Different object: 172 × 76 mm, against 161 × 55 mm |
| none | 16 / `036_wood_block` | — | — | — | No catalog object; a toy block is not this object |

The four equivalent objects carry a 16,384-face remesh of the same YCB scans the BOP
models were made from. They remain the single maintained copy of each object; no second
copy is imported. Their canonical names, aliases, policy metadata, mass and collision
geometry are unchanged, and each gains `dataset`, `frames`, `symmetries` and
`provenance` sections and its YCB name as an alias. `spam_can` also gains the alias
"potted meat can".

## Findings for the collision models

The collision boxes were sized from product dimensions rather than the scans. Measured
against each scan's extent along the asset axes:

| Object | Scan extent (mm) | Collision box (mm) | Scan larger by (mm) |
| --- | --- | --- | --- |
| `cracker_box` | 71.8 × 164.0 × 213.4 | 62 × 162 × 208 | 9.8, 2.0, 5.4 |
| `gelatin_box` | 73.1 × 89.7 × 30.1 | 72 × 88 × 28 | 1.1, 1.7, 2.1 |
| `spam_can` | 102.1 × 60.1 × 83.5 | 102 × 58 × 84 | 0.1, 2.1, −0.5 |
| `sugar_box` | 176.0 × 94.2 × 49.5 | 176 × 94 × 48 | 0.0, 0.2, 1.5 |

The cracker box's visual mesh therefore extends about 5 mm beyond its collision box on
each broad face and sinks 2.7 mm into a supporting surface. A pose estimated from the
BOP model and placed in simulation keeps the visual right but contacts about 5 mm
early, which matters for grasping. Issue #30 corrected it: the cracker box and the gelatin
box now bound their scans, and every object's collision geometry is checked against its
visual mesh in CI (see [the schema](schema.md#collision-geometry)).

## Moved paths

The four objects moved into the collection. Their canonical names, and so every
`AssetManager` lookup, are unchanged; direct filesystem paths are not:

| Before | After |
| --- | --- |
| `objects/cracker_box/` | `objects/ycbv/cracker_box/` |
| `objects/sugar_box/` | `objects/ycbv/sugar_box/` |
| `objects/gelatin_box/` | `objects/ycbv/gelatin_box/` |
| `objects/spam_can/` | `objects/ycbv/spam_can/` |

No code in the workspace (geodude, mj_environment, mj_manipulator, ada_mj and the
others) referred to these paths when they moved. The repository's own scripts and tests
now find objects by their `meta.yaml` at any depth.
