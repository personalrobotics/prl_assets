# Object metadata schema

This document specifies the sections of `meta.yaml` that prl_assets owns beyond those
asset_manager interprets. asset_manager's
[schema](https://github.com/personalrobotics/asset_manager/blob/main/docs/schema.md)
owns discovery, `name`, `category`, the `mujoco` path, and `perception.aliases`; it
stores every other section unchecked, so prl_assets' tests validate the sections below.

## Collections

An object is a directory holding a `meta.yaml`. Objects may be grouped in collection
directories, which hold no `meta.yaml` of their own. `prl_assets.OBJECTS_DIR` is the
whole catalog, collections included; `prl_assets.YCBV_OBJECTS_DIR` is the YCB-Video
collection alone. Canonical names are unique across the whole catalog, and a
collection does not prefix them.

## Collision geometry

MuJoCo simulates an object's collision geoms, not the mesh it renders, so the two must
agree. `prl_assets.authoring.check` measures each compiled model, and
`tests/test_collision_fit.py` runs it on every object in CI. The visual mesh is the
geoms in group 2; the collision geoms are those with `contype` or `conaffinity` set, in
group 3. A mesh collision geom counts as its convex hull, as MuJoCo collides it. An
object with no visual mesh renders its collision geoms, which are then its shape, so only
`resting` and the sizes below apply to it.

| Check | Default limit |
| --- | --- |
| `visual_outside`: 99th percentile of the visual surface's distance outside the collision geoms (undersized) | 3 mm |
| `collision_excess`: 99th percentile of the collision surface's distance outside the visual mesh's convex hull (oversized) | 3 mm |
| `sink`: how far the visual mesh reaches below the collision geometry's bottom | 1 mm |
| `resting`: the collision geometry's bottom at the model file's pose, which must rest on z = 0 | 1 mm |
| `floor_gap`: for open objects, a 1 cm sphere dropped on the axis comes to rest this far above the visual floor: the larger of its height above the visual surface beneath its resting point and above the floor beneath the drop point. A sphere that comes to rest off the object (rolled off a filled top) fails | 3 mm |

The model must also have a free joint on its top-level body (unless `category` includes
`fixture`), and its compiled mass must equal `mass`.

Legacy readers still use `geometric_properties` for physical placement shapes. Until
those readers migrate, physical `dimensions` and legacy sizes must match the collision
geometry's extent along the asset axes within
2 mm: `dimensions`, `geometric_properties.size` and `outer_size` (full x, y, z extents),
and `radius` or `outer_radius` (as a diameter in x and y, with `height` in z).

### `geometric_properties.open`

`open: true` (a boolean) declares an interior that things are put into: a cup, bowl,
plate, tote, tray or bin. Such an object with a visual mesh is checked for `floor_gap`,
which a solid or filled interior fails by roughly its depth.

### `geometric_properties.collision_tolerance_mm`

An object may widen a limit where a primitive deliberately departs from the scan, for
example a box with flat faces for a scanned box whose edges the scanner rounded. The
reason is required:

```yaml
geometric_properties:
  type: box
  collision_tolerance_mm:
    collision_excess: 10.0
    reason: "the scan rounds the box's edges; the box keeps flat faces for resting and face grasps"
```

Keys are the check names above and values are millimetres. A tolerance without a
reason, with an unknown key, or with a value that is not a non-negative number is an
error.

## YCB-Video identity: `dataset.ycbv`

Every object in `objects/ycbv/` declares which YCB-Video object it is:

```yaml
dataset:
  ycbv:
    bop_id: 2                 # BOP ycbv object id, 1-21
    ycb_name: 003_cracker_box # the original YCB model name
```

`bop_id` and `ycb_name` follow the BOP release's numbering, which is contiguous and
differs from the numeric prefixes of the YCB names. Each id names exactly one object in
the catalog. A detector's class index is not an asset identity; map it to `bop_id`
at the perception boundary. Every YCB-V object also lists its `ycb_name` among its
`perception.aliases`, so `resolve_alias("003_cracker_box")` returns its canonical name.

## Frames: `frames.T_asset_bop`

Three frames are involved:

- **BOP model frame.** The frame of the reference model `models/obj_<bop_id>.ply` in the
  BOP release, in millimetres. Pose estimators trained on YCB-V report
  `T_camera_bop` in this frame.
- **Asset frame.** The frame in which the object's visual mesh and collision geometry
  are expressed, in metres. It is the MuJoCo body frame: the model's geoms carry no
  offset from the body.
- **Resting placement.** The body's `pos` (and `quat`, if any) in the model file only
  places the object in the world when the file is loaded on its own, resting on
  `z = 0`. It is not part of the asset frame, and a scene that sets the free joint's
  pose replaces it.

`T_asset_bop` is a 4×4 homogeneous rigid transform, stored as a list of four rows,
mapping BOP-frame points, **after conversion from millimetres to metres**, into the
asset frame:

```text
p_asset         = T_asset_bop @ (p_bop_mm / 1000)
T_camera_asset  = T_camera_bop_m @ inverse(T_asset_bop)
```

where `T_camera_bop_m` is the detector's pose with its translation converted to metres.
Scale never enters `T_asset_bop`; its rotation block is orthonormal with determinant 1.

The four objects that predate the collection keep their frames, so existing scenes
behave as before:

| Object | `T_asset_bop` rotation | Why |
| --- | --- | --- |
| `cracker_box` | identity | |
| `spam_can` | identity | |
| `sugar_box` | 90° about y | laid on its largest face |
| `gelatin_box` | −13.64° about z | edges aligned with the asset axes |

Translations are zero: the asset and BOP origins coincide, at the BOP model origin near
the geometric centre. New objects in the collection use the identity.

`scripts/audit_ycbv.py` checks each declared transform against the BOP model, by shape
and by texture. Current full-catalog results are in `docs/ycbv_catalog.json`; see
[the audit](ycbv_audit.md).

## Symmetries: `symmetries`

```yaml
symmetries:
  frame: bop      # the BOP model frame, millimetres
  discrete: []    # 4x4 transforms, as 16 row-major numbers each
  continuous: []  # {axis: [x, y, z], offset: [x, y, z]}
```

The entries are copied unchanged from the BOP release's `models_info.json`
(`symmetries_discrete`, `symmetries_continuous`) and stay in its frame and units. Empty
lists mean the release declares no symmetry. A consumer working in the asset frame
converts a discrete symmetry `S` (translation in millimetres) as
`T_asset_bop @ S_m @ inverse(T_asset_bop)`, where `S_m` has its translation divided by
1000.

## Provenance: `provenance`

```yaml
provenance:
  geometry: where the shipped mesh came from, and how it was produced
  reference_model: the BOP file this object is checked against
  license: SPDX identifier of the mesh and texture license
  attribution: where the full notice lives
```

The full notices for the YCB-V collection are in `objects/ycbv/NOTICE.md`, which ships
with the package. The package's MIT license does not relicense object data.

## Imported YCB-V geometry and physical properties

New objects retain the BOP model frame and use `T_asset_bop = identity`; their
visual geometry is converted from millimetres to metres without recentering or
simplification. Texture coordinates are retained and PNG textures are resampled
to 1024 by 1024. Their model file's body position supplies the resting offset.

The importer validates the source archive before parsing, reads only fixed model
and texture members, and rejects unsupported external resource paths in retained
objects before compiling them. Existing YCB-V identities are never overwritten.
Staging metadata is named `.meta.yaml.pending`; it becomes `meta.yaml` only after
all requested new objects validate. A failed publication removes only newly created
objects and preserves existing ones. The POSIX authoring tool uses a cooperative
collection lock; individual assets are published atomically, not an entire batch
snapshot for simultaneous readers.

`provenance` records the archive/member hashes, decomposition version and settings,
reference mass source, and the assumptions behind inertia and friction. New
compound collision models distribute mass by convex-part volume; overlapping
parts are counted. This is a uniform-density inertia estimate, not a measurement.
Their box-shaped `geometric_properties` is a deprecated legacy approximation whose
dimensions match the collision bounds. New grasp readers use only `grasp_regions`;
the importer and catalog-wide migration remain follow-up work after consumers migrate.
This legacy approximation is not a claim of solid box collision geometry. Concave collision pieces remain separate mesh geoms in MuJoCo.

Collision models are as coarse as their interactions allow. A closed, nearly convex
object is one convex hull. Each decomposed object uses the coarsest CoACD setting in
its recipe that passes `authoring.check`, including the floor probe of an open
interior, keeps the reference silhouette's apertures open (below), and bridges the
scan's concavities by at most about 13 mm at the 99th percentile. Collision cost grows
with the parts in contact, so a finer decomposition than this buys nothing a task can
use and multiplies the cost of every contact-rich scene. CoACD's output depends on the
platform; the committed colliders are reproduced on Linux x86-64 with the locked
dependencies, and `provenance.conversion` records the settings.

## Installed asset verification

The distributed wheel contains the visual and collision resources, identity/frame
metadata, source notices and both source-audit artifacts. The base package imports
without simulator or scientific authoring dependencies. `ycbv` supplies those optional
dependencies; `asset_manager` remains a separate consumer-owned package.

For rendered frame verification, world coordinates equal the BOP model coordinates in
metres. The reference mesh is converted from the pinned PLY; the asset's compiled body
is placed at `inverse(T_asset_bop)`, replacing its initial resting placement. The same
camera renders both. Four views at 320 by 240 pixels must each have foreground
silhouette intersection-over-union of at least 0.97. Camera position, vertical field of
view, look-at point and azimuth/elevation are recorded with every result. Foreground is
depth below 99% of the far clipping distance. The tolerance allows the four retained
remeshings' edge differences; the separate texture audit protects orientation of nearly
symmetric objects.

A visible aperture is an enclosed background region in the reference silhouette,
eroded by two pixels to avoid boundary discretization. When at least 25 pixels remain,
at most 10% may be occupied by the rendered convex collision pieces. A deliberately
solid-hull mug must violate this condition. This checks visible through-openings in the
chosen views. Open interiors additionally use the physical floor probe specified above.
The capped pitcher scan declares no open interior.

Per-object verification also requires finite positive compiled mass and principal
inertia, finite nonnegative friction, a loaded texture, the collision/metadata contract,
and resting collision clearance within 1 mm of the support. The JSON artifact records
the physical parameters, source/frame measurements, projection/aperture measurements
and file hashes. A successful overall artifact requires every object and the seeded
scene to pass and the filled-handle negative control to be detected. README owns the
reproduction commands, scene settings and interpretation of rendered artifacts.

## Grasp regions (version 1)

`grasp_regions` describes robot-neutral primitive approximations used to generate grasp
TSRs. It does not describe collision geometry, a support, a usable container interior,
a stable placement shape or a physical footprint. A region need not enclose the object
or fit its collider, and it certifies neither collision freedom nor grasp feasibility.
Hand/aperture, finger geometry, TCP offsets, approach distances and TSR tolerances remain
in the explicitly selected hand/profile/task configuration.

```yaml
grasp_regions:
  version: 1
  regions:
    - name: body
      primitive: cylinder
      radius: 0.039
      height: 0.074
      T_asset_region: [[1,0,0,-0.012], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
      strategies: [side, top]
```

The section requires exactly `version` (integer 1) and `regions` (a list, possibly
empty). A missing section means no regions. An explicit null or malformed section is
an error. Region names are unique and match `[a-z][a-z0-9_]*`. Each region requires
`name`, `primitive`, and exactly its primitive's parameters:

| Primitive | Parameters (metres) | Default strategies |
| --- | --- | --- |
| cylinder | positive `radius`, `height` | side, top, bottom |
| box | positive `size: [x,y,z]`, full extents | top, bottom, face_x, face_y |
| sphere | positive `radius` | all |
| torus | positive `major_radius`, `minor_radius`, minor < major | side, span |

Optional `strategies` is a nonempty list of distinct members of the corresponding
vocabulary. Names correspond to existing sstsr factories, including
`grasp_cylinder_side`, `grasp_box_face_x`, `grasp_sphere` and `grasp_torus_span`.
This is a primitive-description schema, separate from sstsr's serialized TSR templates.

Every primitive has its origin at its geometric centre, with z the cylinder/torus axis
and the box's local z direction. Optional `T_asset_region` defaults to identity. It is
a row-major 4 by 4 rigid transform mapping region points into the asset frame, with
translation in metres, orthonormal rotation and determinant +1. Validation tolerances
are 1e-6 for rotation/determinant and 1e-8 for the homogeneous bottom row.

`T_world_region = T_world_asset @ T_asset_region`. sstsr's cylinder and box **grasp**
factories use a bottom-centre origin, so `bottom_face_frame()` returns
`T_asset_region @ Tz(-height/2)` (`size[2]` for a box). For sphere/torus it returns the
unchanged centre frame. Compose `T_world_asset @ region.bottom_face_frame()` once;
do not subtract another half-height. StablePlacer's placement convention is centred
and must not use this grasp-factory shift.

`prl_assets.grasp_regions.parse(meta)` takes an already loaded metadata dictionary,
uses only NumPy and the standard library, and returns immutable `GraspRegion` records.
Unknown keys, unsupported primitives/strategies, bools, numeric strings, nonfinite or
nonpositive parameters, duplicate names/strategies, malformed or nonrigid transforms,
and missing/unsupported versions raise `GraspRegionError` naming the object, region
and field as applicable. No malformed value is coerced or ignored. Returned frame and
bounds arrays can be edited without mutating the record.

Authoring checks parse this section before model construction and validate each region's
extent along its own axes against the visual mesh's vertices along those axes plus 10 mm
on every side (a turned box is not held to the corners of an axis-aligned box it does
not fill). This is a coarse unit/frame plausibility guard, not a geometric fit,
alignment certificate or grasp-success claim. Collision bounds are not used for this check.
A declared region requires a visual mesh. Physical and legacy dimensions retain their
separate 2 mm collision-extent check.

`geometric_properties` remains a deprecated **legacy view** for legacy readers only.
It may coexist with the new section during migration, but new readers never merge
fields or fall back to legacy geometry. Placement and containment are described by
`placement` (below), not by grasp regions. Consumer migration (ssgrasping#46 and
geodude#260), catalog/importer migration and eventual legacy-read retirement follow in
that order under prl_assets#36/#37.

Which objects declare regions (prl_assets#40):

- The mug: a `body` cylinder and a framed `handle` box, measured from its scan.
- **Legacy parity.** The objects in `tests/test_grasp_regions.py`'s `LEGACY_PARITY`
  (the nine recycling kinds and three other boxes), whose frame is centred on their
  collision geometry (within 1 mm) and whose legacy grasp primitive is their shape. Each
  declares one region, `body`: its legacy `geometric_properties` primitive at the object
  frame, with the grasp families the legacy grasp path uses (a cylinder `side`; a box
  `face_x`, `face_y`, `top`, `bottom`). A consumer switching its grasping to them makes
  the same grasp candidates; the test holds them to it and writes
  `artifacts/grasp_region_parity.json`. Parity is about grasp candidates only: a
  consumer that treats declaring regions as a reason to stage or place an object
  differently (Geodude stages such objects from their physical data) will see those
  change. A collider rebuild that would change such an object's `dimensions` is refused
  until its regions are re-authored.
- Fixtures (`category` includes `fixture`) declare none: they are never picked up.
- **Measured by recipe.** The objects in `prl_assets.grasp_recipes.GRASP_RECIPES`. A
  recipe names each region's primitive and strategies and the part of the visual model it
  is measured from (a box in the asset frame, or in that frame turned about its z, so
  that a part on a diagonal, such as a clamp's handle, is selected apart from what it
  joins; outliers removed only by an explicit, reasoned exclusion). `prl_assets.authoring.measure_grasp_regions` measures every number
  from the visual mesh's samples and vertices, and `scripts/author_grasp_regions.py`
  writes the section. A recipe object's asset z must be vertical at its model file's
  pose. The fits:
  - a `cylinder` stands `upright`, about the asset's vertical through the smallest
    circle enclosing the points' xy, or is `lying`, along the long side of their xy's
    minimum-area rectangle (its yaw in (-90°, 90°]), through the smallest circle
    enclosing them across it; the region's z is the cylinder's axis, and lying, its x is
    horizontal and its y up;
  - a `box` is upright, from the minimum-area rectangle of the points' xy hull: x its
    shorter side, the yaw in (-90°, 90°]; when the sides differ by less than 2 mm, x is
    the side nearer asset x and the yaw is in (-45°, 45°], so unchanged inputs never
    flip a near-square box by a quarter turn;
  - every selected point lies inside its region within 10 µm, the rounding of written
    values; sizes are rounded up so that a written region still covers.

  `authoring.check` re-measures a recipe object and fails when its section is missing,
  names other regions, or differs from the measurement by more than 2 mm or 0.5°. A
  region of interest is chosen from the object's geometry, never from how a robot fared
  with it. `tests/test_grasp_fitting.py` recovers known primitives with the fitters and
  writes `artifacts/grasp_regions_analytic.json`, which also records each catalog
  region's evidence.
  Parts of an object are regions of their own: the bleach bottle's cap beside its body;
  the banana's middle; the drill's, clamps' and pitcher's handles; the scissors' shank.
- The containers (bowl, plate, tray, tote: rim and handle regions) get recipes in
  follow-up work.

## Placement (version 1)

`placement` describes a container's usable opening and the surfaces other objects may
be placed on: what a placement planner cannot work out from collision geometry alone. It
is separate from grasp regions and collision geometry, and nothing in it is derived from
a primitive type. How an object rests is not declared: sstsr's `StablePlacer` computes
an object's stable rests, their heights and footprints from its collision geometry and
centre of mass, and the model file's pose is the one it is modelled upright in.
Every field is measured from the compiled model by `prl_assets.authoring`, written by
`scripts/author_placement.py`, and checked against the model by `authoring.check`.

```yaml
placement:
  version: 1
  openings:
  - name: top
    shape: rectangle
    size: [0.238, 0.238]
    depth: 0.757
    T_asset_opening: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.797], [0, 0, 0, 1]]
  supports:
  - name: top
    shape: rectangle
    size: [1.852, 0.742]
    T_asset_support: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.735], [0, 0, 0, 1]]
```

The section requires `version` (integer 1) and `openings` or `supports` lists, together
naming at least one entry. A missing section means none is declared; an explicit
null or malformed section is an error. Names are unique within each list and match
`[a-z][a-z0-9_]*`. Lengths are metres; transforms are row-major 4 by 4 rigid transforms
validated as in grasp regions (1e-6 rotation and determinant, 1e-8 bottom row).

| Entry | Fields | Meaning |
| --- | --- | --- |
| opening | `shape`, `size` or `radius`, `depth`, optional `T_asset_opening` | A rectangle (full `size: [x, y]`) or circle (`radius`) centred on the rim plane, z pointing out of the container, with the interior floor `depth` below it along -z. A circle's radius is the largest circle inside the rim. |
| support | `shape: rectangle`, `size`, optional `T_asset_support` | A rectangular surface centred on itself, z its outward normal. Version 1 measures a support as its top's x-y bounds, so it has rectangles only. |

Omitted opening and support frames are the identity. The object frame need not be
centred on anything: a YCB-V object keeps its BOP frame, and its opening frame says
where it opens relative to that frame.

**Measurement** (`authoring.measure_opening`, `measure_support`), at the model file's
pose, which rests the object on z = 0. Openings and supports face up there, and are
measured along the asset's first horizontal axis (x) and the vertical (z); an asset with
no axis vertical at that pose is refused, since its axes name no rectangle's sides.

- An opening's rim is the top of the collision geometry. Rays cast in the collision
  geometry 3 mm below the rim centre the interior between opposing walls, give a
  rectangle's wall-to-wall width along x and y, or a circle's shortest of 64 rays, and
  the floor below the centre gives the depth. A shape that does not describe the opening
  is refused rather than approximated: a rectangle any of whose 64 rays ends more than
  2 mm from its outline, and a circle whose longest ray is more than 1.25 times its
  shortest (a square's is 1.41; the catalog's least round, the scanned YCB-V mug, is
  1.18). So is a non-convex collision mesh: MuJoCo collides it as its hull, while a ray
  meets its triangles.
- A support is the top face: the highest collision point, and the bounds of the
  collision vertices within 0.1 mm of it, so a geom that flares below the top adds
  nothing. A top whose outline falls short of its bounding rectangle by more than 2 mm
  along its perimeter (a round or bevelled top) is refused.

Analytic models with known openings and tops on a turned, off-centre body are the
measurers' oracle (`tests/test_placement.py`).

`check` fails an object whose declared opening or support differs from its measurement
by more than 2 mm, or 1e-3 in a rotation entry, and an open object
(`geometric_properties.open`) that declares no opening. Rerun
`scripts/author_placement.py <name>` after a collider change; it rewrites the section only, and a rerun on an unchanged object changes nothing.

`prl_assets.placement.parse(meta)` uses only NumPy and the standard library and returns
immutable `Placement`, `Opening` and `Support` records. Unknown keys (`rests` among
them), bools, numeric strings, nonfinite or nonpositive sizes, an empty section, the
wrong parameter for a shape, duplicate names, nonrigid transforms and a missing or unsupported
version raise `PlacementError` naming the object, entry and field.

**Legacy policy.** `geometric_properties.open`, `outer_size`, `inner_size`,
`wall_thickness` and `policy.placement.drop_zone_margin` remain for legacy readers until
they migrate (geodude#260). New readers read only `placement` and never fall back to
them. The recycle bin's opening equals its legacy `inner_size` and `outer_size` exactly
(`tests/test_placement.py`); the tray's and tote's legacy `wall_thickness` describes
their floors, not their tapered rims, and `artifacts/placement.json` records the
difference. Which objects declare what:

- Openings: recycle_bin, yellow_tote and wicker_tray (rectangles); plastic_bowl,
  ycb_bowl, plastic_glass, plastic_plate and mug (circles).
- Supports: ada_table's top.
