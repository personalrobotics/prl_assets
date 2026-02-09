# prl_assets

Reusable manipulation objects for Personal Robotics Lab projects.

![Assets Preview](docs/assets_preview.png)

This package provides simulation models and metadata for common manipulation objects like cans, bottles, tableware, and containers. Objects follow the `asset_manager` metadata format for compatibility with perception and planning pipelines.

## Installation

```bash
uv add prl_assets
```

For development:
```bash
git clone https://github.com/personalrobotics/prl_assets.git
cd prl_assets
uv sync --dev
```

## Quick Start

Use with `asset_manager` to load objects:

```python
from asset_manager import AssetManager
from prl_assets import OBJECTS_DIR

# Initialize asset manager with prl_assets objects
assets = AssetManager(OBJECTS_DIR)

# List available objects
print(assets.list())
# ['can', 'fuze_bottle', 'plastic_bowl', 'plastic_glass', 'plastic_plate',
#  'pop_tarts', 'recycle_bin', 'wicker_tray']

# Get path to simulation model
can_path = assets.get_path("can", "mujoco")

# Load in MuJoCo
import mujoco
model = mujoco.MjModel.from_xml_path(can_path)

# Get object metadata
meta = assets.get("can")
print(meta["dimensions"])  # [0.066, 0.066, 0.123]
print(meta["mass"])        # 0.05 kg

# Find objects by category
containers = assets.by_category("container")
```

## Available Objects

| Object | Description | Collision |
|--------|-------------|-----------|
| `can` | Aluminum soda/beer can | cylinder |
| `fuze_bottle` | Beverage bottle with texture | cylinder |
| `plastic_bowl` | Tableware bowl (hollow) | cylinder + 8 angled boxes |
| `plastic_glass` | Drinking glass | cylinder |
| `plastic_plate` | Dinner plate | cylinder |
| `pop_tarts` | Food box with texture | box |
| `recycle_bin` | Open-top recycling bin (hollow) | 5 boxes |
| `wicker_tray` | Serving tray (hollow) | 5 boxes |

## Object Structure

Each object follows a standardized directory structure:

```
objects/
└── object_name/
    ├── meta.yaml          # Object metadata
    ├── object_name.xml    # MuJoCo model
    └── *_visual.obj       # Visual mesh (optional)
```

### Metadata Format

The `meta.yaml` file contains all object properties:

```yaml
name: can
description: Standard aluminum soda/beer can
category: [container, recyclable, graspable]

# Physical properties
mass: 0.05  # kg
dimensions: [0.066, 0.066, 0.123]  # meters
material: aluminum

# Geometric properties for grasp planning
geometric_properties:
  type: cylinder
  radius: 0.033
  height: 0.123

# Simulator-specific configuration
mujoco:
  xml_path: can.xml
  scale: 1.0
  friction: [0.6, 0.005, 0.0001]

# Perception aliases for detection systems
perception:
  aliases: ["can", "soda can", "beer can"]

# Manipulation policy hints
policy:
  grasping:
    affordances: [lift, pour]
    preferred_grasp_type: side_grasp
    difficulty: easy
```

## Adding New Objects

1. Create a new directory under `src/prl_assets/objects/`
2. Add a `meta.yaml` with required fields
3. Add simulator model files (e.g., `.xml` for MuJoCo)
4. Update this README with the new object

### MuJoCo Model Requirements

Objects used with `mj_environment` must follow these conventions:

```xml
<mujoco model="object_name">
  <worldbody>
    <body name="object_name" pos="0 0 0">
      <freejoint name="object_name_joint"/>  <!-- Required for positioning -->
      <!-- geoms, sites, etc. -->
    </body>
  </worldbody>
</mujoco>
```

- **freejoint**: Required for `registry.activate()` to set object position/orientation
- **body name**: Should match the object type (becomes `{type}_{index}` when loaded)

## License

MIT License - see LICENSE file for details.
