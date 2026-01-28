# prl_assets

Reusable manipulation objects for Personal Robotics Lab projects.

This package provides simulation models and metadata for common manipulation objects like cans, bins, and other graspable items. Objects follow a standardized metadata format for compatibility with perception and planning pipelines.

## Installation

```bash
pip install prl_assets
```

For development:
```bash
git clone https://github.com/personalrobotics/prl_assets.git
cd prl_assets
pip install -e ".[dev]"
```

## Quick Start

```python
from prl_assets import list_objects, get_object_path, get_object_metadata

# List available objects
print(list_objects())
# ['can', 'recycle_bin']

# Get path to simulation model
can_path = get_object_path("can", simulator="mujoco")

# Load in MuJoCo
import mujoco
model = mujoco.MjModel.from_xml_path(str(can_path))

# Get object metadata
meta = get_object_metadata("can")
print(meta["dimensions"])  # [0.066, 0.066, 0.123]
print(meta["mass"])        # 0.05 kg

# Find objects by category
from prl_assets import get_objects_by_category
recyclables = get_objects_by_category("recyclable")
```

## Available Objects

### can
![can](src/prl_assets/objects/can/can.png)

Standard aluminum soda/beer can for manipulation tasks.

| Property | Value |
|----------|-------|
| Dimensions | 6.6cm diameter × 12.3cm height |
| Mass | 0.05 kg |
| Material | Aluminum |
| Categories | container, recyclable, graspable |

### recycle_bin
![recycle_bin](src/prl_assets/objects/recycle_bin/recycle_bin.png)

Open-top bin for discarding recyclable items.

| Property | Value |
|----------|-------|
| Dimensions | 25cm × 25cm × 30cm |
| Mass | 0.5 kg |
| Material | Plastic |
| Categories | container, receptacle, fixture |

## Object Structure

Each object follows a standardized directory structure:

```
objects/
└── object_name/
    ├── meta.yaml          # Object metadata
    ├── object_name.xml    # MuJoCo model
    ├── object_name.usd    # Isaac Sim model (optional)
    └── object_name.png    # Preview image
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
color: [0.8, 0.1, 0.1]  # RGB
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

## API Reference

### `list_objects() -> list[str]`
Returns a list of all available object names.

### `get_object_path(name: str, simulator: str = "mujoco") -> Path`
Returns the path to an object's simulation model file.

- `name`: Object name (e.g., "can", "recycle_bin")
- `simulator`: Target simulator ("mujoco" or "isaac")

### `get_object_metadata(name: str) -> dict`
Returns the full metadata dictionary for an object.

### `get_objects_by_category(category: str) -> list[str]`
Returns all objects belonging to a specific category.

## Adding New Objects

1. Create a new directory under `src/prl_assets/objects/`
2. Add a `meta.yaml` with required fields
3. Add simulator model files (e.g., `.xml` for MuJoCo)
4. Run `python scripts/render_objects.py` to generate preview image
5. Update this README with the new object

## License

MIT License - see LICENSE file for details.
