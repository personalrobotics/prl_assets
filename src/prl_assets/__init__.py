"""Reusable manipulation objects for Personal Robotics Lab projects.

This package provides MuJoCo models and metadata for common manipulation objects
like cans, bins, and other graspable items. Objects follow the asset_manager
metadata format for compatibility with perception and planning pipelines.

Example:
    >>> from prl_assets import get_object_path, list_objects
    >>> print(list_objects())
    ['can', 'recycle_bin']
    >>> can_path = get_object_path("can")
    >>> model = mujoco.MjModel.from_xml_path(str(can_path))
"""

from pathlib import Path
from typing import Optional
import yaml

__version__ = "0.1.0"

# Objects directory is inside the package
OBJECTS_DIR = Path(__file__).parent / "objects"


def list_objects() -> list[str]:
    """List all available objects.

    Returns:
        List of object names that can be loaded.

    Example:
        >>> list_objects()
        ['can', 'recycle_bin']
    """
    objects = []
    for obj_dir in OBJECTS_DIR.iterdir():
        if obj_dir.is_dir() and (obj_dir / "meta.yaml").exists():
            objects.append(obj_dir.name)
    return sorted(objects)


def get_object_path(name: str, simulator: str = "mujoco") -> Path:
    """Get the path to an object's simulation model.

    Args:
        name: Object name (e.g., "can", "recycle_bin")
        simulator: Simulator type ("mujoco" or "isaac")

    Returns:
        Path to the model file.

    Raises:
        FileNotFoundError: If the object or model doesn't exist.

    Example:
        >>> path = get_object_path("can")
        >>> model = mujoco.MjModel.from_xml_path(str(path))
    """
    obj_dir = OBJECTS_DIR / name

    if not obj_dir.exists():
        available = list_objects()
        raise FileNotFoundError(
            f"Object '{name}' not found. Available objects: {available}"
        )

    meta_path = obj_dir / "meta.yaml"
    if not meta_path.exists():
        raise FileNotFoundError(f"No meta.yaml found for object '{name}'")

    with open(meta_path) as f:
        meta = yaml.safe_load(f)

    sim_config = meta.get(simulator, {})
    if not sim_config:
        raise FileNotFoundError(
            f"No {simulator} configuration for object '{name}'"
        )

    path_key = "xml_path" if simulator == "mujoco" else "usd_path"
    rel_path = sim_config.get(path_key)
    if not rel_path:
        raise FileNotFoundError(
            f"No {path_key} specified for object '{name}'"
        )

    full_path = obj_dir / rel_path
    if not full_path.exists():
        raise FileNotFoundError(f"Model file not found: {full_path}")

    return full_path


def get_object_metadata(name: str) -> dict:
    """Get the full metadata for an object.

    Args:
        name: Object name (e.g., "can", "recycle_bin")

    Returns:
        Dictionary containing all object metadata.

    Example:
        >>> meta = get_object_metadata("can")
        >>> print(meta["dimensions"])
        [0.066, 0.066, 0.123]
    """
    obj_dir = OBJECTS_DIR / name

    if not obj_dir.exists():
        available = list_objects()
        raise FileNotFoundError(
            f"Object '{name}' not found. Available objects: {available}"
        )

    meta_path = obj_dir / "meta.yaml"
    with open(meta_path) as f:
        return yaml.safe_load(f)


def get_objects_by_category(category: str) -> list[str]:
    """Get all objects that belong to a category.

    Args:
        category: Category name (e.g., "graspable", "recyclable", "container")

    Returns:
        List of object names in that category.

    Example:
        >>> get_objects_by_category("recyclable")
        ['can']
    """
    matching = []
    for name in list_objects():
        meta = get_object_metadata(name)
        categories = meta.get("category", [])
        if category in categories:
            matching.append(name)
    return sorted(matching)
