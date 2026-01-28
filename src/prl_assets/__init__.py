"""Reusable manipulation objects for Personal Robotics Lab projects.

This package provides simulation models and metadata for common manipulation
objects like cans, bins, and other graspable items. Objects follow the
asset_manager metadata format for compatibility with perception and planning.

Usage with asset_manager:
    >>> from asset_manager import AssetManager
    >>> from prl_assets import OBJECTS_DIR
    >>>
    >>> assets = AssetManager(OBJECTS_DIR)
    >>> print(assets.list())
    ['can', 'recycle_bin']
    >>>
    >>> can_path = assets.get_path("can", "mujoco")
    >>> model = mujoco.MjModel.from_xml_path(can_path)
"""

from pathlib import Path

__version__ = "0.1.0"

# Objects directory - pass this to AssetManager
OBJECTS_DIR = Path(__file__).parent / "objects"
