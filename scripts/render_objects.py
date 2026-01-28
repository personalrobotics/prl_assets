#!/usr/bin/env python3
"""Render screenshots of all prl_assets objects.

This script renders each object and saves a PNG screenshot alongside
the object's XML file.

Usage:
    uv run python scripts/render_objects.py
"""

import sys
from pathlib import Path

# Add src to path for development
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import mujoco
import numpy as np
from PIL import Image

from asset_manager import AssetManager
from prl_assets import OBJECTS_DIR


def render_object(
    assets: AssetManager, name: str, width: int = 640, height: int = 480
) -> np.ndarray:
    """Render an object and return the image as a numpy array."""
    xml_path = assets.get_path(name, "mujoco")
    meta = assets.get(name)

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Set up offscreen rendering
    model.vis.global_.offwidth = width
    model.vis.global_.offheight = height

    renderer = mujoco.Renderer(model, width=width, height=height)

    # Step physics to settle object
    mujoco.mj_forward(model, data)

    # Calculate camera distance based on object size
    dims = meta.get("dimensions", [0.1, 0.1, 0.1])
    max_dim = max(dims)
    distance = max_dim * 4  # Camera distance relative to object size

    # Create a camera specification
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, max_dim / 2]
    camera.distance = distance
    camera.azimuth = 135
    camera.elevation = -25

    # Update scene and render with the camera
    renderer.update_scene(data, camera=camera)
    pixels = renderer.render()
    renderer.close()

    return pixels


def main():
    """Render all objects and save screenshots."""
    assets = AssetManager(OBJECTS_DIR)
    objects = assets.list()
    print(f"Rendering {len(objects)} objects...")

    for name in objects:
        print(f"  Rendering {name}...")

        try:
            pixels = render_object(assets, name)

            # Save alongside the XML file
            xml_path = Path(assets.get_path(name, "mujoco"))
            png_path = xml_path.parent / f"{name}.png"

            img = Image.fromarray(pixels)
            img.save(png_path)
            print(f"    Saved: {png_path}")

        except Exception as e:
            print(f"    Error: {e}")

    print("Done!")


if __name__ == "__main__":
    main()
