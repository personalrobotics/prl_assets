"""Tests for prl_assets package."""

import pytest
from pathlib import Path

from prl_assets import OBJECTS_DIR


@pytest.fixture
def assets():
    """Create AssetManager for prl_assets."""
    from asset_manager import AssetManager
    return AssetManager(OBJECTS_DIR)


class TestObjectsDir:
    """Tests for OBJECTS_DIR."""

    def test_exists(self):
        """OBJECTS_DIR should exist."""
        assert OBJECTS_DIR.exists()

    def test_is_directory(self):
        """OBJECTS_DIR should be a directory."""
        assert OBJECTS_DIR.is_dir()

    def test_contains_objects(self):
        """OBJECTS_DIR should contain object subdirectories."""
        subdirs = [d for d in OBJECTS_DIR.iterdir() if d.is_dir()]
        assert len(subdirs) >= 2


class TestAssetManager:
    """Tests using asset_manager to load objects."""

    def test_list_returns_list(self, assets):
        """list() should return a list."""
        result = assets.list()
        assert isinstance(result, list)

    def test_list_contains_expected_objects(self, assets):
        """list() should contain can and recycle_bin."""
        result = assets.list()
        assert "can" in result
        assert "recycle_bin" in result

    def test_list_is_sorted(self, assets):
        """list() should return sorted list."""
        result = assets.list()
        assert result == sorted(result)

    def test_get_path_exists(self, assets):
        """get_path() should return an existing file."""
        result = assets.get_path("can", "mujoco")
        assert Path(result).exists()

    def test_get_path_xml_extension(self, assets):
        """MuJoCo path should have .xml extension."""
        result = assets.get_path("can", "mujoco")
        assert result.endswith(".xml")

    def test_get_returns_dict(self, assets):
        """get() should return a dictionary."""
        result = assets.get("can")
        assert isinstance(result, dict)

    def test_get_has_required_fields(self, assets):
        """Metadata should have required fields."""
        required_fields = ["name", "description", "category", "mass", "dimensions"]
        for name in assets.list():
            meta = assets.get(name)
            for field in required_fields:
                assert field in meta, f"{name} missing field: {field}"

    def test_dimensions_format(self, assets):
        """Dimensions should be a list of 3 numbers."""
        for name in assets.list():
            meta = assets.get(name)
            dims = meta["dimensions"]
            assert isinstance(dims, list)
            assert len(dims) == 3
            assert all(isinstance(d, (int, float)) for d in dims)

    def test_mass_is_positive(self, assets):
        """Mass should be a positive number."""
        for name in assets.list():
            meta = assets.get(name)
            assert meta["mass"] > 0

    def test_by_category_container(self, assets):
        """Container category should include can and recycle_bin."""
        result = assets.by_category("container")
        assert "can" in result
        assert "recycle_bin" in result

    def test_by_category_recyclable(self, assets):
        """Recyclable category should include can."""
        result = assets.by_category("recyclable")
        assert "can" in result


class TestMuJoCoIntegration:
    """Integration tests with MuJoCo."""

    @pytest.fixture
    def mujoco(self):
        """Import mujoco, skip if not available."""
        return pytest.importorskip("mujoco")

    def test_can_loads_in_mujoco(self, assets, mujoco):
        """Can XML should load in MuJoCo."""
        path = assets.get_path("can", "mujoco")
        model = mujoco.MjModel.from_xml_path(path)
        assert model.ngeom > 0

    def test_recycle_bin_loads_in_mujoco(self, assets, mujoco):
        """Recycle bin XML should load in MuJoCo."""
        path = assets.get_path("recycle_bin", "mujoco")
        model = mujoco.MjModel.from_xml_path(path)
        assert model.ngeom > 0

    def test_all_objects_load_in_mujoco(self, assets, mujoco):
        """All objects should load in MuJoCo."""
        for name in assets.list():
            path = assets.get_path(name, "mujoco")
            model = mujoco.MjModel.from_xml_path(path)
            assert model.ngeom > 0, f"{name} has no geoms"
