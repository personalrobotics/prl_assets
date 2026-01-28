"""Tests for prl_assets package."""

import pytest
from pathlib import Path

from prl_assets import (
    list_objects,
    get_object_path,
    get_object_metadata,
    get_objects_by_category,
)


class TestListObjects:
    """Tests for list_objects function."""

    def test_returns_list(self):
        """list_objects should return a list."""
        result = list_objects()
        assert isinstance(result, list)

    def test_contains_expected_objects(self):
        """list_objects should contain can and recycle_bin."""
        result = list_objects()
        assert "can" in result
        assert "recycle_bin" in result

    def test_is_sorted(self):
        """list_objects should return sorted list."""
        result = list_objects()
        assert result == sorted(result)


class TestGetObjectPath:
    """Tests for get_object_path function."""

    def test_returns_path(self):
        """get_object_path should return a Path object."""
        result = get_object_path("can")
        assert isinstance(result, Path)

    def test_path_exists(self):
        """get_object_path should return an existing file."""
        result = get_object_path("can")
        assert result.exists()

    def test_xml_extension(self):
        """MuJoCo path should have .xml extension."""
        result = get_object_path("can", simulator="mujoco")
        assert result.suffix == ".xml"

    def test_invalid_object_raises(self):
        """get_object_path should raise for invalid object."""
        with pytest.raises(FileNotFoundError):
            get_object_path("nonexistent_object")

    def test_all_objects_have_paths(self):
        """All listed objects should have valid paths."""
        for name in list_objects():
            path = get_object_path(name)
            assert path.exists(), f"Path for {name} does not exist"


class TestGetObjectMetadata:
    """Tests for get_object_metadata function."""

    def test_returns_dict(self):
        """get_object_metadata should return a dictionary."""
        result = get_object_metadata("can")
        assert isinstance(result, dict)

    def test_has_required_fields(self):
        """Metadata should have required fields."""
        required_fields = ["name", "description", "category", "mass", "dimensions"]
        for name in list_objects():
            meta = get_object_metadata(name)
            for field in required_fields:
                assert field in meta, f"{name} missing field: {field}"

    def test_dimensions_format(self):
        """Dimensions should be a list of 3 numbers."""
        for name in list_objects():
            meta = get_object_metadata(name)
            dims = meta["dimensions"]
            assert isinstance(dims, list)
            assert len(dims) == 3
            assert all(isinstance(d, (int, float)) for d in dims)

    def test_mass_is_positive(self):
        """Mass should be a positive number."""
        for name in list_objects():
            meta = get_object_metadata(name)
            assert meta["mass"] > 0

    def test_invalid_object_raises(self):
        """get_object_metadata should raise for invalid object."""
        with pytest.raises(FileNotFoundError):
            get_object_metadata("nonexistent_object")


class TestGetObjectsByCategory:
    """Tests for get_objects_by_category function."""

    def test_returns_list(self):
        """get_objects_by_category should return a list."""
        result = get_objects_by_category("container")
        assert isinstance(result, list)

    def test_container_category(self):
        """Container category should include can and recycle_bin."""
        result = get_objects_by_category("container")
        assert "can" in result
        assert "recycle_bin" in result

    def test_recyclable_category(self):
        """Recyclable category should include can."""
        result = get_objects_by_category("recyclable")
        assert "can" in result

    def test_empty_category(self):
        """Nonexistent category should return empty list."""
        result = get_objects_by_category("nonexistent_category")
        assert result == []

    def test_is_sorted(self):
        """Results should be sorted."""
        result = get_objects_by_category("container")
        assert result == sorted(result)


class TestMuJoCoIntegration:
    """Integration tests with MuJoCo (requires mujoco package)."""

    @pytest.fixture
    def mujoco(self):
        """Import mujoco, skip if not available."""
        mujoco = pytest.importorskip("mujoco")
        return mujoco

    def test_can_loads_in_mujoco(self, mujoco):
        """Can XML should load in MuJoCo."""
        path = get_object_path("can")
        model = mujoco.MjModel.from_xml_path(str(path))
        assert model.ngeom > 0

    def test_recycle_bin_loads_in_mujoco(self, mujoco):
        """Recycle bin XML should load in MuJoCo."""
        path = get_object_path("recycle_bin")
        model = mujoco.MjModel.from_xml_path(str(path))
        assert model.ngeom > 0

    def test_all_objects_load_in_mujoco(self, mujoco):
        """All objects should load in MuJoCo."""
        for name in list_objects():
            path = get_object_path(name)
            model = mujoco.MjModel.from_xml_path(str(path))
            assert model.ngeom > 0, f"{name} has no geoms"
