# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""Tests for prl_assets package."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from prl_assets import OBJECTS_DIR


@pytest.fixture
def assets():
    """Create AssetManager for prl_assets.

    asset_manager is a sibling workspace package and not a prl_assets
    dependency; skip the tests that rely on it when run in isolation
    (e.g. prl_assets' standalone CI).
    """
    AssetManager = pytest.importorskip("asset_manager").AssetManager
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


def _object_dirs() -> list[Path]:
    return sorted(d for d in OBJECTS_DIR.iterdir() if (d / "meta.yaml").exists())


@pytest.mark.parametrize("obj_dir", _object_dirs(), ids=lambda d: d.name)
def test_compiled_mass_matches_meta(obj_dir: Path) -> None:
    """An object's compiled MuJoCo mass is the real mass declared in its meta.yaml.

    A geom that leaves mass unset takes MuJoCo's default density (1000 kg/m^3), even
    when it does not collide; visual meshes once made the yellow tote weigh 90 kg
    instead of 0.7 kg.
    """
    mujoco = pytest.importorskip("mujoco")
    yaml = pytest.importorskip("yaml")
    meta = yaml.safe_load((obj_dir / "meta.yaml").read_text())
    model = mujoco.MjModel.from_xml_path(str(obj_dir / meta["mujoco"]["xml_path"]))
    assert model.body_mass[1:].sum() == pytest.approx(meta["mass"], abs=1e-6)


class TestPinchTorsion:
    """Every graspable object resists twisting in a two-point pinch.

    Two small spheres, at MuJoCo's default contact settings (condim 3, as a gripper's
    finger geoms commonly are), press on opposite sides of the object along a line
    through its centre of mass, with gravity off and the solver settings MuJoCo recommends
    for grasping (an elliptic friction cone, impratio 10; with its soft default, friction
    creeps well short of its limit). A torque about that line, 10% of the contacts'
    torsional friction capacity (the torsional coefficient times the normal force), then
    acts for 0.5 s. A contact resists turning about its normal only with
    condim 4 or 6, and a contact takes the larger condim of its two geoms, so the object's
    own geoms must carry it: with condim 3 everywhere the object spins between the
    spheres as on an axle.
    """

    _PROBE_RADIUS = 0.005
    _PRESS = 0.001  # m of penetration per probe
    _SETTLE_STEPS, _TORQUE_STEPS = 100, 250
    _FRACTION = 0.1
    # Measured with the two-point pinch below: 0.42-3.04 rad with condim 3; 0.024 rad with
    # condim 4 (0.059 for the bowl, a convex-hull mesh).
    _MAX_SPIN = 0.1  # rad

    @staticmethod
    def _graspable() -> list[str]:
        out = []
        for d in sorted(OBJECTS_DIR.iterdir()):
            meta = d / "meta.yaml"
            if meta.is_file() and "graspable" in (yaml.safe_load(meta.read_text()).get("category") or []):
                out.append(d.name)
        return out

    @pytest.mark.parametrize("name", _graspable.__func__())
    def test_a_pinched_object_does_not_spin(self, name):
        mujoco = pytest.importorskip("mujoco")
        meta = yaml.safe_load((OBJECTS_DIR / name / "meta.yaml").read_text())
        spec = mujoco.MjSpec.from_file(str(OBJECTS_DIR / name / meta["mujoco"]["xml_path"]))
        spec.option.gravity = [0.0, 0.0, 0.0]
        spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        spec.option.impratio = 10.0
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        body = next(b for b in range(1, model.nbody) if model.body_jntnum[b])
        centre = data.xipos[body].copy()
        for i in range(2):
            probe = spec.worldbody.add_body(name=f"probe{i}", mocap=True, pos=centre)
            probe.add_geom(name=f"probe{i}", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[self._PROBE_RADIUS, 0, 0])
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        solids = [
            g
            for g in range(model.ngeom)
            if model.geom_bodyid[g] == body and (model.geom_contype[g] or model.geom_conaffinity[g])
        ]
        # Each probe comes in along x, through the centre of mass, until it presses
        # _PRESS into the object's collision shapes (meshes collide as convex hulls).
        for i, direction in enumerate((1.0, -1.0)):
            probe = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"probe{i}")
            mocap = model.body_mocapid[model.geom_bodyid[probe]]
            near, far = 0.0, 1.0
            for _ in range(60):
                middle = (near + far) / 2
                data.mocap_pos[mocap] = centre + direction * np.array([middle, 0.0, 0.0])
                mujoco.mj_kinematics(model, data)
                gap = min(mujoco.mj_geomDistance(model, data, probe, g, 1.0, None) for g in solids)
                near, far = (middle, far) if gap < -self._PRESS else (near, middle)
            data.mocap_pos[mocap] = centre + direction * np.array([far, 0.0, 0.0])
        mujoco.mj_forward(model, data)
        for _ in range(self._SETTLE_STEPS):
            mujoco.mj_step(model, data)
        force, normal_sum, torsional = np.zeros(6), 0.0, []
        for i in range(data.ncon):
            mujoco.mj_contactForce(model, data, i, force)
            normal_sum += force[0]
            torsional.append(float(data.contact[i].friction[2]))
        if normal_sum <= 0:
            # A thin plate or a shallow tray slips out of a pinch along this line.
            pytest.skip(f"{name}: no two-point pinch along x through its centre of mass holds it")
        torque = self._FRACTION * min(torsional) * normal_sum
        start = data.xquat[body].copy()
        for _ in range(self._TORQUE_STEPS):
            data.xfrc_applied[body, 3:] = [torque, 0.0, 0.0]
            mujoco.mj_step(model, data)
        change = np.zeros(3)
        mujoco.mju_subQuat(change, data.xquat[body], start)
        spin = float(np.linalg.norm(change))
        assert spin < self._MAX_SPIN, (
            f"{name}: spun {spin:.3f} rad under {torque:.2e} N*m with {normal_sum:.2f} N of pinch "
            f"(contact condim {sorted({int(c.dim) for c in data.contact[: data.ncon]})})"
        )


class TestBinCatches:
    """Every graspable object whose upright footprint fits inside the recycle bin's inner
    walls, dropped into the bin, comes to rest inside it, above its floor.

    The bin stands fixed on a ground plane, under MuJoCo's settings for a robot that
    drops into it: Geodude's 2 ms timestep, implicitfast, an elliptic cone, impratio 10.
    Each object is released at rest, upright, centred over the bin, its centre 0.25 m
    above the rim, the least height the recycling demo releases from. It falls about 1 m
    and lands at about 4.5 m/s. With the floor at MuJoCo's default contact softness, a
    thin object (the 28 mm gelatin box) sank through the 6 mm floor and came to rest
    beneath the bin (personalrobotics/geodude#201).
    """

    _ABOVE_RIM = 0.25  # m, the object's centre at release
    _SETTLE_STEPS = 750  # 1.5 s at 2 ms
    _SUNK = 0.001  # m: how far into the floor a resting object's lowest point may be

    @staticmethod
    def _fitting() -> list[str]:
        bin_meta = yaml.safe_load((OBJECTS_DIR / "recycle_bin" / "meta.yaml").read_text())
        inner = bin_meta["geometric_properties"]["inner_size"]
        out = []
        for name in TestPinchTorsion._graspable():
            g = yaml.safe_load((OBJECTS_DIR / name / "meta.yaml").read_text())["geometric_properties"]
            if g["type"] == "box":
                footprint = g["size"][:2]
            elif g["type"] == "cylinder":
                footprint = [2 * g["radius"]] * 2
            elif g["type"] == "bowl":
                footprint = [2 * g["outer_radius"]] * 2
            else:
                footprint = g["outer_size"][:2]
            if footprint[0] < inner[0] and footprint[1] < inner[1]:
                out.append(name)
        return out

    @pytest.mark.parametrize("name", _fitting.__func__())
    def test_a_dropped_object_rests_inside_the_bin(self, name):
        mujoco = pytest.importorskip("mujoco")
        bin_meta = yaml.safe_load((OBJECTS_DIR / "recycle_bin" / "meta.yaml").read_text())
        outer = bin_meta["geometric_properties"]["outer_size"]
        inner = bin_meta["geometric_properties"]["inner_size"]
        floor = outer[2] - inner[2]
        spec = mujoco.MjSpec()
        spec.option.timestep = 0.002
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        spec.option.impratio = 10.0
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1])
        container = mujoco.MjSpec.from_file(str(OBJECTS_DIR / "recycle_bin" / "recycle_bin.xml"))
        stand = spec.worldbody.add_frame()
        stand.attach_body(container.body("recycle_bin"), "bin/", "")
        for joint in list(spec.joints):
            if joint.name.startswith("bin/"):
                spec.delete(joint)
        meta = yaml.safe_load((OBJECTS_DIR / name / "meta.yaml").read_text())
        source = mujoco.MjSpec.from_file(str(OBJECTS_DIR / name / meta["mujoco"]["xml_path"]))
        top = next(b for b in source.worldbody.bodies)
        drop = spec.worldbody.add_frame(pos=[0.0, 0.0, outer[2] + self._ABOVE_RIM])
        dropped = drop.attach_body(top, "obj/", "")
        dropped.pos = [0.0, 0.0, 0.0]
        model = spec.compile()
        data = mujoco.MjData(model)
        for _ in range(self._SETTLE_STEPS):
            mujoco.mj_step(model, data)
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, dropped.name)
        x, y, _ = data.xpos[body]
        assert abs(x) <= inner[0] / 2 and abs(y) <= inner[1] / 2, (x, y)
        # The lowest corner of its collision geoms' bounding boxes: on the floor, not in it.
        corners = np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)
        lowest = min(
            float(
                (
                    data.geom_xpos[g][:, None]
                    + data.geom_xmat[g].reshape(3, 3)
                    @ (model.geom_aabb[g][:3][:, None] + (corners * model.geom_aabb[g][3:]).T)
                )[2].min()
            )
            for g in range(model.ngeom)
            if model.geom_bodyid[g] == body and model.geom_contype[g]
        )
        assert floor - self._SUNK <= lowest < outer[2], (
            f"its lowest point rests at {lowest:.4f} m, the floor's top at {floor:.4f} m"
        )
