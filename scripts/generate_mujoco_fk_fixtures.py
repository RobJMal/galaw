#!/usr/bin/env python3
# /// script
# dependencies = ["mujoco>=3.0", "numpy"]
# ///
"""
Generates FK fixture JSON files for the MuJoCo cross-reference tests.

For each MJCF robot, loads the model with the MuJoCo Python bindings, then
runs FK for a fixed set of named configurations (zero, upper limits, lower
limits, mixed limits, and N random configs) and writes the resulting body
poses to JSON.

The JSON files are committed to the repo. The Rust tests in
  crates/galaw/tests/mjcf_fk_mujoco.rs
read them as static ground truth -- no Python or MuJoCo needed at test time.

Regenerate when MJCF files or the fixture format change:
  uv run scripts/generate_mujoco_fk_fixtures.py
  # or: pip install mujoco numpy && python3 scripts/generate_mujoco_fk_fixtures.py
"""

import json
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
CRATE_ROOT = REPO_ROOT / "crates" / "galaw"
FIXTURE_DIR = CRATE_ROOT / "tests" / "fixtures" / "mujoco_fk"

N_RANDOM = 100
RNG_SEED = 42

# (fixture_stem, mjcf_path_relative_to_crate_root)
ROBOTS = [
    ("spot",              "assets/mjcf/third_party/Boston-Dynamics_Spot/spot.xml"),
    ("panda",             "assets/mjcf/third_party/Franka-Emika_Panda/panda.xml"),
    ("right_shadow_hand", "assets/mjcf/third_party/Shadow-Robot_Hand/right_hand.xml"),
    ("g1",                "assets/mjcf/third_party/Unitree_G1/g1.xml"),
]

# MuJoCo joint type constants
MJT_FREE  = 0
MJT_BALL  = 1
MJT_SLIDE = 2
MJT_HINGE = 3


def collect_1dof_joints(model):
    """Return list of dicts for every hinge/slide joint, in model order."""
    joints = []
    for j in range(model.njnt):
        jtype = int(model.jnt_type[j])
        if jtype not in (MJT_SLIDE, MJT_HINGE):
            continue
        limited = bool(model.jnt_limited[j])
        lo, hi = model.jnt_range[j]
        joints.append({
            "name":  model.joint(j).name,
            "idx":   int(model.jnt_qposadr[j]),
            "lo":    float(lo) if limited else None,
            "hi":    float(hi) if limited else None,
        })
    return joints


def run_fk(model, data, joints_1dof, joint_values: dict) -> dict:
    """
    Set qpos from joint_values (by name), run FK, return body poses.

    Non-hinge/slide DOF (freejoint, ball) are left at their model.qpos0
    values so the base frame of floating-base robots (Spot, G1) stays at
    the MJCF-defined origin -- matching galaw's fixed-joint treatment of the
    root body transform.
    """
    data.qpos[:] = model.qpos0  # reset to reference pose for every config
    for jinfo in joints_1dof:
        name = jinfo["name"]
        if name in joint_values:
            data.qpos[jinfo["idx"]] = joint_values[name]

    mujoco.mj_kinematics(model, data)

    # Body 0 is the world body; skip it.
    poses = {}
    for i in range(1, model.nbody):
        bname = model.body(i).name
        poses[bname] = {
            "pos":       data.xpos[i].tolist(),
            "quat_wxyz": data.xquat[i].tolist(),  # MuJoCo convention: w first
        }
    return poses


def build_configs(model, data, joints_1dof, rng) -> list:
    configs = []

    # ── Zero ──────────────────────────────────────────────────────────────────
    vals = {j["name"]: 0.0 for j in joints_1dof}
    configs.append({
        "name":         "zero",
        "joint_values": vals,
        "link_poses":   run_fk(model, data, joints_1dof, vals),
    })

    # ── Upper limits ──────────────────────────────────────────────────────────
    vals = {j["name"]: j["hi"] if j["hi"] is not None else 0.0
            for j in joints_1dof}
    configs.append({
        "name":         "upper_limits",
        "joint_values": vals,
        "link_poses":   run_fk(model, data, joints_1dof, vals),
    })

    # ── Lower limits ──────────────────────────────────────────────────────────
    vals = {j["name"]: j["lo"] if j["lo"] is not None else 0.0
            for j in joints_1dof}
    configs.append({
        "name":         "lower_limits",
        "joint_values": vals,
        "link_poses":   run_fk(model, data, joints_1dof, vals),
    })

    # ── Mixed limits (even joints → lower, odd joints → upper) ────────────────
    vals = {}
    for idx, j in enumerate(joints_1dof):
        if j["lo"] is not None and j["hi"] is not None:
            vals[j["name"]] = j["lo"] if idx % 2 == 0 else j["hi"]
        else:
            vals[j["name"]] = 0.0
    configs.append({
        "name":         "mixed_limits",
        "joint_values": vals,
        "link_poses":   run_fk(model, data, joints_1dof, vals),
    })

    # ── N random ──────────────────────────────────────────────────────────────
    for i in range(N_RANDOM):
        vals = {}
        for j in joints_1dof:
            if j["lo"] is not None and j["hi"] is not None:
                vals[j["name"]] = float(rng.uniform(j["lo"], j["hi"]))
            else:
                vals[j["name"]] = 0.0
        configs.append({
            "name":         f"random_{i}",
            "joint_values": vals,
            "link_poses":   run_fk(model, data, joints_1dof, vals),
        })

    return configs


def strip_assets(mjcf_path) -> str:
    """
    Return the MJCF XML as a string with mesh/geometry elements removed.

    MuJoCo requires mesh files to be present even when only computing FK.
    Since this repo omits mesh assets, we strip everything that references
    them before loading.  The kinematic structure (bodies, joints, defaults,
    compiler settings) is left intact.

    Elements removed:
      - <asset> (mesh/material/texture definitions)
      - <geom> anywhere in the tree (reference mesh assets)
      - <site>, <camera>, <light> (not needed for FK)
    """
    # Preserve the XML declaration / encoding so ElementTree round-trips cleanly.
    ET.register_namespace("", "")
    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    # Top-level sections not needed for FK, or that reference stripped elements.
    tags_to_remove = {
        "asset",    # mesh/material/texture definitions
        "geom",     # geometry (references mesh assets)
        "site",     # sensor attachment points
        "camera",
        "light",
        "sensor",   # references sites that were stripped
        "tendon",   # may reference sites
        "contact",  # references geoms
        "equality", # constraints
        "actuator", # may reference stripped sites
        "keyframe", # ctrl sizes become inconsistent after actuator removal
    }

    def prune(node):
        for child in list(node):
            if child.tag in tags_to_remove:
                node.remove(child)
            else:
                prune(child)

    prune(root)
    return ET.tostring(root, encoding="unicode")


def generate_fixture(fixture_stem, mjcf_path) -> dict:
    xml_str = strip_assets(mjcf_path)
    model = mujoco.MjModel.from_xml_string(xml_str)
    data  = mujoco.MjData(model)

    joints_1dof = collect_1dof_joints(model)
    rng         = np.random.default_rng(RNG_SEED)
    configs     = build_configs(model, data, joints_1dof, rng)

    return {
        "robot":   fixture_stem,
        "configs": configs,
    }


def _round_floats(obj, sig=10):
    """Round every float in a nested dict/list to `sig` significant digits."""
    if isinstance(obj, float):
        return float(f"{obj:.{sig}g}")
    if isinstance(obj, dict):
        return {k: _round_floats(v, sig) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_round_floats(v, sig) for v in obj]
    return obj


def main():
    FIXTURE_DIR.mkdir(parents=True, exist_ok=True)

    for fixture_stem, rel_path in ROBOTS:
        mjcf_path = CRATE_ROOT / rel_path
        print(f"Generating {fixture_stem}...", flush=True)

        fixture   = generate_fixture(fixture_stem, mjcf_path)
        out_path  = FIXTURE_DIR / f"{fixture_stem}.json"

        with open(out_path, "w") as f:
            json.dump(_round_floats(fixture), f, indent=2)

        n_links = len(fixture["configs"][0]["link_poses"])
        print(f"  wrote {out_path}  "
              f"({len(joints_1dof_count(fixture))} joints, {n_links} bodies, "
              f"{len(fixture['configs'])} configs)")


def joints_1dof_count(fixture) -> list:
    """Helper: joint names from the first config."""
    return list(fixture["configs"][0]["joint_values"].keys())


if __name__ == "__main__":
    main()
