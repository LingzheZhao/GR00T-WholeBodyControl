#!/usr/bin/env python3
"""Validate the reviewed G1 mode-5 contract and generate its C++ mirror.

The JSON file is the single source of truth for every per-joint value that can
reach LowCmd.  CMake runs this tool in ``--check`` mode so a hand-edited header
or JSON file cannot produce a controller binary.  ``--write`` is only the
mechanical regeneration step used after a reviewed contract change.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
from typing import Any
from xml.etree import ElementTree as ET


SCHEMA = "motionlcm.g1.deployment-contract.v1"
PROFILE_ID = "sonic-g1-mode5-derived-v1"
JOINT_COUNT = 29


class ContractError(ValueError):
    pass


def _reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ContractError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ContractError(f"invalid JSON numeric constant: {value}")


def _exact_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractError(f"{label} must be an integer")
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError(f"{label} must be numeric")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ContractError(f"{label} must be finite")
    return parsed


def canonical_json_sha256(value: Any) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def canonical_sha256(contract: dict[str, Any]) -> str:
    return canonical_json_sha256(contract)


def load_contract(path: Path) -> dict[str, Any]:
    try:
        contract = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicates,
            parse_constant=_reject_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ContractError(f"cannot load contract: {error}") from error
    if not isinstance(contract, dict):
        raise ContractError("contract must be a JSON object")
    _validate(contract)
    return contract


def _validate_artifact(value: Any, label: str) -> None:
    if not isinstance(value, dict) or set(value) != {
        "path_from_wbc_root",
        "size",
        "sha256",
        "role",
    }:
        raise ContractError(f"{label} artifact fields are malformed")
    path = value["path_from_wbc_root"]
    digest = value["sha256"]
    if not isinstance(path, str) or not path or Path(path).is_absolute() or ".." in Path(path).parts:
        raise ContractError(f"{label} path must stay below the WBC root")
    if _exact_int(value["size"], f"{label}.size") <= 0:
        raise ContractError(f"{label}.size must be positive")
    if not isinstance(digest, str) or len(digest) != 64 or any(
        character not in "0123456789abcdef" for character in digest
    ):
        raise ContractError(f"{label}.sha256 must be lowercase SHA-256")
    if not isinstance(value["role"], str) or not value["role"]:
        raise ContractError(f"{label}.role must be non-empty")


def _validate(contract: dict[str, Any]) -> None:
    required = {
        "schema",
        "profile_id",
        "contract_revision",
        "status",
        "vendor_attested",
        "compatibility_basis",
        "sources",
        "policy_transfer",
        "checkpoint",
        "joint_conventions",
        "joint_parameter_sha256",
        "lowcmd",
        "simulation_timing",
        "measured_state_safety",
        "takeover_control",
        "shutdown_damping",
        "models",
        "joints",
    }
    if set(contract) != required:
        raise ContractError("contract has missing or unknown top-level fields")
    if contract["schema"] != SCHEMA or contract["profile_id"] != PROFILE_ID:
        raise ContractError("unsupported contract schema or profile id")
    if _exact_int(contract["contract_revision"], "contract_revision") != 3:
        raise ContractError("unexpected contract revision")
    if contract["status"] != "experimental-cross-revision":
        raise ContractError("unexpected contract status")
    if contract["vendor_attested"] is not False:
        raise ContractError("derived contract must not claim vendor attestation")
    if contract["compatibility_basis"] != "released-mode11-policy-semantics-on-mode5-physical-plant":
        raise ContractError("unexpected compatibility basis")

    expected_sources = {
        "unitree_model": {
            "repository": "unitreerobotics/unitree_ros",
            "revision": "daadf41ee9afce8f90fdc09a98506012691fa122",
            "path": "robots/g1_description/g1_29dof_rev_1_0.xml",
            "sha256": "165fa7a5275745e45fa25e1bc78010c1055d81bb81c013b3c7bc8b9347a3b9c3",
        },
        "unitree_actuator": {
            "repository": "unitreerobotics/unitree_rl_mjlab",
            "revision": "1425b15f73bd4095f0df53709d7c389c3eb9e790",
            "path": "src/assets/robots/unitree_g1/g1_constants.py",
            "sha256": "136b59af97082a74fd3a2a4250bc2e290b3ba6a26533b1492be52114dd844c5d",
        },
        "unitree_takeover_control": {
            "repository": "unitreerobotics/unitree_sdk2",
            "revision": "9754cd153af3da471b0fe5f3aa535e426fb11db3",
            "path": "example/g1/low_level/g1_ankle_swing_example.cpp",
            "sha256": "9394824751ee4f74d252546fe4f3c20145f9148becfbb9eeb53cbeef449c6b8b",
        },
    }
    if contract["sources"] != expected_sources:
        raise ContractError("primary-source revisions differ from the reviewed pins")
    expected_transfer = {
        "checkpoint_training_mode_machine": 11,
        "physical_plant_mode_machine": 5,
        "parameter_semantics": "released-mode11-policy-values-preserved-bit-exact",
        "hip_pitch_policy_gain_basis": "7520_22-mode11",
        "hip_pitch_physical_actuator": "7520_14",
        "hip_pitch_physical_effort_limit_nm": 88.0,
    }
    if contract["policy_transfer"] != expected_transfer or any(
        type(contract["policy_transfer"][key]) is not type(value)
        for key, value in expected_transfer.items()
    ):
        raise ContractError("policy transfer semantics differ from the reviewed pairing")
    expected_checkpoint = {
        "source": "nvidia/GEAR-SONIC",
        "revision": "9c0ff22b4ffec27c5392e8e284eb2f2df7a5b4e2",
        "files": [
            {
                "path": "policy/release/model_decoder.onnx",
                "size": 40900688,
                "sha256": "c7241a123eaa36b5d64bad19540efde93cac1ad443bd4572fd12ca99898118ed",
            },
            {
                "path": "policy/release/model_encoder.onnx",
                "size": 50100513,
                "sha256": "013ab0287236aa2721e13f1e936d699db982302d0de0bfcdae76d5c3245362d3",
            },
            {
                "path": "policy/release/observation_config.yaml",
                "size": 2336,
                "sha256": "466d05947c78af6c76388adfb86e3a2a77b2a1d921a64883ed3d085ebf58de1b",
            },
        ],
    }
    if contract["checkpoint"] != expected_checkpoint:
        raise ContractError("checkpoint artifacts differ from the reviewed release")
    for index, artifact in enumerate(contract["checkpoint"]["files"]):
        if set(artifact) != {"path", "size", "sha256"}:
            raise ContractError(f"checkpoint.files[{index}] fields are malformed")
        if (
            not isinstance(artifact["path"], str)
            or Path(artifact["path"]).is_absolute()
            or ".." in Path(artifact["path"]).parts
            or _exact_int(artifact["size"], f"checkpoint.files[{index}].size") <= 0
            or not isinstance(artifact["sha256"], str)
            or len(artifact["sha256"]) != 64
        ):
            raise ContractError(f"checkpoint.files[{index}] is malformed")
    expected_conventions = {
        "position_unit": "radian",
        "velocity_unit": "radian_per_second",
        "torque_unit": "newton_meter",
        "policy_to_hardware_target": "q_lowcmd[h]=default_angle_rad[h]+command_offset_rad[h]+command_sign[h]*policy_action_scale_rad[h]*action[policy_isaaclab_index[h]]",
        "hardware_to_policy_position": "q_policy[p]=command_sign[h]*(q_lowstate[h]-default_angle_rad[h]-command_offset_rad[h]),p=policy_isaaclab_index[h]",
        "hardware_to_policy_velocity": "dq_policy[p]=command_sign[h]*dq_lowstate[h],p=policy_isaaclab_index[h]",
        "policy_to_fk": "q_mjcf[h]=command_sign[h]*q_policy_absolute[policy_isaaclab_index[h]]+command_offset_rad[h]",
    }
    if contract["joint_conventions"] != expected_conventions:
        raise ContractError("joint sign/unit conventions differ from the reviewed contract")

    lowcmd = contract["lowcmd"]
    if not isinstance(lowcmd, dict) or set(lowcmd) != {
        "mode_machine",
        "mode_pr",
        "active_motor_count",
        "motor_mode",
        "writer_rate_hz",
    }:
        raise ContractError("LowCmd contract fields are malformed")
    expected_lowcmd = {
        "mode_machine": 5,
        "mode_pr": 0,
        "active_motor_count": JOINT_COUNT,
        "motor_mode": 1,
        "writer_rate_hz": 500,
    }
    if lowcmd != expected_lowcmd or any(
        type(lowcmd[key]) is not type(value) for key, value in expected_lowcmd.items()
    ):
        raise ContractError("LowCmd contract differs from the reviewed mode-5 wire profile")

    simulation_timing = contract["simulation_timing"]
    expected_simulation_timing = {
        "physics_timestep_seconds": 0.002,
        "physics_rate_hz": 500,
        "physics_steps_per_lowcmd_period": 1,
        "torque_application": "external-pd-recomputed-once-per-mujoco-step",
    }
    if simulation_timing != expected_simulation_timing or any(
        type(simulation_timing[key]) is not type(value)
        for key, value in expected_simulation_timing.items()
    ):
        raise ContractError("simulation timing differs from the reviewed 500 Hz profile")
    physics_timestep = _number(
        simulation_timing["physics_timestep_seconds"],
        "simulation_timing.physics_timestep_seconds",
    )
    physics_rate = _exact_int(
        simulation_timing["physics_rate_hz"],
        "simulation_timing.physics_rate_hz",
    )
    steps_per_lowcmd = _exact_int(
        simulation_timing["physics_steps_per_lowcmd_period"],
        "simulation_timing.physics_steps_per_lowcmd_period",
    )
    if physics_rate != lowcmd["writer_rate_hz"] * steps_per_lowcmd:
        raise ContractError("simulation physics rate is incoherent with LowCmd writer rate")
    if physics_timestep * physics_rate != 1.0:
        raise ContractError("simulation physics timestep is not the reciprocal physics rate")

    measured_state_safety = contract["measured_state_safety"]
    expected_measured_state_safety = {
        "position_limit_tolerance_rad": 0.0,
        "velocity_limit_tolerance_rad_s": 0.0,
    }
    if measured_state_safety != expected_measured_state_safety or any(
        type(measured_state_safety[key]) is not type(value)
        for key, value in expected_measured_state_safety.items()
    ):
        raise ContractError("measured-state safety tolerances differ from fail-closed zero")

    takeover_control = contract["takeover_control"]
    expected_takeover_control = {
        "duration_seconds": 3.0,
        "semantics": "fixed-duration-linear-position-ramp-and-idle-hold",
        "source_role": "official-g1-low-level-init-and-idle-gain-schedule",
    }
    if takeover_control != expected_takeover_control or any(
        type(takeover_control[key]) is not type(value)
        for key, value in expected_takeover_control.items()
    ):
        raise ContractError("takeover control differs from the reviewed Unitree schedule")

    damping = contract["shutdown_damping"]
    expected_damping = {
        "q_rad": 0.0,
        "dq_rad_s": 0.0,
        "kp_nm_per_rad": 0.0,
        "kd_nm_s_per_rad": 8.0,
        "tau_ff_nm": 0.0,
        "repeat_ms": 250,
    }
    if not isinstance(damping, dict) or set(damping) != set(expected_damping):
        raise ContractError("shutdown damping fields are malformed")
    for key, expected in expected_damping.items():
        actual = damping[key]
        if type(actual) is not type(expected) or actual != expected:
            raise ContractError("shutdown damping differs from the reviewed boundary command")

    models = contract["models"]
    if not isinstance(models, dict) or set(models) != {
        "fk_mjcf",
        "sim_mjcf",
        "sim_scene",
        "sim_config",
        "sim_scene_include",
        "sim_config_scene_override",
        "common_kinematics_sha256",
        "common_29dof_kinematics_sha256",
    }:
        raise ContractError("model contract fields are malformed")
    expected_model_artifacts = {
        "fk_mjcf": {
            "path_from_wbc_root": "gear_sonic/data/assets/robot_description/mjcf/g1_29dof_rev_1_0.xml",
            "size": 36030,
            "sha256": "15a330f1dbff68cdbeb8284be6162a857bdcd21a858f65a8617fb32d5115f209",
            "role": "canonical-mode5-fk-and-hard-position-limits",
        },
        "sim_mjcf": {
            "path_from_wbc_root": "decoupled_wbc/control/robot_model/model_data/g1/g1_29dof_with_hand_rev_1_0_activatedfinger.xml",
            "size": 45258,
            "sha256": "388d682c8ed36e833eca986109fdca2797c4d5f3882822e2b03a1af1a2675f07",
            "role": "mode5-native-mujoco-with-dex3-extension",
        },
        "sim_scene": {
            "path_from_wbc_root": "decoupled_wbc/control/robot_model/model_data/g1/scene_29dof_activated3dex.xml",
            "size": 1169,
            "sha256": "88a24971e125eb20382e59217a18ee97eb0756c3e450e29a95a9e34ef397fbfc",
            "role": "mode5-native-mujoco-scene-floor-friction-and-options",
        },
        "sim_config": {
            "path_from_wbc_root": "gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml",
            "size": 12448,
            "sha256": "15520caa4b5748fab451f7f34661646bfb9db9dbd06fd8ed240f507f629d01a6",
            "role": "base-wbc-config-overridden-to-mode5-scene-by-sim-profile",
        },
    }
    for label, expected in expected_model_artifacts.items():
        _validate_artifact(models[label], label)
        if models[label] != expected:
            raise ContractError(f"{label} differs from the reviewed artifact")
    if models["sim_scene_include"] != "g1_29dof_with_hand_rev_1_0_activatedfinger.xml":
        raise ContractError("simulation scene include differs from the reviewed MJCF")
    if models["sim_config_scene_override"] != (
        "decoupled_wbc/control/robot_model/model_data/g1/"
        "scene_29dof_activated3dex.xml"
    ):
        raise ContractError("simulation config override differs from the reviewed scene")
    semantic_digest = models["common_kinematics_sha256"]
    if not isinstance(semantic_digest, str) or len(semantic_digest) != 64 or any(
        character not in "0123456789abcdef" for character in semantic_digest
    ):
        raise ContractError("common kinematics digest must be lowercase SHA-256")
    if models["common_29dof_kinematics_sha256"] != semantic_digest:
        raise ContractError("common kinematics digest aliases disagree")
    if semantic_digest != "4b4272979a730962981652897f5035d02abbab9db4197dd29320266f564ffdc5":
        raise ContractError("common kinematics digest differs from the reviewed asset")

    joints = contract["joints"]
    if not isinstance(joints, list) or len(joints) != JOINT_COUNT:
        raise ContractError(f"contract must contain exactly {JOINT_COUNT} joints")
    expected_fields = {
        "index",
        "name",
        "policy_isaaclab_index",
        "command_sign",
        "command_offset_rad",
        "position_lower_rad",
        "position_upper_rad",
        "physical_actuator",
        "plant_effort_limit_nm",
        "plant_velocity_limit_rad_s",
        "plant_armature_kg_m2",
        "passive_damping_nm_s_per_rad",
        "passive_frictionloss_nm",
        "takeover_kp_nm_per_rad",
        "takeover_kd_nm_s_per_rad",
        "takeover_gain_basis",
        "policy_gain_basis",
        "policy_kp_nm_per_rad",
        "policy_kd_nm_s_per_rad",
        "policy_action_scale_rad",
        "default_angle_rad",
    }
    names: set[str] = set()
    isaac_indices: set[int] = set()
    for index, row in enumerate(joints):
        label = f"joints[{index}]"
        if not isinstance(row, dict) or set(row) != expected_fields:
            raise ContractError(f"{label} fields are malformed")
        if _exact_int(row["index"], f"{label}.index") != index:
            raise ContractError(f"{label} index is not contiguous")
        name = row["name"]
        if not isinstance(name, str) or not name.endswith("_joint") or name in names:
            raise ContractError(f"{label}.name is invalid or duplicated")
        names.add(name)
        policy_index = _exact_int(row["policy_isaaclab_index"], f"{label}.policy_isaaclab_index")
        if not 0 <= policy_index < JOINT_COUNT or policy_index in isaac_indices:
            raise ContractError(f"{label}.policy_isaaclab_index is invalid or duplicated")
        isaac_indices.add(policy_index)
        if _number(row["command_sign"], f"{label}.command_sign") != 1.0:
            raise ContractError(f"{label}.command_sign must remain +1")
        if _number(row["command_offset_rad"], f"{label}.command_offset_rad") != 0.0:
            raise ContractError(f"{label}.command_offset_rad must remain zero")
        lower = _number(row["position_lower_rad"], f"{label}.position_lower_rad")
        upper = _number(row["position_upper_rad"], f"{label}.position_upper_rad")
        if not lower < upper:
            raise ContractError(f"{label} position limits are unordered")
        for field in (
            "plant_effort_limit_nm",
            "plant_velocity_limit_rad_s",
            "plant_armature_kg_m2",
            "takeover_kp_nm_per_rad",
            "takeover_kd_nm_s_per_rad",
            "policy_kp_nm_per_rad",
            "policy_kd_nm_s_per_rad",
            "policy_action_scale_rad",
        ):
            if _number(row[field], f"{label}.{field}") <= 0.0:
                raise ContractError(f"{label}.{field} must be positive")
        for field in ("passive_damping_nm_s_per_rad", "passive_frictionloss_nm"):
            if _number(row[field], f"{label}.{field}") < 0.0:
                raise ContractError(f"{label}.{field} must be non-negative")
        _number(row["default_angle_rad"], f"{label}.default_angle_rad")
        default = _number(row["default_angle_rad"], f"{label}.default_angle_rad")
        if not lower <= default <= upper:
            raise ContractError(f"{label}.default_angle_rad is outside hard limits")
        for field in ("physical_actuator", "takeover_gain_basis", "policy_gain_basis"):
            if not isinstance(row[field], str) or not row[field]:
                raise ContractError(f"{label}.{field} must be non-empty")
        if row["takeover_gain_basis"] != "unitree-sdk2-g1-ankle-swing-example":
            raise ContractError(f"{label}.takeover_gain_basis differs from the source pin")
    if isaac_indices != set(range(JOINT_COUNT)):
        raise ContractError("policy joint map is not a permutation")
    expected_takeover_kp = (
        60.0, 60.0, 60.0, 100.0, 40.0, 40.0,
        60.0, 60.0, 60.0, 100.0, 40.0, 40.0,
        60.0, 40.0, 40.0,
        40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
        40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
    )
    expected_takeover_kd = (
        1.0, 1.0, 1.0, 2.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 2.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    )
    if tuple(row["takeover_kp_nm_per_rad"] for row in joints) != expected_takeover_kp:
        raise ContractError("takeover Kp schedule differs from the pinned Unitree example")
    if tuple(row["takeover_kd_nm_s_per_rad"] for row in joints) != expected_takeover_kd:
        raise ContractError("takeover Kd schedule differs from the pinned Unitree example")
    joint_digest = canonical_json_sha256(joints)
    if contract["joint_parameter_sha256"] != joint_digest or joint_digest != (
        "8c46e85155070a359dce3059a86a98b0c7cd8d8b1ba3079e2f4d97a13236237b"
    ):
        raise ContractError("joint parameters differ from the bit-preserved reviewed profile")


def _parse_unique_simulate_dt(data: bytes) -> float:
    """Return the one top-level YAML ``SIMULATE_DT`` scalar, fail closed."""

    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ContractError("sim_config is not valid UTF-8") from error
    values: list[float] = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        content = line.split("#", 1)[0].rstrip()
        match = re.fullmatch(r"(\s*)SIMULATE_DT\s*:\s*(\S+)\s*", content)
        if match is None:
            continue
        if match.group(1):
            raise ContractError(
                f"sim_config SIMULATE_DT must be top-level (line {line_number})"
            )
        try:
            value = float(match.group(2))
        except ValueError as error:
            raise ContractError("sim_config SIMULATE_DT is not numeric") from error
        if not math.isfinite(value) or value <= 0.0:
            raise ContractError("sim_config SIMULATE_DT must be finite and positive")
        values.append(value)
    if len(values) != 1:
        raise ContractError("sim_config must define SIMULATE_DT exactly once")
    return values[0]


def verify_model_artifacts(contract: dict[str, Any], wbc_root: Path) -> None:
    try:
        root = wbc_root.resolve(strict=True)
    except OSError as error:
        raise ContractError(f"cannot resolve WBC root: {error}") from error
    resolved_artifacts: dict[str, Path] = {}
    artifact_bytes: dict[str, bytes] = {}
    for label in ("fk_mjcf", "sim_mjcf", "sim_scene", "sim_config"):
        artifact = contract["models"][label]
        path = root / artifact["path_from_wbc_root"]
        try:
            resolved = path.resolve(strict=True)
        except OSError as error:
            raise ContractError(f"missing {label}: {path}") from error
        if path.is_symlink():
            raise ContractError(f"{label} must not be a symlink")
        if not resolved.is_relative_to(root) or not resolved.is_file():
            raise ContractError(f"{label} escapes the WBC root or is not a regular file")
        data = resolved.read_bytes()
        if len(data) != artifact["size"]:
            raise ContractError(f"{label} size mismatch")
        if hashlib.sha256(data).hexdigest() != artifact["sha256"]:
            raise ContractError(f"{label} SHA-256 mismatch")
        resolved_artifacts[label] = resolved
        artifact_bytes[label] = data

    yaml_timestep = _parse_unique_simulate_dt(artifact_bytes["sim_config"])
    if yaml_timestep != contract["simulation_timing"]["physics_timestep_seconds"]:
        raise ContractError(
            "sim_config SIMULATE_DT differs from simulation_timing physics timestep"
        )

    expected_digest = contract["models"]["common_kinematics_sha256"]
    fk_records, fk_digest = mjcf_kinematics_sha256(resolved_artifacts["fk_mjcf"])
    if fk_digest != expected_digest:
        raise ContractError("fk_mjcf common-kinematics digest mismatch")
    joint_records = [record for record in fk_records if record["joint"] != "floating_base_joint"]
    if len(joint_records) != JOINT_COUNT:
        raise ContractError("fk_mjcf does not contain the reviewed 29-joint tree")
    for row, record in zip(contract["joints"], joint_records, strict=True):
        if record["joint"] != row["name"]:
            raise ContractError("fk_mjcf joint order/name differs from the contract")
        try:
            lower, upper = map(float, record["range"])
            force_lower, force_upper = map(float, record["actuatorfrcrange"])
        except (TypeError, ValueError) as error:
            raise ContractError("fk_mjcf joint limit fields are malformed") from error
        if lower != row["position_lower_rad"] or upper != row["position_upper_rad"]:
            raise ContractError(f"fk_mjcf position range differs for {row['name']}")
        effort = row["plant_effort_limit_nm"]
        if force_lower != -effort or force_upper != effort:
            raise ContractError(f"fk_mjcf effort range differs for {row['name']}")

    shared_joint_names = {record["joint"] for record in fk_records}
    sim_records, sim_digest = mjcf_kinematics_sha256(
        resolved_artifacts["sim_mjcf"], allowed_joints=shared_joint_names
    )
    if sim_digest != expected_digest or sim_records != fk_records:
        raise ContractError("sim_mjcf does not share the reviewed 29-DOF kinematics")

    try:
        scene_root = ET.parse(resolved_artifacts["sim_scene"]).getroot()
    except ET.ParseError as error:
        raise ContractError("sim_scene is not valid XML") from error
    includes = scene_root.findall("include")
    expected_include = contract["models"]["sim_scene_include"]
    if len(includes) != 1 or includes[0].get("file") != expected_include:
        raise ContractError("sim_scene must contain exactly the reviewed direct include")
    included_path = (resolved_artifacts["sim_scene"].parent / expected_include).resolve()
    if included_path != resolved_artifacts["sim_mjcf"]:
        raise ContractError("sim_scene include does not resolve to the pinned sim_mjcf")


def verify_checkpoint_artifacts(contract: dict[str, Any], deploy_root: Path) -> None:
    try:
        root = deploy_root.resolve(strict=True)
    except OSError as error:
        raise ContractError(f"cannot resolve deploy root: {error}") from error
    if not root.is_dir():
        raise ContractError("deploy root is not a directory")
    for index, artifact in enumerate(contract["checkpoint"]["files"]):
        candidate = root / artifact["path"]
        try:
            resolved = candidate.resolve(strict=True)
        except OSError as error:
            raise ContractError(f"missing checkpoint artifact: {artifact['path']}") from error
        if candidate.is_symlink() or not resolved.is_relative_to(root) or not resolved.is_file():
            raise ContractError(
                f"checkpoint.files[{index}] escapes the deploy root or is not a regular file"
            )
        data = resolved.read_bytes()
        if len(data) != artifact["size"]:
            raise ContractError(f"checkpoint artifact size mismatch: {artifact['path']}")
        if hashlib.sha256(data).hexdigest() != artifact["sha256"]:
            raise ContractError(f"checkpoint artifact SHA-256 mismatch: {artifact['path']}")


def verify_takeover_source_artifact(
    contract: dict[str, Any], deploy_root: Path, takeover_source: Path
) -> None:
    """Bind the reviewed takeover gains to the vendored Unitree source bytes."""

    try:
        root = deploy_root.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise ContractError(f"cannot resolve deploy root: {error}") from error
    if not root.is_dir():
        raise ContractError("deploy root is not a directory")

    source_pin = contract["sources"]["unitree_takeover_control"]
    vendored_root_path = root / "thirdparty/unitree_sdk2"
    try:
        vendored_root = vendored_root_path.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise ContractError(
            f"cannot resolve vendored Unitree SDK root: {vendored_root_path}"
        ) from error
    if not vendored_root.is_relative_to(root) or not vendored_root.is_dir():
        raise ContractError("vendored Unitree SDK root escapes the deploy root")

    expected_path = vendored_root_path / source_pin["path"]
    try:
        expected_resolved = expected_path.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise ContractError(f"missing takeover source: {expected_path}") from error
    try:
        supplied_resolved = takeover_source.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise ContractError(f"missing takeover source: {takeover_source}") from error

    if takeover_source.is_symlink():
        raise ContractError("takeover source must not be a symlink")
    if (
        supplied_resolved != expected_resolved
        or not supplied_resolved.is_relative_to(vendored_root)
        or not supplied_resolved.is_file()
    ):
        raise ContractError(
            "takeover source is not the expected regular file below the vendored "
            "Unitree SDK root"
        )
    try:
        digest = hashlib.sha256(supplied_resolved.read_bytes()).hexdigest()
    except OSError as error:
        raise ContractError(f"cannot read takeover source: {error}") from error
    if digest != source_pin["sha256"]:
        raise ContractError("takeover source SHA-256 mismatch")


def mjcf_kinematics_sha256(
    path: Path, *, allowed_joints: set[str] | None = None
) -> tuple[list[dict[str, Any]], str]:
    """Hash FK-relevant body/joint semantics, ignoring XML formatting/meshes.

    The simulator model extends the official 29-DOF tree with Dex3 hand
    bodies.  ``allowed_joints`` selects the official common tree while still
    traversing the extension, so any change to a shared parent/transform,
    axis, position range, or force range changes the digest.
    """

    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as error:
        raise ContractError(f"cannot parse MJCF {path}: {error}") from error
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ContractError(f"MJCF has no worldbody: {path}")
    records: list[dict[str, Any]] = []

    def walk(body: ET.Element, parent: str | None = None) -> None:
        joints = body.findall("joint")
        if len(joints) != 1:
            raise ContractError(
                f"MJCF body {body.get('name')!r} has {len(joints)} direct joints"
            )
        joint = joints[0]
        joint_name = joint.get("name")
        if allowed_joints is None or joint_name in allowed_joints:
            records.append(
                {
                    "body": body.get("name"),
                    "parent": parent,
                    "body_pos": body.get("pos", "0 0 0").split(),
                    "body_quat": body.get("quat", "1 0 0 0").split(),
                    "joint": joint_name,
                    "type": joint.get("type", "hinge"),
                    "joint_pos": joint.get("pos", "0 0 0").split(),
                    "axis": joint.get("axis", "0 0 0").split(),
                    "range": joint.get("range", "").split(),
                    "actuatorfrcrange": joint.get(
                        "actuatorfrcrange", ""
                    ).split(),
                }
            )
        for child in body.findall("body"):
            walk(child, body.get("name"))

    for body in worldbody.findall("body"):
        walk(body)
    payload = json.dumps(
        records, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return records, hashlib.sha256(payload).hexdigest()


def _cpp_string(value: str) -> str:
    return json.dumps(value, ensure_ascii=True)


def _cpp_number(value: Any, *, suffix: str = "") -> str:
    number = _number(value, "generated number")
    literal = format(number, ".17g")
    if "." not in literal and "e" not in literal:
        literal += ".0"
    return literal + suffix


def _cpp_bit_exact(value: Any, *, kind: str) -> str:
    number = _number(value, "generated number")
    if kind == "float":
        bits = struct.unpack("<I", struct.pack("<f", number))[0]
        return f"std::bit_cast<float>(std::uint32_t{{0x{bits:08x}u}})"
    if kind == "double":
        bits = struct.unpack("<Q", struct.pack("<d", number))[0]
        return f"std::bit_cast<double>(std::uint64_t{{0x{bits:016x}ull}})"
    raise AssertionError(kind)


def _array(rows: list[dict[str, Any]], field: str, *, suffix: str = "") -> str:
    return ", ".join(_cpp_number(row[field], suffix=suffix) for row in rows)


def _exact_array(rows: list[dict[str, Any]], field: str, *, kind: str) -> str:
    return ", ".join(_cpp_bit_exact(row[field], kind=kind) for row in rows)


def render_header(contract: dict[str, Any]) -> str:
    rows = contract["joints"]
    mujoco_to_isaac = [int(row["policy_isaaclab_index"]) for row in rows]
    isaac_to_mujoco = [0] * JOINT_COUNT
    for mujoco_index, isaac_index in enumerate(mujoco_to_isaac):
        isaac_to_mujoco[isaac_index] = mujoco_index
    digest = canonical_sha256(contract)
    checkpoint_digest = canonical_json_sha256(contract["checkpoint"])
    checkpoint_files = contract["checkpoint"]["files"]
    sources = contract["sources"]
    lines = [
        "// Generated by tools/generate_mode5_contract.py. DO NOT EDIT.",
        "#ifndef SONIC_G1_MODE5_CONTRACT_GENERATED_HPP",
        "#define SONIC_G1_MODE5_CONTRACT_GENERATED_HPP",
        "",
        "#include <array>",
        "#include <bit>",
        "#include <cstdint>",
        "#include <string_view>",
        "",
        "namespace sonic::mode5_contract {",
        "struct CheckpointArtifact {",
        "  std::string_view path_from_deploy_root;",
        "  std::uint64_t size;",
        "  std::string_view sha256;",
        "};",
        "",
        'inline constexpr std::string_view kCapabilitySchema = "motionlcm.sonic.capabilities.v5";',
        f"inline constexpr std::string_view kProfileId = {_cpp_string(contract['profile_id'])};",
        f"inline constexpr std::string_view kCanonicalSha256 = {_cpp_string(digest)};",
        'inline constexpr std::string_view kCanonicalization = "json.dumps(sort_keys=True,separators=(\\\",\\\",\\\":\\\"),ensure_ascii=False,allow_nan=False)-utf8-sha256";',
        f"inline constexpr std::string_view kStatus = {_cpp_string(contract['status'])};",
        f"inline constexpr bool kVendorAttested = {str(contract['vendor_attested']).lower()};",
        f"inline constexpr std::string_view kCompatibilityBasis = {_cpp_string(contract['compatibility_basis'])};",
        f"inline constexpr int kContractRevision = {contract['contract_revision']};",
        f"inline constexpr std::string_view kContractPathFromDeployRoot = {_cpp_string('g1/sonic_g1_mode5_derived_v1.contract.json')};",
        f"inline constexpr std::string_view kUnitreeModelRepository = {_cpp_string(sources['unitree_model']['repository'])};",
        f"inline constexpr std::string_view kUnitreeModelRevision = {_cpp_string(sources['unitree_model']['revision'])};",
        f"inline constexpr std::string_view kUnitreeModelSourcePath = {_cpp_string(sources['unitree_model']['path'])};",
        f"inline constexpr std::string_view kUnitreeModelSourceSha256 = {_cpp_string(sources['unitree_model']['sha256'])};",
        f"inline constexpr std::string_view kUnitreeActuatorRepository = {_cpp_string(sources['unitree_actuator']['repository'])};",
        f"inline constexpr std::string_view kUnitreeActuatorRevision = {_cpp_string(sources['unitree_actuator']['revision'])};",
        f"inline constexpr std::string_view kUnitreeActuatorSourcePath = {_cpp_string(sources['unitree_actuator']['path'])};",
        f"inline constexpr std::string_view kUnitreeActuatorSourceSha256 = {_cpp_string(sources['unitree_actuator']['sha256'])};",
        f"inline constexpr std::string_view kTakeoverSourceRepository = {_cpp_string(sources['unitree_takeover_control']['repository'])};",
        f"inline constexpr std::string_view kTakeoverSourceRevision = {_cpp_string(sources['unitree_takeover_control']['revision'])};",
        f"inline constexpr std::string_view kTakeoverSourcePath = {_cpp_string(sources['unitree_takeover_control']['path'])};",
        f"inline constexpr std::string_view kTakeoverSourceSha256 = {_cpp_string(sources['unitree_takeover_control']['sha256'])};",
        f"inline constexpr std::string_view kCheckpointSource = {_cpp_string(contract['checkpoint']['source'])};",
        f"inline constexpr std::string_view kCheckpointRevision = {_cpp_string(contract['checkpoint']['revision'])};",
        f"inline constexpr std::string_view kCheckpointBundleSha256 = {_cpp_string(checkpoint_digest)};",
        f"inline constexpr std::array<CheckpointArtifact, {len(checkpoint_files)}> kCheckpointArtifacts = {{{{",
        *[
            "    {"
            + _cpp_string(item["path"])
            + f", std::uint64_t{{{item['size']}}}, "
            + _cpp_string(item["sha256"])
            + "},"
            for item in checkpoint_files
        ],
        "}};",
        f"inline constexpr int kCheckpointTrainingModeMachine = {contract['policy_transfer']['checkpoint_training_mode_machine']};",
        f"inline constexpr int kPhysicalPlantModeMachine = {contract['policy_transfer']['physical_plant_mode_machine']};",
        f"inline constexpr std::string_view kPolicyParameterSemantics = {_cpp_string(contract['policy_transfer']['parameter_semantics'])};",
        f"inline constexpr std::string_view kJointParameterSha256 = {_cpp_string(contract['joint_parameter_sha256'])};",
        f"inline constexpr std::string_view kFkMjcfPathFromWbcRoot = {_cpp_string(contract['models']['fk_mjcf']['path_from_wbc_root'])};",
        f"inline constexpr std::uint64_t kFkMjcfSize = {contract['models']['fk_mjcf']['size']};",
        f"inline constexpr std::string_view kFkMjcfSha256 = {_cpp_string(contract['models']['fk_mjcf']['sha256'])};",
        f"inline constexpr std::string_view kSimMjcfPathFromWbcRoot = {_cpp_string(contract['models']['sim_mjcf']['path_from_wbc_root'])};",
        f"inline constexpr std::uint64_t kSimMjcfSize = {contract['models']['sim_mjcf']['size']};",
        f"inline constexpr std::string_view kSimMjcfSha256 = {_cpp_string(contract['models']['sim_mjcf']['sha256'])};",
        f"inline constexpr std::string_view kSimScenePathFromWbcRoot = {_cpp_string(contract['models']['sim_scene']['path_from_wbc_root'])};",
        f"inline constexpr std::uint64_t kSimSceneSize = {contract['models']['sim_scene']['size']};",
        f"inline constexpr std::string_view kSimSceneSha256 = {_cpp_string(contract['models']['sim_scene']['sha256'])};",
        f"inline constexpr std::string_view kSimConfigPathFromWbcRoot = {_cpp_string(contract['models']['sim_config']['path_from_wbc_root'])};",
        f"inline constexpr std::uint64_t kSimConfigSize = {contract['models']['sim_config']['size']};",
        f"inline constexpr std::string_view kSimConfigSha256 = {_cpp_string(contract['models']['sim_config']['sha256'])};",
        f"inline constexpr std::string_view kSimSceneInclude = {_cpp_string(contract['models']['sim_scene_include'])};",
        f"inline constexpr std::string_view kSimConfigSceneOverride = {_cpp_string(contract['models']['sim_config_scene_override'])};",
        f"inline constexpr std::string_view kKinematicsSha256 = {_cpp_string(contract['models']['common_kinematics_sha256'])};",
        f"inline constexpr std::uint8_t kModeMachine = {contract['lowcmd']['mode_machine']};",
        f"inline constexpr std::uint8_t kModePr = {contract['lowcmd']['mode_pr']};",
        f"inline constexpr int kActiveMotorCount = {contract['lowcmd']['active_motor_count']};",
        f"inline constexpr int kMotorMode = {contract['lowcmd']['motor_mode']};",
        f"inline constexpr int kWriterRateHz = {contract['lowcmd']['writer_rate_hz']};",
        f"inline constexpr double kPhysicsTimestepSeconds = {_cpp_bit_exact(contract['simulation_timing']['physics_timestep_seconds'], kind='double')};",
        f"inline constexpr int kPhysicsRateHz = {contract['simulation_timing']['physics_rate_hz']};",
        f"inline constexpr int kPhysicsStepsPerLowcmdPeriod = {contract['simulation_timing']['physics_steps_per_lowcmd_period']};",
        f"inline constexpr std::string_view kTorqueApplication = {_cpp_string(contract['simulation_timing']['torque_application'])};",
        f"inline constexpr double kMeasuredPositionTolerance = {_cpp_bit_exact(contract['measured_state_safety']['position_limit_tolerance_rad'], kind='double')};",
        f"inline constexpr double kMeasuredVelocityTolerance = {_cpp_bit_exact(contract['measured_state_safety']['velocity_limit_tolerance_rad_s'], kind='double')};",
        f"inline constexpr double kTakeoverDurationSeconds = {_cpp_bit_exact(contract['takeover_control']['duration_seconds'], kind='double')};",
        f"inline constexpr std::string_view kTakeoverSemantics = {_cpp_string(contract['takeover_control']['semantics'])};",
        f"inline constexpr std::string_view kTakeoverSourceRole = {_cpp_string(contract['takeover_control']['source_role'])};",
        f"inline constexpr float kShutdownQ = {_cpp_number(contract['shutdown_damping']['q_rad'], suffix='f')};",
        f"inline constexpr float kShutdownDq = {_cpp_number(contract['shutdown_damping']['dq_rad_s'], suffix='f')};",
        f"inline constexpr float kShutdownKp = {_cpp_number(contract['shutdown_damping']['kp_nm_per_rad'], suffix='f')};",
        f"inline constexpr float kShutdownKd = {_cpp_number(contract['shutdown_damping']['kd_nm_s_per_rad'], suffix='f')};",
        f"inline constexpr float kShutdownTauFf = {_cpp_number(contract['shutdown_damping']['tau_ff_nm'], suffix='f')};",
        f"inline constexpr int kShutdownRepeatMs = {contract['shutdown_damping']['repeat_ms']};",
        "",
        "inline constexpr std::array<std::string_view, 29> kJointNames = {",
        "    " + ", ".join(_cpp_string(row["name"]) for row in rows),
        "};",
        "inline constexpr std::array<std::string_view, 29> kPhysicalActuator = {",
        "    " + ", ".join(_cpp_string(row["physical_actuator"]) for row in rows),
        "};",
        "inline constexpr std::array<std::string_view, 29> kPolicyGainBasis = {",
        "    " + ", ".join(_cpp_string(row["policy_gain_basis"]) for row in rows),
        "};",
        "inline constexpr std::array<std::string_view, 29> kTakeoverGainBasis = {",
        "    " + ", ".join(_cpp_string(row["takeover_gain_basis"]) for row in rows),
        "};",
        "inline constexpr std::array<int, 29> kMujocoOrderInIsaaclabIndex = {",
        "    " + ", ".join(str(value) for value in mujoco_to_isaac),
        "};",
        "inline constexpr std::array<int, 29> kIsaaclabOrderInMujocoIndex = {",
        "    " + ", ".join(str(value) for value in isaac_to_mujoco),
        "};",
        "inline constexpr std::array<double, 29> kCommandSign = {",
        "    " + _exact_array(rows, "command_sign", kind="double"),
        "};",
        "inline constexpr std::array<double, 29> kCommandOffset = {",
        "    " + _exact_array(rows, "command_offset_rad", kind="double"),
        "};",
        "inline constexpr std::array<double, 29> kPositionLower = {",
        "    " + _exact_array(rows, "position_lower_rad", kind="double"),
        "};",
        "inline constexpr std::array<double, 29> kPositionUpper = {",
        "    " + _exact_array(rows, "position_upper_rad", kind="double"),
        "};",
        "inline constexpr std::array<double, 29> kPlantEffortLimit = {",
        "    " + _exact_array(rows, "plant_effort_limit_nm", kind="double"),
        "};",
        "inline constexpr std::array<double, 29> kPlantVelocityLimit = {",
        "    " + _exact_array(rows, "plant_velocity_limit_rad_s", kind="double"),
        "};",
        "inline constexpr std::array<double, 29> kPlantArmature = {",
        "    " + _exact_array(rows, "plant_armature_kg_m2", kind="double"),
        "};",
        "inline constexpr std::array<double, 29> kPassiveDamping = {",
        "    " + _exact_array(rows, "passive_damping_nm_s_per_rad", kind="double"),
        "};",
        "inline constexpr std::array<double, 29> kPassiveFrictionLoss = {",
        "    " + _exact_array(rows, "passive_frictionloss_nm", kind="double"),
        "};",
        "inline constexpr std::array<float, 29> kPolicyKp = {",
        "    " + _exact_array(rows, "policy_kp_nm_per_rad", kind="float"),
        "};",
        "inline constexpr std::array<float, 29> kPolicyKd = {",
        "    " + _exact_array(rows, "policy_kd_nm_s_per_rad", kind="float"),
        "};",
        "inline constexpr std::array<float, 29> kTakeoverKp = {",
        "    " + _exact_array(rows, "takeover_kp_nm_per_rad", kind="float"),
        "};",
        "inline constexpr std::array<float, 29> kTakeoverKd = {",
        "    " + _exact_array(rows, "takeover_kd_nm_s_per_rad", kind="float"),
        "};",
        "inline constexpr std::array<double, 29> kPolicyActionScale = {",
        "    " + _exact_array(rows, "policy_action_scale_rad", kind="double"),
        "};",
        "inline constexpr std::array<double, 29> kDefaultAngles = {",
        "    " + _exact_array(rows, "default_angle_rad", kind="double"),
        "};",
        "",
        "consteval bool JointMapIsABijection() {",
        "  std::array<bool, 29> seen{};",
        "  for (int hardware_index = 0; hardware_index < 29; ++hardware_index) {",
        "    const int policy_index = kMujocoOrderInIsaaclabIndex[hardware_index];",
        "    if (policy_index < 0 || policy_index >= 29 || seen[policy_index] ||",
        "        kIsaaclabOrderInMujocoIndex[policy_index] != hardware_index) return false;",
        "    seen[policy_index] = true;",
        "  }",
        "  return true;",
        "}",
        "consteval bool JointRowsAreSafe() {",
        "  for (int index = 0; index < 29; ++index) {",
        "    if (!(kPositionLower[index] < kPositionUpper[index]) ||",
        "        kDefaultAngles[index] < kPositionLower[index] ||",
        "        kDefaultAngles[index] > kPositionUpper[index] ||",
        "        (kCommandSign[index] != -1.0 && kCommandSign[index] != 1.0) ||",
        "        kPolicyKp[index] <= 0.0f || kPolicyKd[index] <= 0.0f ||",
        "        kTakeoverKp[index] <= 0.0f || kTakeoverKd[index] <= 0.0f ||",
        "        kTakeoverGainBasis[index].empty() ||",
        "        kPolicyActionScale[index] <= 0.0 ||",
        "        kPlantEffortLimit[index] <= 0.0 ||",
        "        kPlantVelocityLimit[index] <= 0.0) return false;",
        "  }",
        "  return true;",
        "}",
        "consteval bool TakeoverScheduleIsPinned() {",
        "  constexpr std::array<float, 29> expected_kp{{",
        "      60, 60, 60, 100, 40, 40, 60, 60, 60, 100, 40, 40, 60, 40, 40,",
        "      40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,",
        "  }};",
        "  constexpr std::array<float, 29> expected_kd{{",
        "      1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1,",
        "      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,",
        "  }};",
        "  return kTakeoverKp == expected_kp && kTakeoverKd == expected_kd;",
        "}",
        "static_assert(!kVendorAttested);",
        "static_assert(kModeMachine == kPhysicalPlantModeMachine);",
        "static_assert(kActiveMotorCount == 29);",
        "static_assert(kPhysicsTimestepSeconds == 0.002);",
        "static_assert(kPhysicsRateHz == kWriterRateHz * kPhysicsStepsPerLowcmdPeriod);",
        "static_assert(kPhysicsTimestepSeconds * kPhysicsRateHz == 1.0);",
        "static_assert(kPhysicsStepsPerLowcmdPeriod == 1);",
        "static_assert(!kTorqueApplication.empty());",
        "static_assert(kTakeoverDurationSeconds == 3.0);",
        "static_assert(!kTakeoverSourceRepository.empty());",
        "static_assert(!kTakeoverSourceRevision.empty());",
        "static_assert(!kTakeoverSourcePath.empty());",
        "static_assert(kTakeoverSourceSha256.size() == 64);",
        "static_assert(JointMapIsABijection());",
        "static_assert(JointRowsAreSafe());",
        "static_assert(TakeoverScheduleIsPinned());",
        "",
        "}  // namespace sonic::mode5_contract",
        "",
        "#endif  // SONIC_G1_MODE5_CONTRACT_GENERATED_HPP",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--header", type=Path, required=True)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--check", action="store_true")
    action.add_argument("--write", action="store_true")
    parser.add_argument("--wbc-root", type=Path)
    parser.add_argument("--deploy-root", type=Path)
    parser.add_argument("--takeover-source", type=Path)
    args = parser.parse_args()
    try:
        contract = load_contract(args.contract)
        if args.check and (
            args.deploy_root is None or args.takeover_source is None
        ):
            raise ContractError(
                "--check requires --deploy-root and --takeover-source"
            )
        if args.wbc_root is not None:
            verify_model_artifacts(contract, args.wbc_root)
        if args.deploy_root is not None:
            verify_checkpoint_artifacts(contract, args.deploy_root)
        if args.takeover_source is not None:
            if args.deploy_root is None:
                raise ContractError("--takeover-source requires --deploy-root")
            verify_takeover_source_artifact(
                contract, args.deploy_root, args.takeover_source
            )
        expected = render_header(contract)
        if args.write:
            args.header.parent.mkdir(parents=True, exist_ok=True)
            args.header.write_text(expected, encoding="utf-8")
            return 0
        try:
            actual = args.header.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as error:
            raise ContractError(f"cannot read generated header: {error}") from error
        if actual != expected:
            raise ContractError(
                "generated header is stale; regenerate it from the reviewed JSON contract"
            )
    except ContractError as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
