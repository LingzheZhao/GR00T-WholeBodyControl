#!/usr/bin/env python3
"""Validate G1 Mode-5 control parameters and generate their C++ mirror.

The JSON contract is the source of truth for values that affect joint ordering,
LowCmd, gains, limits, and simulation timing. Repository revisions, hashes, and
file sizes in the JSON are provenance notes only: they are deliberately not
build admission criteria. When a WBC root is supplied, model semantics are
checked directly, independent of file bytes or formatting.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import struct
from typing import Any
from xml.etree import ElementTree as ET


SCHEMA = "motionlcm.g1.deployment-contract.v1"
JOINT_COUNT = 29
G1_MODE5_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


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


def _object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ContractError(f"{label} must be an object")
    return value


def _require_fields(value: dict[str, Any], fields: set[str], label: str) -> None:
    missing = fields - set(value)
    if missing:
        raise ContractError(f"{label} is missing fields: {', '.join(sorted(missing))}")


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


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ContractError(f"{label} must be a non-empty string")
    return value


def _relative_path(value: Any, label: str) -> Path:
    text = _nonempty_string(value, label)
    path = Path(text)
    if path.is_absolute() or ".." in path.parts:
        raise ContractError(f"{label} must be a relative path without '..'")
    return path


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


def _validate(contract: dict[str, Any]) -> None:
    required = {
        "schema",
        "joint_conventions",
        "lowcmd",
        "simulation_timing",
        "takeover_control",
        "shutdown_damping",
        "models",
        "joints",
    }
    _require_fields(contract, required, "contract")
    if contract["schema"] != SCHEMA:
        raise ContractError("unsupported contract schema")

    # These equations define how the arrays are interpreted. A new convention
    # needs generator/controller support, not a metadata-only edit.
    expected_conventions = {
        "position_unit": "radian",
        "velocity_unit": "radian_per_second",
        "torque_unit": "newton_meter",
        "policy_to_hardware_target": "q_lowcmd[h]=default_angle_rad[h]+command_offset_rad[h]+command_sign[h]*policy_action_scale_rad[h]*action[policy_isaaclab_index[h]]",
        "hardware_to_policy_position": "q_policy[p]=command_sign[h]*(q_lowstate[h]-default_angle_rad[h]-command_offset_rad[h]),p=policy_isaaclab_index[h]",
        "hardware_to_policy_velocity": "dq_policy[p]=command_sign[h]*dq_lowstate[h],p=policy_isaaclab_index[h]",
        "policy_to_fk": "q_mjcf[h]=command_sign[h]*q_policy_absolute[policy_isaaclab_index[h]]+command_offset_rad[h]",
    }
    conventions = _object(contract["joint_conventions"], "joint_conventions")
    if any(conventions.get(key) != value for key, value in expected_conventions.items()):
        raise ContractError("unsupported joint sign/unit conventions")

    lowcmd = _object(contract["lowcmd"], "lowcmd")
    _require_fields(
        lowcmd,
        {"mode_machine", "mode_pr", "active_motor_count", "motor_mode", "writer_rate_hz"},
        "lowcmd",
    )
    expected_wire_values = {
        "mode_machine": 5,
        "mode_pr": 0,
        "active_motor_count": JOINT_COUNT,
        "motor_mode": 1,
        "writer_rate_hz": 500,
    }
    for key, expected in expected_wire_values.items():
        if _exact_int(lowcmd[key], f"lowcmd.{key}") != expected:
            raise ContractError(f"lowcmd.{key} is incompatible with the G1 Mode-5 wire protocol")

    timing = _object(contract["simulation_timing"], "simulation_timing")
    _require_fields(
        timing,
        {
            "physics_timestep_seconds",
            "physics_rate_hz",
            "physics_steps_per_lowcmd_period",
            "torque_application",
        },
        "simulation_timing",
    )
    timestep = _number(
        timing["physics_timestep_seconds"],
        "simulation_timing.physics_timestep_seconds",
    )
    physics_rate = _exact_int(
        timing["physics_rate_hz"], "simulation_timing.physics_rate_hz"
    )
    steps = _exact_int(
        timing["physics_steps_per_lowcmd_period"],
        "simulation_timing.physics_steps_per_lowcmd_period",
    )
    if timestep <= 0.0 or physics_rate <= 0 or steps <= 0:
        raise ContractError("simulation timing values must be positive")
    if physics_rate != lowcmd["writer_rate_hz"] * steps:
        raise ContractError("simulation physics rate is incoherent with LowCmd writer rate")
    if not math.isclose(timestep * physics_rate, 1.0, rel_tol=0.0, abs_tol=1e-12):
        raise ContractError("simulation physics timestep is not the reciprocal physics rate")
    _nonempty_string(
        timing["torque_application"], "simulation_timing.torque_application"
    )

    takeover = _object(contract["takeover_control"], "takeover_control")
    _require_fields(takeover, {"duration_seconds"}, "takeover_control")
    if _number(
        takeover["duration_seconds"], "takeover_control.duration_seconds"
    ) <= 0.0:
        raise ContractError("takeover_control.duration_seconds must be positive")

    damping = _object(contract["shutdown_damping"], "shutdown_damping")
    _require_fields(
        damping,
        {
            "q_rad",
            "dq_rad_s",
            "kp_nm_per_rad",
            "kd_nm_s_per_rad",
            "tau_ff_nm",
            "repeat_ms",
        },
        "shutdown_damping",
    )
    for field in ("q_rad", "dq_rad_s", "tau_ff_nm"):
        _number(damping[field], f"shutdown_damping.{field}")
    for field in ("kp_nm_per_rad", "kd_nm_s_per_rad"):
        if _number(damping[field], f"shutdown_damping.{field}") < 0.0:
            raise ContractError(f"shutdown_damping.{field} must be non-negative")
    if _exact_int(damping["repeat_ms"], "shutdown_damping.repeat_ms") <= 0:
        raise ContractError("shutdown_damping.repeat_ms must be positive")

    models = _object(contract["models"], "models")
    _require_fields(
        models,
        {"fk_mjcf", "sim_mjcf", "sim_scene", "sim_config", "sim_scene_include"},
        "models",
    )
    for label in ("fk_mjcf", "sim_mjcf", "sim_scene", "sim_config"):
        artifact = _object(models[label], f"models.{label}")
        _require_fields(artifact, {"path_from_wbc_root"}, f"models.{label}")
        _relative_path(
            artifact["path_from_wbc_root"],
            f"models.{label}.path_from_wbc_root",
        )
    include = _relative_path(
        models["sim_scene_include"], "models.sim_scene_include"
    )
    if len(include.parts) != 1:
        raise ContractError("models.sim_scene_include must be a direct filename")

    joints = contract["joints"]
    if not isinstance(joints, list) or len(joints) != JOINT_COUNT:
        raise ContractError(f"contract must contain exactly {JOINT_COUNT} joints")
    required_joint_fields = {
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
    policy_indices: set[int] = set()
    for index, row_value in enumerate(joints):
        label = f"joints[{index}]"
        row = _object(row_value, label)
        _require_fields(row, required_joint_fields, label)
        if _exact_int(row["index"], f"{label}.index") != index:
            raise ContractError(f"{label}.index is not contiguous")
        if row["name"] != G1_MODE5_JOINT_NAMES[index]:
            raise ContractError(
                f"{label}.name does not match the G1 29-DOF hardware order"
            )
        policy_index = _exact_int(
            row["policy_isaaclab_index"], f"{label}.policy_isaaclab_index"
        )
        if not 0 <= policy_index < JOINT_COUNT or policy_index in policy_indices:
            raise ContractError(
                f"{label}.policy_isaaclab_index is invalid or duplicated"
            )
        policy_indices.add(policy_index)
        sign = _number(row["command_sign"], f"{label}.command_sign")
        if sign not in (-1.0, 1.0):
            raise ContractError(f"{label}.command_sign must be -1 or +1")
        _number(row["command_offset_rad"], f"{label}.command_offset_rad")
        lower = _number(row["position_lower_rad"], f"{label}.position_lower_rad")
        upper = _number(row["position_upper_rad"], f"{label}.position_upper_rad")
        if not lower < upper:
            raise ContractError(f"{label} position limits are unordered")
        default = _number(row["default_angle_rad"], f"{label}.default_angle_rad")
        if not lower <= default <= upper:
            raise ContractError(f"{label}.default_angle_rad is outside hard limits")
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
        for field in (
            "passive_damping_nm_s_per_rad",
            "passive_frictionloss_nm",
        ):
            if _number(row[field], f"{label}.{field}") < 0.0:
                raise ContractError(f"{label}.{field} must be non-negative")
        for field in (
            "physical_actuator",
            "takeover_gain_basis",
            "policy_gain_basis",
        ):
            _nonempty_string(row[field], f"{label}.{field}")
    if policy_indices != set(range(JOINT_COUNT)):
        raise ContractError("policy joint map is not a permutation")


def _parse_unique_simulate_dt(data: bytes) -> float:
    """Return the one top-level YAML ``SIMULATE_DT`` scalar."""

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


def _mjcf_kinematics_records(
    path: Path, *, allowed_joints: set[str] | None = None
) -> list[dict[str, Any]]:
    """Read FK-relevant body/joint semantics, ignoring formatting and meshes."""

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
            record = {
                "body": body.get("name"),
                "parent": parent,
                "body_pos": body.get("pos", "0 0 0").split(),
                "body_quat": body.get("quat", "1 0 0 0").split(),
                "joint": joint_name,
                "type": joint.get("type", "hinge"),
                "joint_pos": joint.get("pos", "0 0 0").split(),
                "axis": joint.get("axis", "0 0 0").split(),
                "range": joint.get("range", "").split(),
                "actuatorfrcrange": joint.get("actuatorfrcrange", "").split(),
            }
            for field, width in (
                ("body_pos", 3),
                ("body_quat", 4),
                ("joint_pos", 3),
                ("axis", 3),
            ):
                tokens = record[field]
                if len(tokens) != width:
                    raise ContractError(
                        f"MJCF {joint_name} {field} must contain {width} numbers"
                    )
                try:
                    values = tuple(float(token) for token in tokens)
                except ValueError as error:
                    raise ContractError(
                        f"MJCF {joint_name} {field} is not numeric"
                    ) from error
                if not all(math.isfinite(value) for value in values):
                    raise ContractError(f"MJCF {joint_name} {field} must be finite")
                if field == "body_quat" and not any(value != 0.0 for value in values):
                    raise ContractError(f"MJCF {joint_name} body_quat is zero")
            records.append(record)
        for child in body.findall("body"):
            walk(child, body.get("name"))

    for body in worldbody.findall("body"):
        walk(body)
    return records


def verify_model_artifacts(contract: dict[str, Any], wbc_root: Path) -> None:
    """Verify model/controller semantics without checking identity or file bytes."""

    try:
        root = wbc_root.resolve(strict=True)
    except OSError as error:
        raise ContractError(f"cannot resolve WBC root: {error}") from error
    if not root.is_dir():
        raise ContractError("WBC root is not a directory")
    paths: dict[str, Path] = {}
    for label in ("fk_mjcf", "sim_mjcf", "sim_scene", "sim_config"):
        relative = _relative_path(
            contract["models"][label]["path_from_wbc_root"],
            f"models.{label}.path_from_wbc_root",
        )
        try:
            resolved = (root / relative).resolve(strict=True)
        except OSError as error:
            raise ContractError(f"missing {label}: {root / relative}") from error
        if not resolved.is_file():
            raise ContractError(f"{label} is not a regular file: {resolved}")
        paths[label] = resolved

    yaml_timestep = _parse_unique_simulate_dt(paths["sim_config"].read_bytes())
    contract_timestep = _number(
        contract["simulation_timing"]["physics_timestep_seconds"],
        "simulation_timing.physics_timestep_seconds",
    )
    if not math.isclose(
        yaml_timestep, contract_timestep, rel_tol=0.0, abs_tol=1e-12
    ):
        raise ContractError(
            "sim_config SIMULATE_DT differs from simulation_timing"
        )

    fk_records = _mjcf_kinematics_records(paths["fk_mjcf"])
    joint_records = [
        record for record in fk_records if record["joint"] != "floating_base_joint"
    ]
    if len(joint_records) != JOINT_COUNT:
        raise ContractError("fk_mjcf does not contain the G1 29-joint tree")
    for row, record in zip(contract["joints"], joint_records, strict=True):
        if record["joint"] != row["name"]:
            raise ContractError("fk_mjcf joint order/name differs from the contract")
        try:
            lower, upper = map(float, record["range"])
            force_lower, force_upper = map(float, record["actuatorfrcrange"])
        except (TypeError, ValueError) as error:
            raise ContractError("fk_mjcf joint limit fields are malformed") from error
        values = (lower, upper, force_lower, force_upper)
        if not all(math.isfinite(value) for value in values):
            raise ContractError(f"fk_mjcf has non-finite limits for {row['name']}")
        if lower != row["position_lower_rad"] or upper != row["position_upper_rad"]:
            raise ContractError(
                f"fk_mjcf position range differs for {row['name']}"
            )
        effort = row["plant_effort_limit_nm"]
        if force_lower != -effort or force_upper != effort:
            raise ContractError(f"fk_mjcf effort range differs for {row['name']}")

    shared_joint_names = {record["joint"] for record in fk_records}
    sim_records = _mjcf_kinematics_records(
        paths["sim_mjcf"], allowed_joints=shared_joint_names
    )
    if sim_records != fk_records:
        raise ContractError(
            "sim_mjcf and fk_mjcf do not share the same 29-DOF kinematics"
        )

    try:
        scene_root = ET.parse(paths["sim_scene"]).getroot()
    except ET.ParseError as error:
        raise ContractError("sim_scene is not valid XML") from error
    includes = scene_root.findall("include")
    expected_include = contract["models"]["sim_scene_include"]
    if len(includes) != 1 or includes[0].get("file") != expected_include:
        raise ContractError(
            "sim_scene must contain the configured direct model include"
        )
    if (
        paths["sim_scene"].parent / expected_include
    ).resolve() != paths["sim_mjcf"]:
        raise ContractError("sim_scene include does not resolve to sim_mjcf")


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


def _exact_array(rows: list[dict[str, Any]], field: str, *, kind: str) -> str:
    return ", ".join(_cpp_bit_exact(row[field], kind=kind) for row in rows)


def render_header(contract: dict[str, Any]) -> str:
    rows = contract["joints"]
    mujoco_to_isaac = [int(row["policy_isaaclab_index"]) for row in rows]
    isaac_to_mujoco = [0] * JOINT_COUNT
    for hardware_index, policy_index in enumerate(mujoco_to_isaac):
        isaac_to_mujoco[policy_index] = hardware_index
    lines = [
        "// Generated by tools/generate_mode5_contract.py. DO NOT EDIT.",
        "// Runtime parameters only; hashes/revisions are not build gates.",
        "#ifndef SONIC_G1_MODE5_CONTRACT_GENERATED_HPP",
        "#define SONIC_G1_MODE5_CONTRACT_GENERATED_HPP",
        "",
        "#include <array>",
        "#include <bit>",
        "#include <cstdint>",
        "#include <string_view>",
        "",
        "namespace sonic::mode5_contract {",
        f"inline constexpr std::string_view kFkMjcfPathFromWbcRoot = {_cpp_string(contract['models']['fk_mjcf']['path_from_wbc_root'])};",
        f"inline constexpr std::uint8_t kModeMachine = {contract['lowcmd']['mode_machine']};",
        f"inline constexpr std::uint8_t kModePr = {contract['lowcmd']['mode_pr']};",
        f"inline constexpr int kActiveMotorCount = {contract['lowcmd']['active_motor_count']};",
        f"inline constexpr int kMotorMode = {contract['lowcmd']['motor_mode']};",
        f"inline constexpr int kWriterRateHz = {contract['lowcmd']['writer_rate_hz']};",
        f"inline constexpr double kPhysicsTimestepSeconds = {_cpp_bit_exact(contract['simulation_timing']['physics_timestep_seconds'], kind='double')};",
        f"inline constexpr int kPhysicsRateHz = {contract['simulation_timing']['physics_rate_hz']};",
        f"inline constexpr int kPhysicsStepsPerLowcmdPeriod = {contract['simulation_timing']['physics_steps_per_lowcmd_period']};",
        f"inline constexpr double kTakeoverDurationSeconds = {_cpp_bit_exact(contract['takeover_control']['duration_seconds'], kind='double')};",
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
        "consteval bool JointRowsAreValid() {",
        "  for (int index = 0; index < 29; ++index) {",
        "    if (!(kPositionLower[index] < kPositionUpper[index]) ||",
        "        kDefaultAngles[index] < kPositionLower[index] ||",
        "        kDefaultAngles[index] > kPositionUpper[index] ||",
        "        (kCommandSign[index] != -1.0 && kCommandSign[index] != 1.0) ||",
        "        kPolicyKp[index] <= 0.0f || kPolicyKd[index] <= 0.0f ||",
        "        kTakeoverKp[index] <= 0.0f || kTakeoverKd[index] <= 0.0f ||",
        "        kPolicyActionScale[index] <= 0.0 ||",
        "        kPlantEffortLimit[index] <= 0.0 ||",
        "        kPlantVelocityLimit[index] <= 0.0) return false;",
        "  }",
        "  return true;",
        "}",
        "static_assert(kModeMachine == 5);",
        "static_assert(kModePr == 0);",
        "static_assert(kActiveMotorCount == 29);",
        "static_assert(kMotorMode == 1);",
        "static_assert(kWriterRateHz == 500);",
        "static_assert(kPhysicsRateHz == kWriterRateHz * kPhysicsStepsPerLowcmdPeriod);",
        "static_assert(kPhysicsTimestepSeconds * kPhysicsRateHz == 1.0);",
        "static_assert(JointMapIsABijection());",
        "static_assert(JointRowsAreValid());",
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
    parser.add_argument(
        "--wbc-root",
        type=Path,
        help="also verify live MJCF/YAML semantics; hashes and sizes are ignored",
    )
    args = parser.parse_args()
    try:
        contract = load_contract(args.contract)
        if args.wbc_root is not None:
            verify_model_artifacts(contract, args.wbc_root)
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
                "generated header is stale; run generate_mode5_contract.py --write"
            )
    except ContractError as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
