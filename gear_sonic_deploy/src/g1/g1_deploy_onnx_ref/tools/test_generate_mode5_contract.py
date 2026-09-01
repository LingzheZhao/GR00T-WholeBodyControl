from __future__ import annotations

import copy
import json
from pathlib import Path
import shutil
import tempfile
import unittest

import generate_mode5_contract as generator


CONTROLLER_ROOT = Path(__file__).resolve().parents[1]
DEPLOY_ROOT = CONTROLLER_ROOT.parents[2]
WBC_ROOT = CONTROLLER_ROOT.parents[3]
CONTRACT_PATH = DEPLOY_ROOT / "g1/sonic_g1_mode5_derived_v1.contract.json"
HEADER_PATH = CONTROLLER_ROOT / "include/mode5_contract_generated.hpp"


class Mode5ContractGeneratorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.contract = generator.load_contract(CONTRACT_PATH)

    def test_generated_header_matches_runtime_parameters_only(self) -> None:
        header = HEADER_PATH.read_text(encoding="utf-8")
        self.assertEqual(header, generator.render_header(self.contract))
        for obsolete_identity in (
            "Sha256",
            "CheckpointArtifact",
            "VendorAttested",
            "Canonicalization",
            "Repository",
            "Revision",
        ):
            self.assertNotIn(obsolete_identity, header)
        for required_parameter in (
            "kModeMachine",
            "kActiveMotorCount",
            "kJointNames",
            "kPositionLower",
            "kPositionUpper",
            "kPlantEffortLimit",
            "kPlantVelocityLimit",
            "kPolicyKp",
            "kPolicyKd",
            "kTakeoverKp",
            "kTakeoverKd",
            "JointMapIsABijection",
            "JointRowsAreValid",
        ):
            self.assertIn(required_parameter, header)

    def test_provenance_and_asset_identity_do_not_affect_the_build_header(self) -> None:
        changed = copy.deepcopy(self.contract)
        changed["profile_id"] = "local-development-profile"
        changed["sources"]["unitree_model"]["revision"] = "new-upstream-revision"
        changed["sources"]["unitree_model"]["sha256"] = "not-a-build-token"
        changed["checkpoint"]["revision"] = "replacement-checkpoint"
        for artifact in changed["checkpoint"]["files"]:
            artifact["size"] = 1
            artifact["sha256"] = "replacement"
        for label in ("fk_mjcf", "sim_mjcf", "sim_scene", "sim_config"):
            changed["models"][label]["size"] = 1
            changed["models"][label]["sha256"] = "replacement"
        changed["models"]["common_kinematics_sha256"] = "informational"
        changed["joint_parameter_sha256"] = "informational"

        generator._validate(changed)
        self.assertEqual(
            generator.render_header(changed), generator.render_header(self.contract)
        )

    def test_valid_gain_change_is_explicitly_generated_not_hash_blocked(self) -> None:
        changed = copy.deepcopy(self.contract)
        changed["joints"][0]["takeover_kp_nm_per_rad"] = 61.0
        changed["joints"][0]["policy_kp_nm_per_rad"] += 0.5

        generator._validate(changed)
        self.assertNotEqual(
            generator.render_header(changed), generator.render_header(self.contract)
        )

    def test_g1_hardware_order_and_policy_bijection_are_required(self) -> None:
        changed = copy.deepcopy(self.contract)
        changed["joints"][0]["name"], changed["joints"][1]["name"] = (
            changed["joints"][1]["name"],
            changed["joints"][0]["name"],
        )
        with self.assertRaisesRegex(generator.ContractError, "hardware order"):
            generator._validate(changed)

        changed = copy.deepcopy(self.contract)
        changed["joints"][1]["policy_isaaclab_index"] = changed["joints"][0][
            "policy_isaaclab_index"
        ]
        with self.assertRaisesRegex(generator.ContractError, "duplicated"):
            generator._validate(changed)

    def test_nonfinite_unordered_and_nonpositive_joint_values_are_rejected(self) -> None:
        cases = (
            ("policy_kp_nm_per_rad", float("nan"), "finite"),
            ("policy_kd_nm_s_per_rad", 0.0, "positive"),
            ("plant_effort_limit_nm", -1.0, "positive"),
            ("command_sign", 0.5, "-1 or \\+1"),
        )
        for field, value, message in cases:
            with self.subTest(field=field):
                changed = copy.deepcopy(self.contract)
                changed["joints"][0][field] = value
                with self.assertRaisesRegex(generator.ContractError, message):
                    generator._validate(changed)

        changed = copy.deepcopy(self.contract)
        changed["joints"][0]["position_lower_rad"] = changed["joints"][0][
            "position_upper_rad"
        ]
        with self.assertRaisesRegex(generator.ContractError, "unordered"):
            generator._validate(changed)

        changed = copy.deepcopy(self.contract)
        changed["joints"][0]["default_angle_rad"] = 10.0
        with self.assertRaisesRegex(generator.ContractError, "outside hard limits"):
            generator._validate(changed)

    def test_mode5_wire_profile_and_timing_coherence_are_required(self) -> None:
        for field, value in (
            ("mode_machine", 11),
            ("mode_pr", 1),
            ("active_motor_count", 35),
            ("motor_mode", 0),
            ("writer_rate_hz", 50),
        ):
            with self.subTest(field=field):
                changed = copy.deepcopy(self.contract)
                changed["lowcmd"][field] = value
                with self.assertRaisesRegex(generator.ContractError, "Mode-5 wire"):
                    generator._validate(changed)

        changed = copy.deepcopy(self.contract)
        changed["simulation_timing"]["physics_timestep_seconds"] = 0.001
        with self.assertRaisesRegex(generator.ContractError, "reciprocal"):
            generator._validate(changed)

    def test_live_model_semantics_match_generated_limits_and_fk(self) -> None:
        generator.verify_model_artifacts(self.contract, WBC_ROOT)

        changed = copy.deepcopy(self.contract)
        changed["joints"][0]["position_lower_rad"] += 0.001
        generator._validate(changed)
        with self.assertRaisesRegex(generator.ContractError, "position range differs"):
            generator.verify_model_artifacts(changed, WBC_ROOT)

    def test_model_formatting_changes_do_not_act_as_identity_failures(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_root = Path(temporary_directory)
            for label in ("fk_mjcf", "sim_mjcf", "sim_scene", "sim_config"):
                relative = Path(
                    self.contract["models"][label]["path_from_wbc_root"]
                )
                source = WBC_ROOT / relative
                destination = temporary_root / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)
                if destination.suffix == ".xml":
                    with destination.open("a", encoding="utf-8") as stream:
                        stream.write("\n<!-- local formatting change -->\n")
                else:
                    with destination.open("a", encoding="utf-8") as stream:
                        stream.write("\n# local formatting change\n")

            generator.verify_model_artifacts(self.contract, temporary_root)

    def test_duplicate_keys_nonfinite_json_and_path_traversal_are_rejected(self) -> None:
        duplicate = CONTRACT_PATH.read_text(encoding="utf-8").replace(
            '"schema": "motionlcm.g1.deployment-contract.v1",',
            '"schema": "motionlcm.g1.deployment-contract.v1",\n'
            '  "schema": "motionlcm.g1.deployment-contract.v1",',
            1,
        )
        with self.assertRaisesRegex(generator.ContractError, "duplicate JSON key"):
            json.loads(
                duplicate,
                object_pairs_hook=generator._reject_duplicates,
                parse_constant=generator._reject_constant,
            )
        with self.assertRaisesRegex(generator.ContractError, "invalid JSON numeric"):
            json.loads('{"x":NaN}', parse_constant=generator._reject_constant)

        changed = copy.deepcopy(self.contract)
        changed["models"]["fk_mjcf"]["path_from_wbc_root"] = "../outside.xml"
        with self.assertRaisesRegex(generator.ContractError, "relative path"):
            generator._validate(changed)

    def test_simulate_dt_parser_requires_one_finite_top_level_value(self) -> None:
        self.assertEqual(generator._parse_unique_simulate_dt(b"SIMULATE_DT: 0.002\n"), 0.002)
        with self.assertRaisesRegex(generator.ContractError, "exactly once"):
            generator._parse_unique_simulate_dt(
                b"SIMULATE_DT: 0.002\nSIMULATE_DT: 0.002\n"
            )
        with self.assertRaisesRegex(generator.ContractError, "top-level"):
            generator._parse_unique_simulate_dt(b"  SIMULATE_DT: 0.002\n")
        with self.assertRaisesRegex(generator.ContractError, "finite and positive"):
            generator._parse_unique_simulate_dt(b"SIMULATE_DT: inf\n")

    def test_mjcf_kinematic_vectors_must_be_finite(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            model = Path(temporary_directory) / "nonfinite.xml"
            model.write_text(
                '<mujoco><worldbody><body name="pelvis" pos="nan 0 0">'
                '<joint name="floating_base_joint" type="free"/>'
                "</body></worldbody></mujoco>",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(generator.ContractError, "must be finite"):
                generator._mjcf_kinematics_records(model)


if __name__ == "__main__":
    unittest.main()
