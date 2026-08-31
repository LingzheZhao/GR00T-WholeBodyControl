from __future__ import annotations

import copy
import json
from pathlib import Path
import struct
import tempfile
import unittest

import generate_mode5_contract as generator


CONTROLLER_ROOT = Path(__file__).resolve().parents[1]
DEPLOY_ROOT = CONTROLLER_ROOT.parents[2]
WBC_ROOT = CONTROLLER_ROOT.parents[3]
CONTRACT_PATH = DEPLOY_ROOT / "g1/sonic_g1_mode5_derived_v1.contract.json"
HEADER_PATH = CONTROLLER_ROOT / "include/mode5_contract_generated.hpp"
CONTROLLER_SOURCE = CONTROLLER_ROOT / "src/g1_deploy_onnx_ref.cpp"
TAKEOVER_SOURCE = (
    DEPLOY_ROOT
    / "thirdparty/unitree_sdk2/example/g1/low_level/g1_ankle_swing_example.cpp"
)


class Mode5ContractGeneratorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.contract = generator.load_contract(CONTRACT_PATH)

    def test_canonical_contract_and_generated_header_are_exact(self) -> None:
        self.assertEqual(
            generator.canonical_sha256(self.contract),
            "1ab7b04a98e5e32a15371c215291940d134e9978ad3d35c16f6a74bc964a1420",
        )
        self.assertEqual(
            generator.canonical_json_sha256(self.contract["joints"]),
            "8c46e85155070a359dce3059a86a98b0c7cd8d8b1ba3079e2f4d97a13236237b",
        )
        self.assertEqual(
            HEADER_PATH.read_text(encoding="utf-8"),
            generator.render_header(self.contract),
        )

    def test_all_live_model_and_checkpoint_artifacts_are_pinned(self) -> None:
        generator.verify_model_artifacts(self.contract, WBC_ROOT)
        generator.verify_checkpoint_artifacts(self.contract, DEPLOY_ROOT)
        generator.verify_takeover_source_artifact(
            self.contract, DEPLOY_ROOT, TAKEOVER_SOURCE
        )
        self.assertEqual(
            self.contract["models"]["common_kinematics_sha256"],
            "4b4272979a730962981652897f5035d02abbab9db4197dd29320266f564ffdc5",
        )
        self.assertEqual(
            self.contract["models"]["sim_scene_include"],
            Path(self.contract["models"]["sim_mjcf"]["path_from_wbc_root"]).name,
        )

    def test_simulation_timing_is_exact_and_matches_the_yaml_scalar(self) -> None:
        self.assertEqual(
            self.contract["simulation_timing"],
            {
                "physics_timestep_seconds": 0.002,
                "physics_rate_hz": 500,
                "physics_steps_per_lowcmd_period": 1,
                "torque_application": "external-pd-recomputed-once-per-mujoco-step",
            },
        )
        sim_config = (
            WBC_ROOT
            / self.contract["models"]["sim_config"]["path_from_wbc_root"]
        )
        self.assertEqual(
            generator._parse_unique_simulate_dt(sim_config.read_bytes()), 0.002
        )
        for field, replacement in (
            ("physics_timestep_seconds", 0.005),
            ("physics_rate_hz", 200),
            ("physics_steps_per_lowcmd_period", 2),
            ("torque_application", "implicit-actuator"),
        ):
            with self.subTest(field=field):
                changed = copy.deepcopy(self.contract)
                changed["simulation_timing"][field] = replacement
                with self.assertRaisesRegex(
                    generator.ContractError, "simulation timing differs"
                ):
                    generator._validate(changed)
        with self.assertRaisesRegex(generator.ContractError, "exactly once"):
            generator._parse_unique_simulate_dt(
                b"SIMULATE_DT: 0.002\nSIMULATE_DT: 0.002\n"
            )
        with self.assertRaisesRegex(generator.ContractError, "top-level"):
            generator._parse_unique_simulate_dt(b"  SIMULATE_DT: 0.002\n")

    def test_missing_vendored_takeover_source_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            deploy_root = Path(temporary_directory)
            (deploy_root / "thirdparty/unitree_sdk2").mkdir(parents=True)
            missing = (
                deploy_root
                / "thirdparty/unitree_sdk2"
                / self.contract["sources"]["unitree_takeover_control"]["path"]
            )
            with self.assertRaisesRegex(
                generator.ContractError, "missing takeover source"
            ):
                generator.verify_takeover_source_artifact(
                    self.contract, deploy_root, missing
                )

    def test_wrong_vendored_takeover_source_bytes_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            deploy_root = Path(temporary_directory)
            source = (
                deploy_root
                / "thirdparty/unitree_sdk2"
                / self.contract["sources"]["unitree_takeover_control"]["path"]
            )
            source.parent.mkdir(parents=True)
            source.write_bytes(b"not the reviewed Unitree source\n")
            with self.assertRaisesRegex(
                generator.ContractError, "takeover source SHA-256 mismatch"
            ):
                generator.verify_takeover_source_artifact(
                    self.contract, deploy_root, source
                )

    def test_contract_cannot_silently_change_released_policy_values(self) -> None:
        for field in (
            "policy_kp_nm_per_rad",
            "policy_kd_nm_s_per_rad",
            "policy_action_scale_rad",
            "default_angle_rad",
            "command_sign",
            "position_upper_rad",
            "plant_velocity_limit_rad_s",
            "passive_damping_nm_s_per_rad",
            "takeover_kp_nm_per_rad",
            "takeover_kd_nm_s_per_rad",
        ):
            with self.subTest(field=field):
                changed = copy.deepcopy(self.contract)
                changed["joints"][0][field] += 0.001
                with self.assertRaisesRegex(
                    generator.ContractError,
                    "joint parameters differ|command_sign must remain|takeover K[dp] schedule differs",
                ):
                    generator._validate(changed)

    def test_takeover_schedule_and_unitree_source_are_exact(self) -> None:
        expected_kp = (
            60.0, 60.0, 60.0, 100.0, 40.0, 40.0,
            60.0, 60.0, 60.0, 100.0, 40.0, 40.0,
            60.0, 40.0, 40.0,
            40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
            40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
        )
        expected_kd = (
            1.0, 1.0, 1.0, 2.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 2.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        )
        self.assertEqual(
            tuple(row["takeover_kp_nm_per_rad"] for row in self.contract["joints"]),
            expected_kp,
        )
        self.assertEqual(
            tuple(row["takeover_kd_nm_s_per_rad"] for row in self.contract["joints"]),
            expected_kd,
        )
        self.assertEqual(
            {row["takeover_gain_basis"] for row in self.contract["joints"]},
            {"unitree-sdk2-g1-ankle-swing-example"},
        )
        self.assertEqual(self.contract["takeover_control"]["duration_seconds"], 3.0)
        self.assertEqual(
            self.contract["sources"]["unitree_takeover_control"],
            {
                "repository": "unitreerobotics/unitree_sdk2",
                "revision": "9754cd153af3da471b0fe5f3aa535e426fb11db3",
                "path": "example/g1/low_level/g1_ankle_swing_example.cpp",
                "sha256": "9394824751ee4f74d252546fe4f3c20145f9148becfbb9eeb53cbeef449c6b8b",
            },
        )

    def test_full_released_gain_and_action_arrays_are_bit_exact(self) -> None:
        expected_kp = (
            0x42C63265, 0x42C63265, 0x4220B78A, 0x42C63265, 0x41E4028D,
            0x41E4028D, 0x42C63265, 0x42C63265, 0x4220B78A, 0x42C63265,
            0x41E4028D, 0x41E4028D, 0x4220B78A, 0x41E4028D, 0x41E4028D,
            0x4164028D, 0x4164028D, 0x4164028D, 0x4164028D, 0x4164028D,
            0x41863A04, 0x41863A04, 0x4164028D, 0x4164028D, 0x4164028D,
            0x4164028D, 0x4164028D, 0x41863A04, 0x41863A04,
        )
        expected_kd = (
            0x40C9E1B4, 0x40C9E1B4, 0x4023B477, 0x40C9E1B4, 0x3FE83FC2,
            0x3FE83FC2, 0x40C9E1B4, 0x40C9E1B4, 0x4023B477, 0x40C9E1B4,
            0x3FE83FC2, 0x3FE83FC2, 0x4023B477, 0x3FE83FC2, 0x3FE83FC2,
            0x3F683FC2, 0x3F683FC2, 0x3F683FC2, 0x3F683FC2, 0x3F683FC2,
            0x3F88B8DC, 0x3F88B8DC, 0x3F683FC2, 0x3F683FC2, 0x3F683FC2,
            0x3F683FC2, 0x3F683FC2, 0x3F88B8DC, 0x3F88B8DC,
        )
        expected_scale = (
            0x3FD6713CCA842003, 0x3FD6713CCA842003, 0x3FE185802A245631,
            0x3FD6713CCA842003, 0x3FDC11A695046077, 0x3FDC11A695046077,
            0x3FD6713CCA842003, 0x3FD6713CCA842003, 0x3FE185802A245631,
            0x3FD6713CCA842003, 0x3FDC11A695046077, 0x3FDC11A695046077,
            0x3FE185802A245631, 0x3FDC11A695046077, 0x3FDC11A695046077,
            0x3FDC11A695046077, 0x3FDC11A695046077, 0x3FDC11A695046077,
            0x3FDC11A695046077, 0x3FDC11A695046077, 0x3FB3127D3196B9A0,
            0x3FB3127D3196B9A0, 0x3FDC11A695046077, 0x3FDC11A695046077,
            0x3FDC11A695046077, 0x3FDC11A695046077, 0x3FDC11A695046077,
            0x3FB3127D3196B9A0, 0x3FB3127D3196B9A0,
        )
        actual_kp = tuple(
            struct.unpack("<I", struct.pack("<f", row["policy_kp_nm_per_rad"]))[0]
            for row in self.contract["joints"]
        )
        actual_kd = tuple(
            struct.unpack("<I", struct.pack("<f", row["policy_kd_nm_s_per_rad"]))[0]
            for row in self.contract["joints"]
        )
        actual_scale = tuple(
            struct.unpack("<Q", struct.pack("<d", row["policy_action_scale_rad"]))[0]
            for row in self.contract["joints"]
        )
        self.assertEqual(actual_kp, expected_kp)
        self.assertEqual(actual_kd, expected_kd)
        self.assertEqual(actual_scale, expected_scale)

    def test_duplicate_keys_and_nonfinite_numbers_are_rejected(self) -> None:
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
            json.loads("{\"x\":NaN}", parse_constant=generator._reject_constant)

    def test_controller_uses_contract_before_init_and_control_commands(self) -> None:
        source = CONTROLLER_SOURCE.read_text(encoding="utf-8")
        invocation = source[
            source.index("void ValidateInvocationProfileOrThrow(") :
            source.index("constexpr int kEncoderModeNeverLogged")
        ]
        simulation_branch = invocation.index("if (simulation_only)")
        self.assertLess(
            invocation.index("VerifyReleasedProfileArtifactsOrThrow("),
            simulation_branch,
        )
        self.assertLess(
            invocation.index('input_type != "zmq"'), simulation_branch
        )
        self.assertLess(
            invocation.index(
                "enable_dex3_hands || !command_max_delta_rad.has_value()"
            ),
            simulation_branch,
        )
        self.assertLess(
            invocation.index("if (simulation_study_disable_command_q_clamp)"),
            simulation_branch,
        )
        self.assertIn(
            "physical runtime requires --enable-command-q-clamp", invocation
        )
        self.assertIn("if (commandMaxDeltaRadSeen)", source)
        self.assertIn(
            "--command-max-delta-rad may be specified only once", source
        )
        self.assertIn(r'\"lowcmd_writer_rate_hz\":', source)
        self.assertIn("sonic::mode5_contract::kWriterRateHz", source)
        takeover = source[
            source.index("void WaitForFreshAuthorizedRobotStateOrThrow()") :
            source.index("void WaitForManualTakeoverAuthorizationOrThrow()")
        ]
        for required in (
            "motor.motorstate() != 0",
            "G1_JOINT_POSITION_LOWER_LIMITS[i]",
            "G1_JOINT_POSITION_UPPER_LIMITS[i]",
            "G1_JOINT_VELOCITY_LIMITS[i]",
        ):
            self.assertIn(required, takeover)
        init = source[
            source.index("bool InitControl()") :
            source.index("bool CheckSafety(const RobotStateSnapshot& snapshot)")
        ]
        self.assertEqual(init.count("CaptureRobotStateSnapshot()"), 1)
        self.assertLess(
            init.index("if (!CheckSafety(snapshot))"),
            init.index("MotorCommand motor_command_tmp"),
        )
        self.assertLess(
            init.index("if (!CheckSafety(snapshot))"),
            init.index("init_start_q_[i]"),
        )

        safety = source[
            source.index("bool CheckSafety(const RobotStateSnapshot& snapshot)") :
            source.index("double GetRosTimestamp()")
        ]
        self.assertNotIn("GetDataWithTime()", safety)
        for required in (
            "mode_machine() != required_mode_machine_",
            "mode_pr() != kRequiredModePr",
            "state.motorstate() != 0",
            "G1_JOINT_POSITION_LOWER_LIMITS",
            "G1_JOINT_POSITION_UPPER_LIMITS",
            "G1_JOINT_VELOCITY_LIMITS",
            "HIGH_TEMP_ENTER",
        ):
            self.assertIn(required, safety)

        gather = source[
            source.index(
                "bool GatherRobotStateToLogger(const RobotStateSnapshot& snapshot)"
            ) :
            source.index("bool GatherInputInterfaceData()")
        ]
        self.assertNotIn("GetDataWithTime()", gather)
        self.assertIn("G1_JOINT_VELOCITY_LIMITS[hardware_index]", gather)
        self.assertNotIn("&& !disable_crc_check_", gather)
        self.assertNotIn("> 35.0", gather)

    def test_stale_deploy_mjcf_is_not_a_cpp_profile_source(self) -> None:
        sources = list((CONTROLLER_ROOT / "include").glob("*.hpp"))
        sources += list((CONTROLLER_ROOT / "src").glob("*.cpp"))
        sources += list((CONTROLLER_ROOT / "unit_tests").glob("*.cpp"))
        stale = "g1/" + "g1_29dof.xml"
        offenders = [
            str(path.relative_to(CONTROLLER_ROOT))
            for path in sources
            if stale in path.read_text(encoding="utf-8", errors="replace")
        ]
        self.assertEqual(offenders, [])


if __name__ == "__main__":
    unittest.main()
