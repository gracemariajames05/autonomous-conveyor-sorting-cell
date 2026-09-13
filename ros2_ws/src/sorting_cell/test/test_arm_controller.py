"""Unit tests for the sorting_cell arm controller subsystem.

Tests command parsing, validation against configurable limits, malformed command handling,
and mock serial communication without requiring physical hardware or ROS middleware.
"""

import sys
from pathlib import Path
import pytest

# Ensure sorting_cell is on PYTHONPATH
pkg_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(pkg_root))

from sorting_cell.arm_controller import ArmCommandParser, JointRange
from sorting_cell.serial_interface import SerialInterface


class TestArmCommandParserValid:
    """Test valid command parsing and conversion to the serial protocol."""

    def setup_method(self):
        self.parser = ArmCommandParser()

    @pytest.mark.parametrize("command,expected_serial,expected_action", [
        ("HOME", "HOME", "HOME"),
        ("home", "HOME", "HOME"),
        ("BASE 90", "BASE:90", "BASE"),
        ("base 0", "BASE:0", "BASE"),
        ("BASE 180", "BASE:180", "BASE"),
        ("SHOULDER 60", "SHOULDER:60", "SHOULDER"),
        ("ELBOW 120", "ELBOW:120", "ELBOW"),
        ("WRIST 90", "WRIST:90", "WRIST"),
        ("wrist 180", "WRIST:180", "WRIST"),
        ("  BASE   90  ", "BASE:90", "BASE"),
        ("BASE 90.0", "BASE:90", "BASE"),
        ("ELBOW 120.4", "ELBOW:120", "ELBOW"),
        ("GRIPPER OPEN", "GRIPPER:OPEN", "GRIPPER"),
        ("gripper open", "GRIPPER:OPEN", "GRIPPER"),
        ("GRIPPER CLOSE", "GRIPPER:CLOSE", "GRIPPER"),
        ("gripper close", "GRIPPER:CLOSE", "GRIPPER"),
    ])
    def test_valid_commands(self, command, expected_serial, expected_action):
        is_valid, serial_cmd, action_name, error = self.parser.parse(command)
        assert is_valid is True
        assert serial_cmd == expected_serial
        assert action_name == expected_action
        assert error is None


class TestArmCommandParserInvalid:
    """Test malformed and out-of-limits commands are rejected without crashing."""

    def setup_method(self):
        self.parser = ArmCommandParser()

    @pytest.mark.parametrize("command,expected_error_substr", [
        ("BASE", "MISSING_OR_EXTRA_ANGLE_ARGUMENT"),
        ("SHOULDER", "MISSING_OR_EXTRA_ANGLE_ARGUMENT"),
        ("ELBOW", "MISSING_OR_EXTRA_ANGLE_ARGUMENT"),
        ("WRIST", "MISSING_OR_EXTRA_ANGLE_ARGUMENT"),
        ("BASE 999", "ANGLE_OUT_OF_LIMITS"),
        ("BASE -50", "ANGLE_OUT_OF_LIMITS"),
        ("BASE hello", "NON_NUMERIC_ANGLE"),
        ("ELBOW abc", "NON_NUMERIC_ANGLE"),
        ("SHOULDER 10", "ANGLE_OUT_OF_LIMITS"),  # default min is 15.0
        ("SHOULDER 170", "ANGLE_OUT_OF_LIMITS"), # default max is 165.0
        ("UNKNOWN", "UNKNOWN_COMMAND"),
        ("FOO BAR", "UNKNOWN_COMMAND"),
        ("GRIPPER 123", "INVALID_GRIPPER_STATE"),
        ("GRIPPER", "INVALID_GRIPPER_FORMAT"),
        ("GRIPPER OPEN CLOSE", "INVALID_GRIPPER_FORMAT"),
        ("HOME extra", "UNEXPECTED_ARGUMENTS"),
        ("", "EMPTY_COMMAND"),
        ("   ", "EMPTY_COMMAND"),
    ])
    def test_invalid_commands(self, command, expected_error_substr):
        is_valid, serial_cmd, action_name, error = self.parser.parse(command)
        assert is_valid is False
        assert serial_cmd is None
        assert action_name is None
        assert error is not None
        assert expected_error_substr in error

    def test_non_string_command(self):
        is_valid, _, _, error = self.parser.parse(None)
        assert is_valid is False
        assert error == "EMPTY_COMMAND"


class TestConfigurableLimits:
    """Verify that servo limits remain centralized and configurable."""

    def test_custom_limits_enforced(self):
        custom_limits = {
            "BASE": JointRange(min_angle=45.0, max_angle=135.0),
            "SHOULDER": JointRange(min_angle=30.0, max_angle=90.0),
            "ELBOW": JointRange(min_angle=45.0, max_angle=120.0),
            "WRIST": JointRange(min_angle=10.0, max_angle=80.0),
        }
        parser = ArmCommandParser(joint_limits=custom_limits)

        # Inside custom limits
        assert parser.parse("BASE 90")[0] is True
        assert parser.parse("BASE 45")[0] is True
        assert parser.parse("BASE 135")[0] is True

        # Outside custom limits
        assert parser.parse("BASE 30")[0] is False
        assert parser.parse("BASE 140")[0] is False
        assert parser.parse("BASE 0")[0] is False
        assert parser.parse("BASE 180")[0] is False

    def test_runtime_limit_update(self):
        parser = ArmCommandParser()
        assert parser.parse("BASE 10")[0] is True

        # Restrict base to min 30
        parser.update_joint_limit("BASE", min_angle=30.0, max_angle=150.0)
        assert parser.parse("BASE 10")[0] is False
        assert parser.parse("BASE 90")[0] is True


class TestMockSerialInterface:
    """Verify mock serial communication behaves according to specification."""

    def setup_method(self):
        self.serial = SerialInterface(port='/dev/ttyACM0', mock_mode=True)
        connected, status = self.serial.connect()
        assert connected is True
        assert status == "CONNECTED_MOCK"

    def teardown_method(self):
        self.serial.disconnect()

    @pytest.mark.parametrize("cmd,expected_response", [
        ("HOME", "OK:HOME"),
        ("BASE:90", "OK:BASE"),
        ("SHOULDER:60", "OK:SHOULDER"),
        ("ELBOW:120", "OK:ELBOW"),
        ("WRIST:90", "OK:WRIST"),
        ("GRIPPER:OPEN", "OK:GRIPPER"),
        ("GRIPPER:CLOSE", "OK:GRIPPER"),
    ])
    def test_mock_valid_responses(self, cmd, expected_response):
        success, response = self.serial.send_and_receive(cmd)
        assert success is True
        assert response == expected_response

    @pytest.mark.parametrize("bad_cmd", [
        ("UNKNOWN:123"),
        ("INVALID"),
        ("GRIPPER:MAYBE"),
        ("BASE:NOT_A_NUMBER"),
    ])
    def test_mock_invalid_responses(self, bad_cmd):
        success, response = self.serial.send_and_receive(bad_cmd)
        assert success is False
        assert response == "ERROR:INVALID_COMMAND"
