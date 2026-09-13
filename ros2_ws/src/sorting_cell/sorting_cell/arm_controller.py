"""ROS 2 Arm Controller Node for Autonomous Conveyor Sorting Cell.

Subscribes to /arm/command, validates and parses commands against configurable
joint limits, transmits commands to the low-level Arduino controller via serial,
and publishes execution status on /arm/status.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple
import sys

# Conditional import to allow pure Python parsing & unit testing on systems without rclpy
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    RCLPY_AVAILABLE = True
except ImportError:
    rclpy = None
    Node = object
    String = None
    RCLPY_AVAILABLE = False

from sorting_cell.serial_interface import SerialInterface


@dataclass
class JointRange:
    """Configurable angular limits for a single arm joint."""
    min_angle: float
    max_angle: float

    def contains(self, angle: float) -> bool:
        return self.min_angle <= angle <= self.max_angle


class ArmCommandParser:
    """Parses and validates incoming ROS commands against centralized joint limits.

    Converts valid ROS commands into the corresponding serial protocol string.
    Never crashes on malformed or out-of-range input.
    """

    SUPPORTED_JOINTS = ("BASE", "SHOULDER", "ELBOW", "WRIST")
    VALID_GRIPPER_STATES = ("OPEN", "CLOSE")

    def __init__(self, joint_limits: Optional[Dict[str, JointRange]] = None) -> None:
        """Initialize the parser with configurable joint limits.

        CRITICAL SAFETY NOTICE:
        These default values are uncalibrated software test placeholders ONLY.
        They DO NOT represent mechanically safe limits. The hardware team will
        provide actual safe angular limits and calibration values.
        """
        self.joint_limits: Dict[str, JointRange] = joint_limits or {
            "BASE": JointRange(min_angle=0.0, max_angle=180.0),
            "SHOULDER": JointRange(min_angle=15.0, max_angle=165.0),
            "ELBOW": JointRange(min_angle=15.0, max_angle=165.0),
            "WRIST": JointRange(min_angle=0.0, max_angle=180.0),
        }

    def update_joint_limit(self, joint_name: str, min_angle: float, max_angle: float) -> None:
        """Update limit for a specific joint."""
        joint_name = joint_name.upper()
        self.joint_limits[joint_name] = JointRange(min_angle=min_angle, max_angle=max_angle)

    def parse(self, raw_command: str) -> Tuple[bool, Optional[str], Optional[str], Optional[str]]:
        """Parse and validate a raw command string.

        :param raw_command: Command received from /arm/command (e.g. 'BASE 90', 'HOME', 'GRIPPER OPEN')
        :return: Tuple of:
                 - is_valid (bool)
                 - serial_command (str or None)
                 - action_name (str or None, e.g. 'BASE', 'HOME', 'GRIPPER')
                 - error_detail (str or None)
        """
        if not raw_command or not isinstance(raw_command, str):
            return False, None, None, "EMPTY_COMMAND"

        tokens = raw_command.strip().split()
        if not tokens:
            return False, None, None, "EMPTY_COMMAND"

        verb = tokens[0].upper()

        # 1. HOME command
        if verb == "HOME":
            if len(tokens) != 1:
                return False, None, None, "UNEXPECTED_ARGUMENTS_FOR_HOME"
            return True, "HOME", "HOME", None

        # 2. GRIPPER command: GRIPPER OPEN | GRIPPER CLOSE
        if verb == "GRIPPER":
            if len(tokens) != 2:
                return False, None, None, "INVALID_GRIPPER_FORMAT"
            state = tokens[1].upper()
            if state not in self.VALID_GRIPPER_STATES:
                return False, None, None, f"INVALID_GRIPPER_STATE:{tokens[1]}"
            return True, f"GRIPPER:{state}", "GRIPPER", None

        # 3. Joint angle commands: BASE <angle>, SHOULDER <angle>, ELBOW <angle>, WRIST <angle>
        if verb in self.SUPPORTED_JOINTS:
            if len(tokens) != 2:
                return False, None, None, f"MISSING_OR_EXTRA_ANGLE_ARGUMENT_FOR_{verb}"

            raw_angle = tokens[1]
            try:
                angle = float(raw_angle)
            except ValueError:
                return False, None, None, f"NON_NUMERIC_ANGLE:{raw_angle}"

            # Validate against configured limits
            limits = self.joint_limits.get(verb)
            if limits and not limits.contains(angle):
                return False, None, None, (
                    f"ANGLE_OUT_OF_LIMITS:{angle} (allowed: [{limits.min_angle}, {limits.max_angle}])"
                )

            # Format as integer angle in serial protocol (e.g. BASE:90)
            angle_int = int(round(angle))
            return True, f"{verb}:{angle_int}", verb, None

        # 4. Any other unrecognized verb
        return False, None, None, f"UNKNOWN_COMMAND:{verb}"


class ArmControllerNode(Node):
    """ROS 2 Node controlling the robotic arm subsystem."""

    def __init__(self) -> None:
        if not RCLPY_AVAILABLE:
            raise RuntimeError("rclpy is not available in this environment")

        super().__init__('arm_controller')

        # Declare configurable parameters
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout', 1.0)

        # Configurable joint limits:
        # CRITICAL SAFETY NOTICE:
        # These default parameter values are uncalibrated software placeholders ONLY.
        # They DO NOT represent mechanically safe limits. The hardware team will
        # provide actual safe limits, home positions, and calibration offsets.
        self.declare_parameter('joint_limits.base.min', 0.0)
        self.declare_parameter('joint_limits.base.max', 180.0)
        self.declare_parameter('joint_limits.shoulder.min', 15.0)
        self.declare_parameter('joint_limits.shoulder.max', 165.0)
        self.declare_parameter('joint_limits.elbow.min', 15.0)
        self.declare_parameter('joint_limits.elbow.max', 165.0)
        self.declare_parameter('joint_limits.wrist.min', 0.0)
        self.declare_parameter('joint_limits.wrist.max', 180.0)

        # Configurable home position placeholders (to be calibrated by hardware team)
        self.declare_parameter('home_positions.base', 90.0)
        self.declare_parameter('home_positions.shoulder', 90.0)
        self.declare_parameter('home_positions.elbow', 90.0)
        self.declare_parameter('home_positions.wrist', 90.0)

        # Retrieve parameters
        self.mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.serial_timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value

        # Initialize joint limits dictionary from parameters
        joint_limits = {
            "BASE": JointRange(
                self.get_parameter('joint_limits.base.min').get_parameter_value().double_value,
                self.get_parameter('joint_limits.base.max').get_parameter_value().double_value,
            ),
            "SHOULDER": JointRange(
                self.get_parameter('joint_limits.shoulder.min').get_parameter_value().double_value,
                self.get_parameter('joint_limits.shoulder.max').get_parameter_value().double_value,
            ),
            "ELBOW": JointRange(
                self.get_parameter('joint_limits.elbow.min').get_parameter_value().double_value,
                self.get_parameter('joint_limits.elbow.max').get_parameter_value().double_value,
            ),
            "WRIST": JointRange(
                self.get_parameter('joint_limits.wrist.min').get_parameter_value().double_value,
                self.get_parameter('joint_limits.wrist.max').get_parameter_value().double_value,
            ),
        }

        self.parser = ArmCommandParser(joint_limits=joint_limits)

        # Initialize serial interface
        self.serial_interface = SerialInterface(
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=self.serial_timeout,
            mock_mode=self.mock_mode,
            logger=lambda msg: self.get_logger().info(msg),
        )

        # ROS 2 Publisher: /arm/status
        self.status_publisher = self.create_publisher(String, '/arm/status', 10)

        # ROS 2 Subscriber: /arm/command
        self.command_subscription = self.create_subscription(
            String,
            '/arm/command',
            self.command_callback,
            10,
        )

        # Connect serial interface (or mock)
        success, status = self.serial_interface.connect()
        if not success:
            self.get_logger().warn(f"Initial serial connection issue: {status}")
            self.publish_status("ERROR:SERIAL_CONNECTION")
        else:
            self.publish_status("IDLE")

        self.get_logger().info(
            f"arm_controller node started (mock_mode={self.mock_mode}, port={self.serial_port})"
        )

    def publish_status(self, status_text: str) -> None:
        """Publish a status message to /arm/status."""
        msg = String()
        msg.data = status_text
        self.status_publisher.publish(msg)

    def command_callback(self, msg: String) -> None:
        """Process incoming command message from /arm/command."""
        raw_cmd = msg.data.strip()
        self.get_logger().info(f"Received command: {raw_cmd}")

        is_valid, serial_cmd, action_name, error_detail = self.parser.parse(raw_cmd)

        if not is_valid:
            self.get_logger().warn(f"Command validation failed ({error_detail}): '{raw_cmd}'")
            self.publish_status("ERROR:INVALID_COMMAND")
            return

        # Publish executing status
        self.publish_status(f"EXECUTING:{action_name}")

        # Send command over serial (real or mock)
        success, response = self.serial_interface.send_and_receive(serial_cmd)

        if not success:
            self.get_logger().error(f"Serial communication failed: {response}")
            self.publish_status(response)
            return

        # Command succeeded
        self.publish_status(response)


def main(args=None) -> None:
    """Main entry point for arm_controller node."""
    if not RCLPY_AVAILABLE:
        print("ERROR: rclpy is required to run the ROS 2 node.", file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_interface.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
