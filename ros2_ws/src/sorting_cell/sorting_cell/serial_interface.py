"""Serial communication layer for the arm controller subsystem.

Supports both real serial hardware communication (via pyserial) and a robust
mock mode for testing without physical Arduino or PCA9685 hardware connected.
"""

from typing import Callable, Optional, Tuple

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    SERIAL_AVAILABLE = False


class SerialInterface:
    """Manages serial communication with the Arduino arm controller."""

    def __init__(
        self,
        port: str = '/dev/ttyACM0',
        baudrate: int = 115200,
        timeout: float = 1.0,
        mock_mode: bool = False,
        logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        """Initialize the serial interface.

        :param port: Serial device port path (configurable, e.g. /dev/ttyACM0, /dev/ttyUSB0, COM3).
        :param baudrate: Serial baud rate.
        :param timeout: Read timeout in seconds.
        :param mock_mode: If True, simulate communication without opening hardware serial.
        :param logger: Optional logger callable for logging info/debug messages.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.mock_mode = mock_mode
        self._logger = logger or (lambda msg: print(f"[INFO] {msg}"))

        self._ser = None
        self._is_connected = False
        self.last_sent_command: Optional[str] = None

        # Test hooks for simulating error conditions in mock mode
        self.mock_simulate_timeout: bool = False
        self.mock_simulate_disconnect: bool = False

    @property
    def is_connected(self) -> bool:
        """Check whether the interface is currently connected."""
        if self.mock_mode:
            return self._is_connected
        return self._is_connected and self._ser is not None and getattr(self._ser, 'is_open', False)

    def log(self, message: str) -> None:
        """Helper to log messages using the configured logger."""
        self._logger(message)

    def connect(self) -> Tuple[bool, str]:
        """Establish the serial connection or initialize mock mode.

        :return: (success: bool, status_message: str)
        """
        if self.mock_mode:
            self._is_connected = True
            self.log(f"MOCK SERIAL: Initialized on virtual port '{self.port}' (mock_mode=True)")
            return True, "CONNECTED_MOCK"

        if not SERIAL_AVAILABLE:
            self._is_connected = False
            self.log(f"[WARN] SERIAL: pyserial is not installed; hardware port '{self.port}' unavailable")
            return False, "ERROR:SERIAL_UNAVAILABLE"

        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
            self._is_connected = True
            self.log(f"SERIAL: Connected to '{self.port}' at {self.baudrate} baud")
            return True, "CONNECTED"
        except Exception as err:
            self._is_connected = False
            self.log(f"[WARN] SERIAL: Failed to connect to '{self.port}': {err}")
            return False, "ERROR:SERIAL_CONNECTION"

    def disconnect(self) -> None:
        """Close the serial connection if open. Safe to call repeatedly."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._is_connected = False

    def send_and_receive(self, command: str) -> Tuple[bool, str]:
        """Send a newline-terminated command and wait for a response line.

        :param command: Command string to transmit. Newline is appended if absent.
        :return: (success: bool, response_or_error: str)
        """
        clean_cmd = command.strip()
        if not clean_cmd:
            return False, "ERROR:INVALID_COMMAND"

        wire_cmd = clean_cmd + '\n'
        self.last_sent_command = wire_cmd

        if self.mock_mode:
            if not self.is_connected:
                return False, "ERROR:ARDUINO_DISCONNECTED"

            if self.mock_simulate_disconnect:
                self.disconnect()
                return False, "ERROR:ARDUINO_DISCONNECTED"

            if self.mock_simulate_timeout:
                return False, "ERROR:SERIAL_TIMEOUT"

            self.log(f"MOCK SERIAL -> {clean_cmd}")
            response = self._simulate_mock_response(clean_cmd)
            self.log(f"MOCK ARDUINO -> {response}")
            is_success = response.startswith("OK")
            return is_success, response

        # Real hardware serial path
        if not self.is_connected:
            connected, _ = self.connect()
            if not connected:
                return False, "ERROR:ARDUINO_DISCONNECTED"

        try:
            self._ser.reset_input_buffer()
            self._ser.write(wire_cmd.encode('utf-8'))
            self._ser.flush()
        except Exception as err:
            self.log(f"[ERROR] SERIAL: Write failure: {err}")
            self.disconnect()
            return False, "ERROR:ARDUINO_DISCONNECTED"

        try:
            raw_line = self._ser.readline()
        except Exception as err:
            self.log(f"[ERROR] SERIAL: Read failure: {err}")
            self.disconnect()
            return False, "ERROR:ARDUINO_DISCONNECTED"

        if not raw_line:
            return False, "ERROR:SERIAL_TIMEOUT"

        response = raw_line.decode('utf-8', errors='replace').strip()
        if not response:
            return False, "ERROR:SERIAL_TIMEOUT"

        if response.startswith("OK"):
            return True, response
        elif response.startswith("ERROR"):
            return False, response
        else:
            return False, f"ERROR:MALFORMED_RESPONSE:{response}"

    def _simulate_mock_response(self, command: str) -> str:
        """Simulate Arduino protocol responses for mock testing.

        Arduino protocol:
        - HOME -> OK:HOME
        - BASE:<angle> -> OK:BASE
        - SHOULDER:<angle> -> OK:SHOULDER
        - ELBOW:<angle> -> OK:ELBOW
        - WRIST:<angle> -> OK:WRIST
        - GRIPPER:OPEN | GRIPPER:CLOSE -> OK:GRIPPER
        - anything else -> ERROR:INVALID_COMMAND
        """
        if command == "HOME":
            return "OK:HOME"

        if ":" in command:
            target, value = command.split(":", 1)
            target = target.strip().upper()
            value = value.strip().upper()

            if target in ("BASE", "SHOULDER", "ELBOW", "WRIST"):
                try:
                    float(value)
                    return f"OK:{target}"
                except ValueError:
                    return "ERROR:INVALID_COMMAND"

            if target == "GRIPPER":
                if value in ("OPEN", "CLOSE"):
                    return "OK:GRIPPER"
                return "ERROR:INVALID_COMMAND"

        return "ERROR:INVALID_COMMAND"
