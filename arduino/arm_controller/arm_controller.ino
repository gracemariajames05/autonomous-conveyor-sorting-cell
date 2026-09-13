/*
 * =====================================================================================
 * Autonomous Conveyor-Based Robotic Sorting Cell
 * Subsystem: Robotic Arm Low-Level Hardware Controller
 * Target: Arduino Uno + Adafruit PCA9685 16-Channel 12-Bit PWM Driver
 *
 * RESPONSIBILITIES:
 * 1. Receive newline-terminated serial commands from ROS 2 arm_controller (or Serial Monitor)
 * 2. Parse and validate commands
 * 3. Communicate with PCA9685 over I2C
 * 4. Drive 6 servos (3x MG996R high torque + 3x MG90S lightweight)
 * 5. Return synchronous OK:<ACTION> / ERROR:INVALID_COMMAND responses
 *
 * =====================================================================================
 * CRITICAL ELECTRICAL & HARDWARE SAFETY REQUIREMENTS:
 * =====================================================================================
 * 1. SEPARATE SERVO POWER:
 *    - NEVER power physical servos directly from the Arduino 5V rail or USB!
 *    - 6 servos under mechanical load will draw up to 5-10A peak, causing Arduino
 *      brownout resets or USB port damage.
 *    - Servos must be powered by an external 5V-6V high-current DC power supply
 *      (e.g., 12V DC mains supply -> high-current buck converter -> PCA9685 V+ screw terminals).
 *    - Arduino USB supplies communication and logic power ONLY.
 *    - Arduino GND MUST be connected to PCA9685 GND (Common Ground).
 *
 * 2. SERVO CALIBRATION & MECHANICAL SAFETY:
 *    - Channel mappings and pulse ranges below are UNCALIBRATED SOFTWARE PLACEHOLDERS.
 *    - The hardware team must calibrate actual mechanical limits and PCA9685 channel mappings
 *      before commanding physical servos.
 *    - Do not assume 0 or 180 degrees are mechanically safe.
 *    - Test physical servos incrementally: 1 servo -> 2 servos -> 3 servos -> 6 servos.
 * =====================================================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize PCA9685 instance at default I2C address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// =====================================================================================
// CONFIGURATION & CALIBRATION (Centralized Constants)
// =====================================================================================

// Serial Baud Rate (must match ROS 2 arm_controller serial_interface baud_rate)
const unsigned long SERIAL_BAUD_RATE = 115200;

// Servo PWM frequency (50 Hz = standard 20ms period for analog & digital RC servos)
const float SERVO_PWM_FREQ = 50.0;

// PCA9685 12-bit PWM pulse width boundaries (4096 counts = 20ms period at 50Hz)
// Standard RC servo pulse: ~1.0ms (min) to ~2.0ms (max), or ~0.5ms to ~2.5ms extended.
// NOTE: These are uncalibrated software defaults; hardware team must calibrate for each joint.
const uint16_t DEFAULT_PULSE_MIN = 102; // ~500 microseconds (placeholder for min angle)
const uint16_t DEFAULT_PULSE_MAX = 512; // ~2500 microseconds (placeholder for max angle)

// PCA9685 Channel Assignments (Software placeholder - hardware team to confirm wiring)
const uint8_t CH_BASE     = 0; // Channel 0: Base rotation (MG996R)
const uint8_t CH_SHOULDER = 1; // Channel 1: Shoulder joint (MG996R)
const uint8_t CH_ELBOW    = 2; // Channel 2: Elbow joint (MG996R)
const uint8_t CH_WRIST    = 3; // Channel 3: Wrist pitch/rotation (MG90S)
const uint8_t CH_GRIPPER  = 4; // Channel 4: Gripper open/close (MG90S)
const uint8_t CH_EXTRA    = 5; // Channel 5: 6th servo placeholder (MG90S wrist roll/spare)

// Configurable Home Position Angles (Software placeholders - to be calibrated by hardware team)
const float HOME_ANGLE_BASE     = 90.0;
const float HOME_ANGLE_SHOULDER = 90.0;
const float HOME_ANGLE_ELBOW    = 90.0;
const float HOME_ANGLE_WRIST    = 90.0;
const float HOME_ANGLE_EXTRA    = 90.0;

// Configurable Gripper Angular Setpoints (Placeholders)
const float GRIPPER_OPEN_ANGLE  = 30.0; // Angle corresponding to open jaws
const float GRIPPER_CLOSE_ANGLE = 90.0; // Angle corresponding to closed jaws

// Serial buffer configuration
const size_t BUFFER_SIZE = 64;
char serialBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;

// =====================================================================================
// PWM CONVERSION & SERVO CONTROL HELPER FUNCTIONS
// =====================================================================================

/**
 * Convert a target angle (0 to 180 deg) to 12-bit PCA9685 PWM count (0 to 4095).
 * Enforces bounds and uses configurable min/max pulse limits.
 */
uint16_t angleToPwmCount(float angle, float minAngle = 0.0, float maxAngle = 180.0,
                         uint16_t minPwm = DEFAULT_PULSE_MIN, uint16_t maxPwm = DEFAULT_PULSE_MAX) {
    if (angle < minAngle) angle = minAngle;
    if (angle > maxAngle) angle = maxAngle;

    float ratio = (angle - minAngle) / (maxAngle - minAngle);
    return (uint16_t)(minPwm + ratio * (maxPwm - minPwm) + 0.5);
}

/**
 * Set a PCA9685 channel to a specific target angle.
 */
void setServoAngle(uint8_t channel, float angle) {
    uint16_t pwmVal = angleToPwmCount(angle);
    pwm.setPWM(channel, 0, pwmVal);
}

/**
 * Move all joints to their configured HOME placeholder positions.
 */
void executeHome() {
    setServoAngle(CH_BASE, HOME_ANGLE_BASE);
    setServoAngle(CH_SHOULDER, HOME_ANGLE_SHOULDER);
    setServoAngle(CH_ELBOW, HOME_ANGLE_ELBOW);
    setServoAngle(CH_WRIST, HOME_ANGLE_WRIST);
    setServoAngle(CH_EXTRA, HOME_ANGLE_EXTRA);
    setServoAngle(CH_GRIPPER, GRIPPER_OPEN_ANGLE);
}

// =====================================================================================
// COMMAND PARSER & DISPATCHER
// =====================================================================================

/**
 * Parses and executes a complete, validated command string.
 * Protocol:
 *   HOME              -> OK:HOME
 *   BASE:<angle>      -> OK:BASE
 *   SHOULDER:<angle>  -> OK:SHOULDER
 *   ELBOW:<angle>     -> OK:ELBOW
 *   WRIST:<angle>     -> OK:WRIST
 *   GRIPPER:OPEN      -> OK:GRIPPER
 *   GRIPPER:CLOSE     -> OK:GRIPPER
 *   <anything else>   -> ERROR:INVALID_COMMAND
 */
void processCommand(char* cmd) {
    // Trim leading whitespace
    while (*cmd == ' ' || *cmd == '\t') {
        cmd++;
    }

    // Trim trailing whitespace
    int len = strlen(cmd);
    while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\t' || cmd[len - 1] == '\r')) {
        cmd[--len] = '\0';
    }

    if (len == 0) {
        Serial.println(F("ERROR:INVALID_COMMAND"));
        return;
    }

    // 1. HOME command
    if (strcmp(cmd, "HOME") == 0) {
        executeHome();
        Serial.println(F("OK:HOME"));
        return;
    }

    // Commands with a colon separator: <TARGET>:<VALUE>
    char* colonPos = strchr(cmd, ':');
    if (colonPos == NULL) {
        Serial.println(F("ERROR:INVALID_COMMAND"));
        return;
    }

    *colonPos = '\0';
    char* target = cmd;
    char* valueStr = colonPos + 1;

    // Validate that value string is not empty
    if (*valueStr == '\0') {
        Serial.println(F("ERROR:INVALID_COMMAND"));
        return;
    }

    // 2. GRIPPER command: GRIPPER:OPEN or GRIPPER:CLOSE
    if (strcmp(target, "GRIPPER") == 0) {
        if (strcmp(valueStr, "OPEN") == 0) {
            setServoAngle(CH_GRIPPER, GRIPPER_OPEN_ANGLE);
            Serial.println(F("OK:GRIPPER"));
            return;
        } else if (strcmp(valueStr, "CLOSE") == 0) {
            setServoAngle(CH_GRIPPER, GRIPPER_CLOSE_ANGLE);
            Serial.println(F("OK:GRIPPER"));
            return;
        } else {
            Serial.println(F("ERROR:INVALID_COMMAND"));
            return;
        }
    }

    // 3. Joint angle commands: BASE, SHOULDER, ELBOW, WRIST
    char* endPtr = NULL;
    float angle = strtod(valueStr, &endPtr);

    // If conversion failed (no digits consumed) or trailing non-whitespace garbage exists
    if (endPtr == valueStr) {
        Serial.println(F("ERROR:INVALID_COMMAND"));
        return;
    }
    // Ensure rest of valueStr is only whitespace
    while (*endPtr != '\0') {
        if (*endPtr != ' ' && *endPtr != '\t') {
            Serial.println(F("ERROR:INVALID_COMMAND"));
            return;
        }
        endPtr++;
    }

    if (strcmp(target, "BASE") == 0) {
        setServoAngle(CH_BASE, angle);
        Serial.println(F("OK:BASE"));
    } else if (strcmp(target, "SHOULDER") == 0) {
        setServoAngle(CH_SHOULDER, angle);
        Serial.println(F("OK:SHOULDER"));
    } else if (strcmp(target, "ELBOW") == 0) {
        setServoAngle(CH_ELBOW, angle);
        Serial.println(F("OK:ELBOW"));
    } else if (strcmp(target, "WRIST") == 0) {
        setServoAngle(CH_WRIST, angle);
        Serial.println(F("OK:WRIST"));
    } else {
        Serial.println(F("ERROR:INVALID_COMMAND"));
    }
}

// =====================================================================================
// ARDUINO MAIN SETUP & LOOP
// =====================================================================================

void setup() {
    // Initialize USB Serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
        ; // Wait for native USB if needed (e.g. Leonardo/Micro; Uno proceeds immediately)
    }

    // Initialize I2C communication and PCA9685 PWM driver
    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000); // Standard internal oscillator frequency (27MHz)
    pwm.setPWMFreq(SERVO_PWM_FREQ);       // 50 Hz servo PWM rate

    // Safety pause after initialization
    delay(10);

    // NOTE: Servos are deliberately NOT homed on powerup to prevent abrupt
    // mechanical movement before explicit command from ROS 2.
}

void loop() {
    // Non-blocking serial reception
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();

        // End of line received (newline)
        if (inChar == '\n') {
            serialBuffer[bufferIndex] = '\0';
            processCommand(serialBuffer);
            bufferIndex = 0; // Reset buffer for next command
        }
        // Discard carriage returns
        else if (inChar == '\r') {
            continue;
        }
        // Append character to buffer with overflow protection
        else {
            if (bufferIndex < BUFFER_SIZE - 1) {
                serialBuffer[bufferIndex++] = inChar;
            } else {
                // Buffer overflow protection: reset and report error
                bufferIndex = 0;
                Serial.println(F("ERROR:INVALID_COMMAND"));
            }
        }
    }
}
