"""
pca9685_arm_robot.py
─────────────────────────────────────────────────────────────────────────────
ros2_control SystemInterface plugin for arm_robot_pkg (5-DOF servo arm).

Robot joint mapping (from URDF ros2_control block):
┌─────────────────┬─────┬──────────────────────────────────┬──────────────┐
│ ros2_control     │ ch  │ URDF kinematic joint             │ limits (rad) │
│ joint name       │     │                                  │              │
├─────────────────┼─────┼──────────────────────────────────┼──────────────┤
│ base_joint       │  2  │ rotating_base_joint              │ ±1.046       │
│ shoulder_joint   │  0  │ shoulder_link_joint              │ ±1.046       │
│ elbow_joint      │  1  │ Elbow_link_joint                 │ ±1.046       │
│ wrist_joint      │  3  │ wrist_link_joint                 │ ±1.57        │
│ gripper_joint    │  4  │ gripper_joint                    │ ±1.57        │
└─────────────────┴─────┴──────────────────────────────────┴──────────────┘

Hardware chain:
    MoveIt 2 → ros2_control → this plugin → I²C → PCA9685 → PWM → Servos

URDF ros2_control block (already in your arm_robot_pkg.urdf):
    <ros2_control name="arm_robot_pkg" type="system">
      <hardware>
        <plugin>pca9685_arm_robot/PCA9685ArmRobot</plugin>
        <param name="i2c_bus">7</param>
        <param name="i2c_address">0x40</param>
        <param name="pwm_frequency">50</param>
      </hardware>
      <joint name="base_joint">
        <param name="channel">2</param>
        <param name="min_pulse_us">280</param>
        <param name="max_pulse_us">2540</param>
        <param name="home_angle_deg">0</param>   <!-- 0 = servo mid = URDF 0 rad -->
        <command_interface name="position"/>
        <state_interface   name="position"/>
      </joint>
      ... (see config/arm_robot_pkg.urdf patch below)
    </ros2_control>

Joint angle convention:
    URDF limits are symmetric about 0 (e.g. −1.046 … +1.046).
    Servo pulse is mapped across the full physical range of the servo.
    0 rad (URDF neutral) → mid pulse = (min_pulse + max_pulse) / 2 = 1500 µs
    Negative URDF angle → shorter pulse (< 1500 µs)
    Positive URDF angle → longer pulse  (> 1500 µs)

    Formula (for symmetric ± limits):
        pulse_us = mid_pulse + (radians / max_limit_rad) * half_pulse_range
    where:
        mid_pulse        = (min_pulse_us + max_pulse_us) / 2
        half_pulse_range = (max_pulse_us - min_pulse_us) / 2
        max_limit_rad    = URDF upper limit (positive side)
"""

import math
import time
from typing import List

from rclpy.logging import get_logger
from hardware_interface import SystemInterface, HardwareInfo, return_type
from hardware_interface.hardware_interface import StateInterface, CommandInterface

try:
    import smbus2
    _SMBUS_OK = True
except ImportError:
    _SMBUS_OK = False


# ── PCA9685 register map ──────────────────────────────────────────────────────
_MODE1         = 0x00
_MODE2         = 0x01
_PRE_SCALE     = 0xFE
_LED0_ON_L     = 0x06
_ALL_LED_OFF_H = 0xFD
_MODE1_SLEEP   = 0x10
_MODE1_AI      = 0x20
_MODE1_RESTART = 0x80
_INTERNAL_OSC  = 25_000_000   # Hz
_PWM_STEPS     = 4096          # 12-bit


# ── Default per-joint calibration (update after running calibrate_servos.py) ─
#
# All joints are symmetric about 0 rad in this URDF.
# 0 rad  → 1500 µs (servo centre)
# At lower/upper URDF limit → min/max pulse respectively.
#
# Adjust min_pulse_us / max_pulse_us per servo after physical calibration.
#
_JOINT_DEFAULTS = {
    # name             ch  min_us  max_us  urdf_limit_rad  home_deg
"rotating_base_joint":{ "channel": 2, "min_pulse_us": 280, "max_pulse_us": 2540,
                       "urdf_limit_rad": 1.046, "home_angle_deg": 0.0 },
"shoulder_joint":{ "channel": 0, "min_pulse_us": 1358, "max_pulse_us": 2058,
                       "urdf_limit_rad": 1.046, "home_angle_deg": 0.0 },
"elbow_joint":   { "channel": 1, "min_pulse_us": 370, "max_pulse_us": 2630,
                       "urdf_limit_rad": 1.046, "home_angle_deg": 0.0 },
"wrist_joint":   { "channel": 3, "min_pulse_us": 530, "max_pulse_us": 2310,
                       "urdf_limit_rad": 1.570, "home_angle_deg": 0.0 },
"gripper_joint": { "channel": 4, "min_pulse_us": 620, "max_pulse_us": 2760,
                       "urdf_limit_rad": 1.570, "home_angle_deg": 0.0 },

"right_gear_joint":  {"channel": 5, "min_pulse_us":  620, "max_pulse_us": 2760,
                       "urdf_limit_rad": 1.570, "home_deg": 0.0},    
}


# ── PCA9685 I²C driver ────────────────────────────────────────────────────────

class PCA9685Driver:
    """Bare-metal I²C driver for PCA9685. No Adafruit dependency."""

    def __init__(self, bus_num: int, address: int, frequency: float, logger):
        self._bus_num = bus_num
        self._addr    = address
        self._freq    = frequency
        self._log     = logger
        self._bus     = None

    def open(self) -> bool:
        if not _SMBUS_OK:
            self._log.error(
                "smbus2 not installed. Run: pip3 install smbus2 --break-system-packages"
            )
            return False
        try:
            self._bus = smbus2.SMBus(self._bus_num)
            self._reset()
            self._set_frequency(self._freq)
            self._log.info(
                f"PCA9685 ready — I²C bus {self._bus_num} "
                f"addr 0x{self._addr:02X} @ {self._freq} Hz"
            )
            return True
        except Exception as e:
            self._log.error(f"PCA9685 open error: {e}")
            return False

    def close(self):
        if self._bus:
            try:
                self._write(_ALL_LED_OFF_H, 0x10)   # all channels off
                self._bus.close()
            except Exception:
                pass
            self._bus = None

    def set_pulse_us(self, channel: int, pulse_us: float):
        """Write a microsecond pulse width to one PCA9685 channel."""
        period_us = 1_000_000.0 / self._freq
        off_count = int(round(pulse_us / period_us * _PWM_STEPS))
        off_count = max(0, min(_PWM_STEPS - 1, off_count))
        self._set_pwm(channel, 0, off_count)

    # ── internals ─────────────────────────────────────────────────────────────

    def _reset(self):
        self._write(_MODE1, _MODE1_AI)
        self._write(_MODE2, 0x04)   # totem-pole drive
        time.sleep(0.005)

    def _set_frequency(self, freq: float):
        prescale = int(round(_INTERNAL_OSC / (_PWM_STEPS * freq))) - 1
        prescale = max(3, min(255, prescale))
        old = self._read(_MODE1)
        self._write(_MODE1, (old & 0x7F) | _MODE1_SLEEP)
        self._write(_PRE_SCALE, prescale)
        self._write(_MODE1, old)
        time.sleep(0.005)
        self._write(_MODE1, old | _MODE1_RESTART | _MODE1_AI)

    def _set_pwm(self, channel: int, on: int, off: int):
        base = _LED0_ON_L + 4 * channel
        self._bus.write_i2c_block_data(self._addr, base, [
            on  & 0xFF, (on  >> 8) & 0x0F,
            off & 0xFF, (off >> 8) & 0x0F,
        ])

    def _write(self, reg: int, val: int):
        self._bus.write_byte_data(self._addr, reg, val)

    def _read(self, reg: int) -> int:
        return self._bus.read_byte_data(self._addr, reg)


# ── Per-joint config & conversion ─────────────────────────────────────────────

class JointConfig:
    """
    Stores calibration for one servo joint and converts URDF angles to pulses.

    URDF joints in arm_robot_pkg are symmetric about 0 rad, so the mapping is:
        radians = 0        → mid pulse  (servo centre)
        radians = +limit   → max pulse
        radians = -limit   → min pulse
    """

    def __init__(self, name: str, params: dict):
        self.name = name

        defaults = _JOINT_DEFAULTS.get(name, {})

        self.channel        = int(  params.get("channel",        defaults.get("channel",        0)))
        self.min_pulse_us   = float(params.get("min_pulse_us",   defaults.get("min_pulse_us",   500.0)))
        self.max_pulse_us   = float(params.get("max_pulse_us",   defaults.get("max_pulse_us",  2500.0)))
        self.urdf_limit_rad = float(params.get("urdf_limit_rad", defaults.get("urdf_limit_rad", 1.046)))
        self.home_angle_deg = float(params.get("home_angle_deg", defaults.get("home_angle_deg", 0.0)))

        self._mid_pulse  = (self.min_pulse_us + self.max_pulse_us) / 2.0
        self._half_range = (self.max_pulse_us - self.min_pulse_us) / 2.0

    # ── conversion ────────────────────────────────────────────────────────────

    def radians_to_pulse_us(self, radians: float) -> float:
        """
        Map a signed URDF joint angle (rad) → servo pulse width (µs).

        Clamps to ±urdf_limit_rad before conversion so the servo is never
        driven past its physical stop even if MoveIt 2 sends an out-of-range
        command.
        """
        r = max(-self.urdf_limit_rad, min(self.urdf_limit_rad, radians))
        pulse = self._mid_pulse + (r / self.urdf_limit_rad) * self._half_range
        return pulse

    @property
    def home_radians(self) -> float:
        return math.radians(self.home_angle_deg)

    def home_pulse_us(self) -> float:
        return self.radians_to_pulse_us(self.home_radians)


# ── ros2_control SystemInterface ──────────────────────────────────────────────

class PCA9685ArmRobot(SystemInterface):
    """
    ros2_control hardware plugin for arm_robot_pkg.

    Lifecycle:
        on_init       — parse URDF <param> tags, build JointConfig list
        on_activate   — open I²C bus, home all servos
        on_deactivate — park servos at home, close I²C
        read()        — echo last commanded position (no encoders)
        write()       — convert rad → µs, write to PCA9685
    """

    _LOG = "PCA9685ArmRobot"

    # Expected joint order from the URDF ros2_control block
    JOINT_ORDER = [
        "rotating_base_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist_joint",
        "gripper_joint",
        "right_link_joint",
        "left_link_joint",
        "right_gear_joint",
        "right_finger_joint",
        "left_gear_joint",
        "left_finger_joint"

    ]

    def __init__(self):
        super().__init__()
        self._log    = get_logger(self._LOG)
        self._driver = None
        self._joints: List[JointConfig] = []
        self._cmd: List[float] = []   # commanded positions (rad)
        self._state: List[float] = [] # reported positions  (rad)

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def on_init(self, hardware_info: HardwareInfo) -> return_type:
        if super().on_init(hardware_info) != return_type.OK:
            return return_type.ERROR

        hw = hardware_info.hardware_parameters
        i2c_bus  = int(  hw.get("i2c_bus",      "7"))
        i2c_addr = int(  hw.get("i2c_address",  "0x40"), 16)
        pwm_freq = float(hw.get("pwm_frequency", "50"))

        self._driver = PCA9685Driver(i2c_bus, i2c_addr, pwm_freq, self._log)

        # Build joint list in URDF declaration order
        joint_map = {j.name: j for j in hardware_info.joints}

        for name in self.JOINT_ORDER:
            if name not in joint_map:
                self._log.error(
                    f"on_init FAIL — joint '{name}' not found in URDF "
                    f"ros2_control block. Found: {list(joint_map.keys())}"
                )
                return return_type.ERROR

            cfg = JointConfig(name, joint_map[name].parameters)
            self._joints.append(cfg)
            self._cmd.append(cfg.home_radians)
            self._state.append(cfg.home_radians)

            self._log.info(
                f"  [{name}] ch={cfg.channel}  "
                f"pulse=[{cfg.min_pulse_us:.0f}, {cfg.max_pulse_us:.0f}] µs  "
                f"limit=±{cfg.urdf_limit_rad:.3f} rad  "
                f"home={cfg.home_angle_deg}°"
            )

        self._log.info(
            f"on_init OK — {len(self._joints)} joints | "
            f"I²C bus {i2c_bus} addr 0x{i2c_addr:02X} @ {pwm_freq} Hz"
        )
        return return_type.OK

    def on_activate(self, previous_state) -> return_type:
        if not self._driver.open():
            return return_type.ERROR

        for i, cfg in enumerate(self._joints):
            pulse = cfg.home_pulse_us()
            self._driver.set_pulse_us(cfg.channel, pulse)
            self._cmd[i]   = cfg.home_radians
            self._state[i] = cfg.home_radians
            self._log.info(
                f"  Homing [{cfg.name}] → {cfg.home_angle_deg}° "
                f"= {pulse:.0f} µs  ch {cfg.channel}"
            )

        time.sleep(0.6)   # allow servos to reach home before control loop starts
        self._log.info("on_activate OK — all servos at home")
        return return_type.OK

    def on_deactivate(self, previous_state) -> return_type:
        self._log.info("Parking servos at home position...")
        for cfg in self._joints:
            self._driver.set_pulse_us(cfg.channel, cfg.home_pulse_us())
        time.sleep(0.6)
        self._driver.close()
        self._log.info("on_deactivate OK")
        return return_type.OK

    # ── State interfaces ──────────────────────────────────────────────────────

    def export_state_interfaces(self) -> List[StateInterface]:
        return [
            StateInterface(cfg.name, "position", self._state, i)
            for i, cfg in enumerate(self._joints)
        ]

    # ── Command interfaces ────────────────────────────────────────────────────

    def export_command_interfaces(self) -> List[CommandInterface]:
        return [
            CommandInterface(cfg.name, "position", self._cmd, i)
            for i, cfg in enumerate(self._joints)
        ]

    # ── Control loop ──────────────────────────────────────────────────────────

    def read(self, time, period) -> return_type:
        """
        No position encoders on this arm — echo commanded → state.
        Replace with real encoder reads here if you add feedback hardware.
        """
        for i in range(len(self._joints)):
            self._state[i] = self._cmd[i]
        return return_type.OK

    def write(self, time, period) -> return_type:
        """
        Converts each commanded joint angle (rad) to a PCA9685 pulse (µs)
        and writes it over I²C.  All 5 channels written every control cycle.
        """
        for i, cfg in enumerate(self._joints):
            pulse_us = cfg.radians_to_pulse_us(self._cmd[i])
            try:
                self._driver.set_pulse_us(cfg.channel, pulse_us)
            except Exception as e:
                self._log.error(
                    f"write() I²C error on [{cfg.name}] ch {cfg.channel}: {e}"
                )
                return return_type.ERROR
        return return_type.OK