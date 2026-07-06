"""
pca9685_arm_robot.py
─────────────────────────────────────────────────────────────────────────────
ros2_control SystemInterface plugin for arm_robot_pkg (5-DOF arm + parallel
gripper with mimic joints).

Robot joint mapping (from URDF ros2_control block):
┌──────────────────────┬─────┬──────────────────────────────┬──────────────┐
│ ros2_control          │ ch  │ URDF kinematic joint          │ limits (rad) │
│ joint name            │     │                               │              │
├──────────────────────┼─────┼──────────────────────────────┼──────────────┤
│ rotating_base_joint   │  2  │ rotating_base_joint           │ ±3.14        │
│ shoulder_joint        │  0  │ shoulder_joint                │ ±1.57        │
│ elbow_joint           │  1  │ elbow_joint                   │ ±1.57        │
│ wrist_joint           │  3  │ wrist_joint                   │ ±1.57        │
│ gripper_joint         │  4  │ gripper_joint (body rotation) │ ±1.57        │
│ right_gear_joint      │  5  │ right_gear_joint (finger act.)│ ±1.57        │
├──────────────────────┼─────┼──────────────────────────────┼──────────────┤
│ right_link_joint      │ N/A │ mimic → right_gear_joint      │ ±1.57        │
│ left_link_joint       │ N/A │ mimic → right_gear_joint      │ ±1.57        │
│ left_gear_joint       │ N/A │ mimic → right_gear_joint      │ ±1.57        │
│ right_finger_joint    │ N/A │ mimic → right_gear_joint      │ ±1.57        │
│ left_finger_joint     │ N/A │ mimic → right_gear_joint      │ ±1.57        │
└──────────────────────┴─────┴──────────────────────────────┴──────────────┘

Mimic joint architecture:
    right_gear_joint is the single physical actuator that opens/closes the
    parallel gripper.  All five mimic joints (right_link, left_link,
    left_gear, right_finger, left_finger) are kinematic copies driven by
    robot_state_publisher using the <mimic> URDF tag.
    They do NOT need separate PCA9685 channels — only right_gear_joint
    drives a real servo (channel 5).  The mimic joints are registered in
    ros2_control purely so joint_state_broadcaster can publish their states.
    Their state is computed here as:  position = multiplier × right_gear_pos + offset
    (multiplier=1, offset=0 for all five per the URDF).

Hardware chain:
    MoveIt 2 / pick_and_place → ros2_control → this plugin → I²C → PCA9685 → PWM → Servos

Joint angle convention:
    URDF limits are symmetric about 0 (e.g. −1.57 … +1.57).
    0 rad (URDF neutral) → mid pulse = (min_pulse + max_pulse) / 2
    Negative URDF angle → shorter pulse
    Positive URDF angle → longer pulse

    Formula:
        pulse_us = mid_pulse + (radians / urdf_limit_rad) × half_pulse_range
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


# ── Per-joint calibration defaults ────────────────────────────────────────────
#
# Actuated joints  → have a real PCA9685 channel and calibrated pulse range.
# Mimic joints     → channel = -1 (sentinel); pulse params unused.
#                    Their state is computed from right_gear_joint × multiplier.
#
# Calibrate right_gear_joint with:
#   python3 calibrate_servos.py --channel 5
# and update min_pulse_us / max_pulse_us below.
#
_JOINT_DEFAULTS = {
    # ── Actuated joints ───────────────────────────────────────────────────────
    "rotating_base_joint": {
        "channel": 2, "min_pulse_us": 280, "max_pulse_us": 2540,
        "urdf_limit_rad": 3.14, "home_angle_deg": 0.0,
        "is_mimic": False,
    },
    "shoulder_joint": {
        "channel": 0, "min_pulse_us": 1358, "max_pulse_us": 2058,
        "urdf_limit_rad": 1.57, "home_angle_deg": 0.0,
        "is_mimic": False,
    },
    "elbow_joint": {
        "channel": 1, "min_pulse_us": 370, "max_pulse_us": 2630,
        "urdf_limit_rad": 1.57, "home_angle_deg": 0.0,
        "is_mimic": False,
    },
    "wrist_joint": {
        "channel": 3, "min_pulse_us": 530, "max_pulse_us": 2310,
        "urdf_limit_rad": 1.57, "home_angle_deg": 0.0,
        "is_mimic": False,
    },
    "gripper_joint": {
        "channel": 4, "min_pulse_us": 620, "max_pulse_us": 2760,
        "urdf_limit_rad": 1.57, "home_angle_deg": 0.0,
        "is_mimic": False,
    },
    # ── right_gear_joint — PRIMARY gripper actuator ───────────────────────────
    # Drives the parallel finger mechanism via channel 5.
    # All other gripper links mimic this joint (multiplier=1, offset=0).
    # Calibrate with:  python3 calibrate_servos.py --channel 5
    # Update min_pulse_us / max_pulse_us after physical calibration.
    "right_gear_joint": {
        "channel": 5, "min_pulse_us": 620, "max_pulse_us": 2760,
        "urdf_limit_rad": 1.57, "home_angle_deg": 0.0,
        "is_mimic": False,
    },
    # ── Mimic joints — NO servo, state echoes right_gear_joint ───────────────
    # channel = -1  →  write() skips I²C for these joints.
    # mimic_multiplier / mimic_offset match the URDF <mimic> tag values.
    "right_link_joint": {
        "channel": -1, "min_pulse_us": 0, "max_pulse_us": 0,
        "urdf_limit_rad": 1.57, "home_angle_deg": 0.0,
        "is_mimic": True, "mimic_source": "right_gear_joint",
        "mimic_multiplier": 1.0, "mimic_offset": 0.0,
    },
    "left_link_joint": {
        "channel": -1, "min_pulse_us": 0, "max_pulse_us": 0,
        "urdf_limit_rad": 1.57, "home_angle_deg": 0.0,
        "is_mimic": True, "mimic_source": "right_gear_joint",
        "mimic_multiplier": 1.0, "mimic_offset": 0.0,
    },
    "left_gear_joint": {
        "channel": -1, "min_pulse_us": 0, "max_pulse_us": 0,
        "urdf_limit_rad": 1.57, "home_angle_deg": 0.0,
        "is_mimic": True, "mimic_source": "right_gear_joint",
        "mimic_multiplier": 1.0, "mimic_offset": 0.0,
    },
    "right_finger_joint": {
        "channel": -1, "min_pulse_us": 0, "max_pulse_us": 0,
        "urdf_limit_rad": 1.57, "home_angle_deg": 0.0,
        "is_mimic": True, "mimic_source": "right_gear_joint",
        "mimic_multiplier": 1.0, "mimic_offset": 0.0,
    },
    "left_finger_joint": {
        "channel": -1, "min_pulse_us": 0, "max_pulse_us": 0,
        "urdf_limit_rad": 1.57, "home_angle_deg": 0.0,
        "is_mimic": True, "mimic_source": "right_gear_joint",
        "mimic_multiplier": 1.0, "mimic_offset": 0.0,
    },
}

# The single source of truth for joint order — must match the URDF
# ros2_control block declaration order.
JOINT_ORDER = [
    "rotating_base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_joint",
    "gripper_joint",
    "right_gear_joint",   # ← primary finger actuator (ch 5)
    "right_link_joint",   # ← mimic
    "left_link_joint",    # ← mimic
    "left_gear_joint",    # ← mimic
    "right_finger_joint", # ← mimic
    "left_finger_joint",  # ← mimic
]


# ── PCA9685 I²C driver ────────────────────────────────────────────────────────

class PCA9685Driver:
    """Bare-metal I²C driver for PCA9685 (no Adafruit dependency)."""

    def __init__(self, bus_num: int, address: int, frequency: float, logger):
        self._bus_num = bus_num
        self._addr    = address
        self._freq    = frequency
        self._log     = logger
        self._bus     = None

    def open(self) -> bool:
        if not _SMBUS_OK:
            self._log.error(
                "smbus2 not installed.  Run:  pip3 install smbus2 --break-system-packages"
            )
            return False
        try:
            self._bus = smbus2.SMBus(self._bus_num)
            self._swreset()
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
        period_us   = 1_000_000.0 / self._freq
        off_count   = int(round(pulse_us / period_us * _PWM_STEPS))
        off_count   = max(0, min(_PWM_STEPS - 1, off_count))
        base = _LED0_ON_L + 4 * channel
        self._bus.write_i2c_block_data(self._addr, base, [
            0x00, 0x00,
            off_count & 0xFF, (off_count >> 8) & 0x0F,
        ])

    # ── internals ─────────────────────────────────────────────────────────────

    def _swreset(self):
        """General Call software reset — clears stuck EXTCLK/SLEEP state."""
        try:
            self._bus.write_byte_data(0x00, 0x06, 0x00)
            time.sleep(0.01)
        except Exception:
            pass  # NACK on general call is normal on some hosts

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

    def _write(self, reg: int, val: int):
        self._bus.write_byte_data(self._addr, reg, val)

    def _read(self, reg: int) -> int:
        return self._bus.read_byte_data(self._addr, reg)


# ── Per-joint config & conversion ─────────────────────────────────────────────

class JointConfig:
    """
    Stores calibration for one servo joint and converts URDF angles to pulses.

    For mimic joints (is_mimic=True):
        - channel is -1 → no I²C write
        - state is derived from the source joint: pos = multiplier × src + offset
    """

    def __init__(self, name: str, params: dict):
        self.name = name

        defaults = _JOINT_DEFAULTS.get(name, {})

        self.channel         = int(  params.get("channel",          defaults.get("channel",         0)))
        self.min_pulse_us    = float(params.get("min_pulse_us",     defaults.get("min_pulse_us",    500.0)))
        self.max_pulse_us    = float(params.get("max_pulse_us",     defaults.get("max_pulse_us",   2500.0)))
        self.urdf_limit_rad  = float(params.get("urdf_limit_rad",   defaults.get("urdf_limit_rad",  1.57)))
        self.home_angle_deg  = float(params.get("home_angle_deg",   defaults.get("home_angle_deg",  0.0)))
        self.is_mimic        =       params.get("is_mimic",         defaults.get("is_mimic",        False))
        self.mimic_source    =       params.get("mimic_source",     defaults.get("mimic_source",    ""))
        self.mimic_multiplier= float(params.get("mimic_multiplier", defaults.get("mimic_multiplier",1.0)))
        self.mimic_offset    = float(params.get("mimic_offset",     defaults.get("mimic_offset",    0.0)))

        if not self.is_mimic:
            self._mid_pulse  = (self.min_pulse_us + self.max_pulse_us) / 2.0
            self._half_range = (self.max_pulse_us - self.min_pulse_us) / 2.0

    def radians_to_pulse_us(self, radians: float) -> float:
        """Map URDF joint angle (rad) → servo pulse width (µs). Clamps to ±limit."""
        r = max(-self.urdf_limit_rad, min(self.urdf_limit_rad, radians))
        return self._mid_pulse + (r / self.urdf_limit_rad) * self._half_range

    @property
    def home_radians(self) -> float:
        return math.radians(self.home_angle_deg)

    def home_pulse_us(self) -> float:
        return self.radians_to_pulse_us(self.home_radians)

    def mimic_position(self, source_pos: float) -> float:
        """Compute mimic joint position from source joint position."""
        return self.mimic_multiplier * source_pos + self.mimic_offset


# ── ros2_control SystemInterface ──────────────────────────────────────────────

class PCA9685ArmRobot(SystemInterface):
    """
    ros2_control hardware plugin for arm_robot_pkg.

    Actuated joints (ch 0–5) receive I²C writes every control cycle.
    Mimic joints (ch -1) have their state computed from right_gear_joint
    so joint_state_broadcaster publishes correct finger positions and
    robot_state_publisher propagates TF to all finger links.

    Lifecycle:
        on_init       — parse URDF <param> tags, build JointConfig list
        on_activate   — open I²C bus, home all actuated servos
        on_deactivate — park servos at home, close I²C
        read()        — echo cmd → state for actuated joints;
                         compute mimic state from source joint
        write()       — rad → µs for actuated joints only; skip mimic joints
    """

    _LOG = "PCA9685ArmRobot"

    def __init__(self):
        super().__init__()
        self._log    = get_logger(self._LOG)
        self._driver = None
        self._joints: List[JointConfig] = []
        self._cmd:    List[float] = []   # commanded positions (rad)
        self._state:  List[float] = []   # reported positions  (rad)
        self._name_to_idx: dict  = {}    # joint name → list index

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def on_init(self, hardware_info: HardwareInfo) -> return_type:
        if super().on_init(hardware_info) != return_type.OK:
            return return_type.ERROR

        hw = hardware_info.hardware_parameters
        i2c_bus  = int(  hw.get("i2c_bus",      "7"))
        i2c_addr = int(  hw.get("i2c_address",  "0x40"), 16)
        pwm_freq = float(hw.get("pwm_frequency", "50"))

        self._driver = PCA9685Driver(i2c_bus, i2c_addr, pwm_freq, self._log)

        joint_map = {j.name: j for j in hardware_info.joints}

        for name in JOINT_ORDER:
            if name not in joint_map:
                self._log.error(
                    f"on_init FAIL — joint '{name}' not found in URDF "
                    f"ros2_control block.  Found: {list(joint_map.keys())}"
                )
                return return_type.ERROR

            cfg = JointConfig(name, joint_map[name].parameters)
            idx = len(self._joints)
            self._joints.append(cfg)
            self._name_to_idx[name] = idx
            self._cmd.append(cfg.home_radians)
            self._state.append(cfg.home_radians)

            kind = "MIMIC" if cfg.is_mimic else f"ch {cfg.channel}"
            self._log.info(
                f"  [{name}] {kind}  "
                + (f"pulse=[{cfg.min_pulse_us:.0f}, {cfg.max_pulse_us:.0f}] µs  "
                   f"limit=±{cfg.urdf_limit_rad:.3f} rad  "
                   f"home={cfg.home_angle_deg}°"
                   if not cfg.is_mimic else
                   f"← {cfg.mimic_source} × {cfg.mimic_multiplier} + {cfg.mimic_offset}")
            )

        self._log.info(
            f"on_init OK — {len(self._joints)} joints "
            f"({sum(1 for j in self._joints if not j.is_mimic)} actuated, "
            f"{sum(1 for j in self._joints if j.is_mimic)} mimic) | "
            f"I²C bus {i2c_bus} addr 0x{i2c_addr:02X} @ {pwm_freq} Hz"
        )
        return return_type.OK

    def on_activate(self, previous_state) -> return_type:
        if not self._driver.open():
            return return_type.ERROR

        for i, cfg in enumerate(self._joints):
            if cfg.is_mimic:
                continue   # no servo to home
            pulse = cfg.home_pulse_us()
            self._driver.set_pulse_us(cfg.channel, pulse)
            self._cmd[i]   = cfg.home_radians
            self._state[i] = cfg.home_radians
            self._log.info(
                f"  Homing [{cfg.name}] ch {cfg.channel} → "
                f"{cfg.home_angle_deg}° = {pulse:.0f} µs"
            )

        # Initialise mimic states from homed source joints
        self._update_mimic_states()

        time.sleep(0.6)   # let servos reach home before control loop starts
        self._log.info("on_activate OK — all actuated servos at home")
        return return_type.OK

    def on_deactivate(self, previous_state) -> return_type:
        self._log.info("Parking servos at home position...")
        for cfg in self._joints:
            if not cfg.is_mimic:
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
        """
        Only actuated joints expose command interfaces.
        Mimic joints are state-only — controllers must not command them.
        """
        return [
            CommandInterface(cfg.name, "position", self._cmd, i)
            for i, cfg in enumerate(self._joints)
            if not cfg.is_mimic
        ]

    # ── Control loop ──────────────────────────────────────────────────────────

    def read(self, time, period) -> return_type:
        """
        Echo commanded → state for actuated joints.
        Compute mimic joint states from their source joint.
        (No encoders on this arm; replace with real reads if hardware is added.)
        """
        for i, cfg in enumerate(self._joints):
            if not cfg.is_mimic:
                self._state[i] = self._cmd[i]

        self._update_mimic_states()
        return return_type.OK

    def write(self, time, period) -> return_type:
        """
        Convert each actuated joint angle (rad) → PCA9685 pulse (µs) and
        write over I²C.  Mimic joints are skipped — they have no servo.
        """
        for i, cfg in enumerate(self._joints):
            if cfg.is_mimic:
                continue   # no servo, skip
            pulse_us = cfg.radians_to_pulse_us(self._cmd[i])
            try:
                self._driver.set_pulse_us(cfg.channel, pulse_us)
            except Exception as e:
                self._log.error(
                    f"write() I²C error on [{cfg.name}] ch {cfg.channel}: {e}"
                )
                return return_type.ERROR
        return return_type.OK

    # ── helpers ───────────────────────────────────────────────────────────────

    def _update_mimic_states(self):
        """
        Propagate source joint state to all mimic joints.
        Called from both read() and on_activate().
        """
        for i, cfg in enumerate(self._joints):
            if not cfg.is_mimic:
                continue
            src_idx = self._name_to_idx.get(cfg.mimic_source)
            if src_idx is not None:
                self._state[i] = cfg.mimic_position(self._state[src_idx])
                self._cmd[i]   = self._state[i]   # keep cmd consistent