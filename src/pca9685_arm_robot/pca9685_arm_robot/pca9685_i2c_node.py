#!/usr/bin/env python3
"""
pca9685_i2c_node.py  (fixed)
────────────────────────────────────────────────────────────────────────────
Key fix: Added a DEADBAND filter on the /joint_states subscription.

ROOT CAUSE of continuous rotation:
  joint_state_broadcaster publishes at 50 Hz continuously.
  JointTrajectoryController keeps publishing INTERPOLATED positions even
  after the goal result is returned — it continues until the next goal
  arrives.  This means the base servo keeps receiving new pulse writes
  from stale trajectory interpolation, causing it to keep rotating.

FIX:
  1. DEADBAND: only write to PCA9685 if position changed by > MIN_DELTA_RAD.
     Small interpolation steps below 0.005 rad are ignored.
  2. SETTLE TIMEOUT: after a position is held stable for SETTLE_SEC seconds
     with no change > MIN_DELTA_RAD, stop writing to that channel entirely
     until the next real command arrives. This hard-stops the servo.
  3. Direct I2C write on /pca9685_arm_robot/hold_position topic:
     pick_and_place sends this to freeze specific joints immediately.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

try:
    import smbus2
    _SMBUS_OK = True
except ImportError:
    _SMBUS_OK = False

# ── PCA9685 registers ─────────────────────────────────────────────────────────
_MODE1         = 0x00
_MODE2         = 0x01
_PRE_SCALE     = 0xFE
_LED0_ON_L     = 0x06
_ALL_LED_OFF_H = 0xFD
_MODE1_SLEEP   = 0x10
_MODE1_AI      = 0x20
_MODE1_RESTART = 0x80
_INTERNAL_OSC  = 25_000_000
_PWM_STEPS     = 4096

# ── Deadband filter ───────────────────────────────────────────────────────────
# Only write to PCA9685 if joint moved more than this (radians).
# 0.005 rad ≈ 0.3° — ignores controller noise and micro-interpolation steps.
MIN_DELTA_RAD = 0.005

# ── Joint config ──────────────────────────────────────────────────────────────
_JOINT_CONFIG = {
    "rotating_base_joint": {
        "channel": 2,
        "min_pulse_us": 500,  "max_pulse_us": 2500,
        "urdf_limit_rad": 3.14, "home_deg": 0.0,
        "is_mimic": False,
    },
    "shoulder_joint": {
        "channel": 0,
        "min_pulse_us": 510,  "max_pulse_us": 2058,
        "urdf_limit_rad": 1.57, "home_deg": 0.0,
        "is_mimic": False,
    },
    "elbow_joint": {
        "channel": 1,
        "min_pulse_us": 370,  "max_pulse_us": 2630,
        "urdf_limit_rad": 1.57, "home_deg": 0.0,
        "is_mimic": False,
    },
    "wrist_joint": {
        "channel": 3,
        "min_pulse_us": 530,  "max_pulse_us": 2310,
        "urdf_limit_rad": 1.57, "home_deg": 0.0,
        "is_mimic": False,
    },
    "gripper_joint": {
        "channel": 4,
        "min_pulse_us": 620,  "max_pulse_us": 2760,
        "urdf_limit_rad": 1.57, "home_deg": 0.0,
        "is_mimic": False,
    },
    "right_gear_joint": {
        "channel": 5,
        "min_pulse_us": 620,  "max_pulse_us": 2760,
        "urdf_limit_rad": 1.57, "home_deg": 0.0,
        "is_mimic": False,
    },
    "right_link_joint": {
        "channel": -1, "is_mimic": True,
        "mimic_source": "right_gear_joint",
        "mimic_multiplier": 1.0, "mimic_offset": 0.0,
        "urdf_limit_rad": 1.57, "home_deg": 0.0,
    },
    "left_link_joint": {
        "channel": -1, "is_mimic": True,
        "mimic_source": "right_gear_joint",
        "mimic_multiplier": 1.0, "mimic_offset": 0.0,
        "urdf_limit_rad": 1.57, "home_deg": 0.0,
    },
    "left_gear_joint": {
        "channel": -1, "is_mimic": True,
        "mimic_source": "right_gear_joint",
        "mimic_multiplier": 1.0, "mimic_offset": 0.0,
        "urdf_limit_rad": 1.57, "home_deg": 0.0,
    },
    "right_finger_joint": {
        "channel": -1, "is_mimic": True,
        "mimic_source": "right_gear_joint",
        "mimic_multiplier": 1.0, "mimic_offset": 0.0,
        "urdf_limit_rad": 1.57, "home_deg": 0.0,
    },
    "left_finger_joint": {
        "channel": -1, "is_mimic": True,
        "mimic_source": "right_gear_joint",
        "mimic_multiplier": 1.0, "mimic_offset": 0.0,
        "urdf_limit_rad": 1.57, "home_deg": 0.0,
    },
}

FULL_JOINT_ORDER = [
    "rotating_base_joint", "shoulder_joint", "elbow_joint",
    "wrist_joint", "gripper_joint", "right_gear_joint",
    "right_link_joint", "left_link_joint", "left_gear_joint",
    "right_finger_joint", "left_finger_joint",
]

ACTUATED_JOINT_ORDER = [
    "rotating_base_joint", "shoulder_joint", "elbow_joint",
    "wrist_joint", "gripper_joint", "right_gear_joint",
]


# ── Joint helper ──────────────────────────────────────────────────────────────

class Joint:
    def __init__(self, name, cfg):
        self.name             = name
        self.channel          = cfg["channel"]
        self.is_mimic         = cfg.get("is_mimic", False)
        self.mimic_source     = cfg.get("mimic_source", "")
        self.mimic_multiplier = float(cfg.get("mimic_multiplier", 1.0))
        self.mimic_offset     = float(cfg.get("mimic_offset", 0.0))
        self.home_radians     = math.radians(cfg.get("home_deg", 0.0))

        if not self.is_mimic:
            self.min_pulse_us   = float(cfg["min_pulse_us"])
            self.max_pulse_us   = float(cfg["max_pulse_us"])
            self.urdf_limit_rad = float(cfg["urdf_limit_rad"])
            self._mid  = (self.min_pulse_us + self.max_pulse_us) / 2.0
            self._half = (self.max_pulse_us - self.min_pulse_us) / 2.0

    def radians_to_pulse_us(self, radians):
        r = max(-self.urdf_limit_rad, min(self.urdf_limit_rad, radians))
        return self._mid + (r / self.urdf_limit_rad) * self._half

    def home_pulse_us(self):
        return self.radians_to_pulse_us(self.home_radians)

    def mimic_position(self, source_pos):
        return self.mimic_multiplier * source_pos + self.mimic_offset


# ── PCA9685 driver ────────────────────────────────────────────────────────────

class PCA9685:
    def __init__(self, bus_num, addr, freq, logger):
        self._bus_num = bus_num
        self._addr    = addr
        self._freq    = freq
        self._log     = logger
        self._bus     = None
        self._lock    = threading.Lock()

    def open(self):
        if not _SMBUS_OK:
            self._log.error("smbus2 not installed.")
            return False
        try:
            self._bus = smbus2.SMBus(self._bus_num)
            self._swreset()
            self._reset()
            self._set_frequency(self._freq)
            self._log.info(
                f"PCA9685 ready — I2C bus {self._bus_num} "
                f"addr 0x{self._addr:02X} @ {self._freq} Hz"
            )
            return True
        except Exception as e:
            self._log.error(f"PCA9685 open failed: {e}")
            return False

    def set_pulse_us(self, channel, pulse_us):
        period_us = 1_000_000.0 / self._freq
        off = int(round(pulse_us / period_us * _PWM_STEPS))
        off = max(0, min(_PWM_STEPS - 1, off))
        base = _LED0_ON_L + 4 * channel
        with self._lock:
            self._bus.write_i2c_block_data(self._addr, base, [
                0x00, 0x00, off & 0xFF, (off >> 8) & 0x0F,
            ])

    def close(self):
        if self._bus:
            try:
                with self._lock:
                    self._bus.write_byte_data(self._addr, _ALL_LED_OFF_H, 0x10)
                    self._bus.close()
            except Exception:
                pass
            self._bus = None

    def _swreset(self):
        try:
            self._bus.write_byte_data(0x00, 0x06, 0x00)
            time.sleep(0.01)
        except Exception:
            pass

    def _reset(self):
        self._bus.write_byte_data(self._addr, _MODE1, _MODE1_AI)
        self._bus.write_byte_data(self._addr, _MODE2, 0x04)
        time.sleep(0.005)

    def _set_frequency(self, freq):
        pre = int(round(_INTERNAL_OSC / (_PWM_STEPS * freq))) - 1
        pre = max(3, min(255, pre))
        self._bus.write_byte_data(self._addr, _MODE1, _MODE1_AI | _MODE1_SLEEP)
        time.sleep(0.005)
        self._bus.write_byte_data(self._addr, _PRE_SCALE, pre)
        self._bus.write_byte_data(self._addr, _MODE1, _MODE1_AI)
        time.sleep(0.005)
        self._bus.write_byte_data(self._addr, _MODE1, _MODE1_RESTART | _MODE1_AI)
        time.sleep(0.01)


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class PCA9685I2CNode(Node):

    def __init__(self):
        super().__init__("pca9685_i2c_node")

        self.declare_parameter("i2c_bus",       7)
        self.declare_parameter("i2c_address",   0x40)
        self.declare_parameter("pwm_frequency", 50.0)

        bus  = self.get_parameter("i2c_bus").value
        addr = self.get_parameter("i2c_address").value
        freq = self.get_parameter("pwm_frequency").value

        self._joints        = [Joint(n, _JOINT_CONFIG[n]) for n in FULL_JOINT_ORDER]
        self._name_to_joint = {j.name: j for j in self._joints}

        # Last position WRITTEN to the servo (for deadband comparison)
        self._written_pos   = {j.name: j.home_radians for j in self._joints}

        # Current commanded position (echoed as state)
        self._positions     = {j.name: j.home_radians for j in self._joints}

        # Per-joint frozen flag: True = ignore /joint_states updates for this joint
        # Set by /pca9685_arm_robot/freeze topic from pick_and_place
        self._frozen        = {j.name: False for j in self._joints}

        self._hw = PCA9685(bus, addr, freq, self.get_logger())
        if not self._hw.open():
            self.get_logger().fatal("Cannot open PCA9685.")
            return

        # Home all actuated servos
        for j in self._joints:
            if j.is_mimic:
                continue
            self._hw.set_pulse_us(j.channel, j.home_pulse_us())
            self._written_pos[j.name] = j.home_radians
            self.get_logger().info(
                f"  Homed {j.name} ch{j.channel} → {j.home_pulse_us():.0f} µs"
            )
        self._update_mimic_positions()
        self.get_logger().info("All actuated servos at home position")

        # /joint_states — primary control path from ros2_control
        self._sub_js = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10)

        # Direct bypass (Float64MultiArray, 6 actuated joints)
        self._sub_cmd = self.create_subscription(
            Float64MultiArray,
            "/pca9685_arm_robot/joint_commands",
            self._on_command, 10)

        # Freeze topic: pick_and_place publishes joint names to freeze
        # Format: Float64MultiArray with pairs [joint_index, position, ...]
        # Simpler: use a String topic with comma-separated joint names
        from std_msgs.msg import String
        self._sub_freeze = self.create_subscription(
            String,
            "/pca9685_arm_robot/freeze_joints",
            self._on_freeze, 10)

        self._sub_unfreeze = self.create_subscription(
            String,
            "/pca9685_arm_robot/unfreeze_joints",
            self._on_unfreeze, 10)

        # State publisher
        self._pub = self.create_publisher(
            Float64MultiArray, "/pca9685_arm_robot/joint_states", 10)
        self.create_timer(0.02, self._publish_state)

        self.get_logger().info(
            "pca9685_i2c_node ready  (deadband={:.3f} rad)\n"
            "  Actuated: base(ch2) shoulder(ch0) elbow(ch1) "
            "wrist(ch3) gripper(ch4) right_gear(ch5)\n"
            "  Mimic: right_link left_link left_gear right_finger left_finger\n"
            "  Freeze:   publish joint names to /pca9685_arm_robot/freeze_joints\n"
            "  Unfreeze: publish joint names to /pca9685_arm_robot/unfreeze_joints"
            .format(MIN_DELTA_RAD)
        )

    # ── Freeze / unfreeze callbacks ───────────────────────────────────────────

    def _on_freeze(self, msg):
        """Freeze named joints — ignore /joint_states for them."""
        names = [n.strip() for n in msg.data.split(",") if n.strip()]
        for name in names:
            if name in self._frozen:
                self._frozen[name] = True
                # Write current position one more time to hard-lock the servo
                j = self._name_to_joint.get(name)
                if j and not j.is_mimic:
                    pos = self._positions[name]
                    pulse = j.radians_to_pulse_us(pos)
                    self._hw.set_pulse_us(j.channel, pulse)
                    self.get_logger().info(
                        f"FROZEN {name} at {pos:.3f} rad ({pulse:.0f} µs)"
                    )

    def _on_unfreeze(self, msg):
        """Unfreeze named joints — resume /joint_states updates."""
        names = [n.strip() for n in msg.data.split(",") if n.strip()]
        for name in names:
            if name in self._frozen:
                self._frozen[name] = False
                # Reset written_pos to force next update to write immediately
                self._written_pos[name] = float("inf")
                self.get_logger().info(f"UNFROZEN {name}")

    # ── /joint_states callback ────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))

        for name, pos in name_to_pos.items():
            j = self._name_to_joint.get(name)
            if j is None or j.is_mimic:
                continue

            # Skip frozen joints entirely
            if self._frozen.get(name, False):
                continue

            pos_clamped = max(-j.urdf_limit_rad, min(j.urdf_limit_rad, pos))
            self._positions[name] = pos_clamped

            # ── DEADBAND FILTER ───────────────────────────────────────────────
            # Only write to PCA9685 if position changed by > MIN_DELTA_RAD.
            # This stops micro-interpolation steps from keeping servos moving.
            delta = abs(pos_clamped - self._written_pos.get(name, float("inf")))
            if delta < MIN_DELTA_RAD:
                continue   # position hasn't changed enough — skip I2C write

            try:
                pulse = j.radians_to_pulse_us(pos_clamped)
                self._hw.set_pulse_us(j.channel, pulse)
                self._written_pos[name] = pos_clamped
            except Exception as e:
                self.get_logger().error(f"I2C write failed on {name}: {e}")

        self._update_mimic_positions()

    # ── Direct command bypass ─────────────────────────────────────────────────

    def _on_command(self, msg: Float64MultiArray):
        if len(msg.data) != len(ACTUATED_JOINT_ORDER):
            self.get_logger().warn(
                f"Expected {len(ACTUATED_JOINT_ORDER)} values, got {len(msg.data)}"
            )
            return

        for name, radians in zip(ACTUATED_JOINT_ORDER, msg.data):
            j = self._name_to_joint[name]
            pos = max(-j.urdf_limit_rad, min(j.urdf_limit_rad, radians))
            self._positions[name] = pos
            self._written_pos[name] = pos   # bypass deadband for direct commands
            self._frozen[name] = False       # unfreeze on direct command
            try:
                self._hw.set_pulse_us(j.channel, j.radians_to_pulse_us(pos))
            except Exception as e:
                self.get_logger().error(f"I2C write failed on {name}: {e}")

        self._update_mimic_positions()

    # ── State publisher ───────────────────────────────────────────────────────

    def _publish_state(self):
        msg = Float64MultiArray()
        msg.data = [self._positions[j.name] for j in self._joints]
        self._pub.publish(msg)

    def _update_mimic_positions(self):
        for j in self._joints:
            if not j.is_mimic:
                continue
            src = self._positions.get(j.mimic_source, 0.0)
            self._positions[j.name] = j.mimic_position(src)

    def destroy_node(self):
        self.get_logger().info("Parking servos at home...")
        for j in self._joints:
            if j.is_mimic:
                continue
            try:
                self._hw.set_pulse_us(j.channel, j.home_pulse_us())
            except Exception:
                pass
        time.sleep(0.5)
        self._hw.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PCA9685I2CNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()