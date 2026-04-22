#!/usr/bin/env python3
"""
pca9685_i2c_node.py
────────────────────────────────────────────────────────────────────────────
Standalone ROS 2 node that bridges ros2_control JointTrajectory commands
to the PCA9685 servo controller over I2C (bus 7, addr 0x40).

This node is self-contained — it does NOT import from pca9685_arm_robot.py
to avoid the hardware_interface dependency.

Subscriptions:
  /pca9685_arm_robot/joint_commands  (std_msgs/Float64MultiArray)
    data[0] = base_joint     (radians)
    data[1] = shoulder_joint (radians)
    data[2] = elbow_joint    (radians)
    data[3] = wrist_joint    (radians)
    data[4] = gripper_joint  (radians)

Publications:
  /pca9685_arm_robot/joint_states  (std_msgs/Float64MultiArray)
    Same order as above — echoes last commanded position at 50 Hz.

The node also bridges ros2_control by forwarding /joint_states positions
to the PCA9685 whenever arm_controller publishes a new trajectory point.

Parameters:
  i2c_bus       (int,   default 7)     — I2C bus number on Jetson AGX Orin
  i2c_address   (int,   default 0x40)  — PCA9685 I2C address
  pwm_frequency (float, default 50.0)  — PWM frequency in Hz
"""

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

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
_INTERNAL_OSC  = 25_000_000
_PWM_STEPS     = 4096


# ── Calibrated joint configuration ───────────────────────────────────────────
# Values measured on 2026-04-20 using calibrate_servos.py
# Format: channel, min_pulse_us, max_pulse_us, urdf_limit_rad, home_deg
_JOINT_CONFIG = {
    "rotating_base_joint":     {"channel": 2, "min_pulse_us": 280, "max_pulse_us": 2540,
                       "urdf_limit_rad": 1.046, "home_deg": 0.0},
    "shoulder_link_joint": {"channel": 0, "min_pulse_us":  1358, "max_pulse_us": 2058,
                       "urdf_limit_rad": 1.046, "home_deg": 0.0},
    "Elbow_link_joint":    {"channel": 1, "min_pulse_us":  370, "max_pulse_us": 2630,
                       "urdf_limit_rad": 1.046, "home_deg": 0.0},
    "wrist_link_joint":    {"channel": 3, "min_pulse_us":  530, "max_pulse_us": 2310,
                       "urdf_limit_rad": 1.570, "home_deg": 0.0},
    "gripper_joint":  {"channel": 4, "min_pulse_us":  620, "max_pulse_us": 2760,
                       "urdf_limit_rad": 1.570, "home_deg": 0.0},
}

JOINT_ORDER = [
    "rotating_base_joint",
    "shoulder_link_joint",
    "Elbow_link_joint",
    "wrist_link_joint",
    "gripper_joint",
]


# ── Joint helper ──────────────────────────────────────────────────────────────

class Joint:
    """Converts URDF radians to PCA9685 pulse width."""

    def __init__(self, name: str, cfg: dict):
        self.name           = name
        self.channel        = cfg["channel"]
        self.min_pulse_us   = float(cfg["min_pulse_us"])
        self.max_pulse_us   = float(cfg["max_pulse_us"])
        self.urdf_limit_rad = float(cfg["urdf_limit_rad"])
        self._mid   = (self.min_pulse_us + self.max_pulse_us) / 2.0
        self._half  = (self.max_pulse_us - self.min_pulse_us) / 2.0
        self.home_radians = math.radians(cfg["home_deg"])

    def radians_to_pulse_us(self, radians: float) -> float:
        r = max(-self.urdf_limit_rad, min(self.urdf_limit_rad, radians))
        return self._mid + (r / self.urdf_limit_rad) * self._half

    def home_pulse_us(self) -> float:
        return self.radians_to_pulse_us(self.home_radians)


# ── PCA9685 driver ────────────────────────────────────────────────────────────

class PCA9685:
    def __init__(self, bus_num: int, addr: int, freq: float, logger):
        self._bus_num = bus_num
        self._addr    = addr
        self._freq    = freq
        self._log     = logger
        self._bus     = None

    def open(self) -> bool:
        if not _SMBUS_OK:
            self._log.error("smbus2 not installed. Run: pip3 install smbus2 --break-system-packages")
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

    def set_pulse_us(self, channel: int, pulse_us: float):
        period_us = 1_000_000.0 / self._freq
        off = int(round(pulse_us / period_us * _PWM_STEPS))
        off = max(0, min(_PWM_STEPS - 1, off))
        base = _LED0_ON_L + 4 * channel
        self._bus.write_i2c_block_data(self._addr, base, [
            0x00, 0x00,
            off & 0xFF, (off >> 8) & 0x0F,
        ])

    def close(self):
        if self._bus:
            try:
                self._bus.write_byte_data(self._addr, _ALL_LED_OFF_H, 0x10)
                self._bus.close()
            except Exception:
                pass
            self._bus = None

    def _swreset(self):
        """General Call software reset — clears stuck EXTCLK/SLEEP state."""
        try:
            self._bus.write_byte_data(0x00, 0x06, 0x00)
            time.sleep(0.01)
        except Exception:
            pass  # NACK on general call is normal on some hosts

    def _reset(self):
        self._bus.write_byte_data(self._addr, _MODE1, _MODE1_AI)
        self._bus.write_byte_data(self._addr, _MODE2, 0x04)
        time.sleep(0.005)

    def _set_frequency(self, freq: float):
        prescale = int(round(_INTERNAL_OSC / (_PWM_STEPS * freq))) - 1
        prescale = max(3, min(255, prescale))
        # Enter sleep (no EXTCLK bit) to change prescaler
        self._bus.write_byte_data(self._addr, _MODE1,
                                  _MODE1_AI | _MODE1_SLEEP)
        time.sleep(0.005)
        self._bus.write_byte_data(self._addr, _PRE_SCALE, prescale)
        self._bus.write_byte_data(self._addr, _MODE1, _MODE1_AI)
        time.sleep(0.005)
        self._bus.write_byte_data(self._addr, _MODE1,
                                  _MODE1_RESTART | _MODE1_AI)
        time.sleep(0.01)


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class PCA9685I2CNode(Node):

    def __init__(self):
        super().__init__("pca9685_i2c_node")

        # ROS parameters
        self.declare_parameter("i2c_bus",       7)
        self.declare_parameter("i2c_address",   0x40)
        self.declare_parameter("pwm_frequency", 50.0)

        bus  = self.get_parameter("i2c_bus").value
        addr = self.get_parameter("i2c_address").value
        freq = self.get_parameter("pwm_frequency").value

        # Build joint list
        self._joints = [Joint(name, _JOINT_CONFIG[name]) for name in JOINT_ORDER]
        self._positions = [j.home_radians for j in self._joints]

        # Init PCA9685
        self._hw = PCA9685(bus, addr, freq, self.get_logger())
        if not self._hw.open():
            self.get_logger().fatal(
                "Cannot open PCA9685. Check I2C wiring and run: "
                "echo '7-0040' | sudo tee /sys/bus/i2c/devices/7-0040/driver/unbind"
            )
            return

        # Home all servos
        for j in self._joints:
            self._hw.set_pulse_us(j.channel, j.home_pulse_us())
            self.get_logger().info(
                f"  Homed {j.name} ch{j.channel} → {j.home_pulse_us():.0f} µs"
            )
        self.get_logger().info("All servos at home position")

        # Subscribe to simple Float64MultiArray commands
        self._sub_cmd = self.create_subscription(
            Float64MultiArray,
            "/pca9685_arm_robot/joint_commands",
            self._on_command,
            10,
        )

        # Subscribe to /joint_states published by joint_state_broadcaster
        # so ros2_control trajectory execution drives the real servos
        self._sub_js = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,
        )

        # Publish state at 50 Hz
        self._pub = self.create_publisher(
            Float64MultiArray,
            "/pca9685_arm_robot/joint_states",
            10,
        )
        self.create_timer(0.02, self._publish_state)

        self.get_logger().info(
            "pca9685_i2c_node ready — listening on /joint_states and "
            "/pca9685_arm_robot/joint_commands"
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState):
        """
        Receives joint positions from joint_state_broadcaster and writes
        them directly to the PCA9685.  This is the main control path when
        ros2_control / MoveIt 2 executes a trajectory.
        """
        name_to_pos = dict(zip(msg.name, msg.position))
        for i, j in enumerate(self._joints):
            if j.name in name_to_pos:
                pos = name_to_pos[j.name]
                try:
                    self._hw.set_pulse_us(j.channel,
                                          j.radians_to_pulse_us(pos))
                    self._positions[i] = pos
                except Exception as e:
                    self.get_logger().error(
                        f"I2C write failed on {j.name}: {e}"
                    )

    def _on_command(self, msg: Float64MultiArray):
        """Direct Float64MultiArray command (bypass ros2_control)."""
        if len(msg.data) != len(self._joints):
            self.get_logger().warn(
                f"Expected {len(self._joints)} values, got {len(msg.data)}"
            )
            return
        for i, (j, radians) in enumerate(zip(self._joints, msg.data)):
            try:
                self._hw.set_pulse_us(j.channel,
                                      j.radians_to_pulse_us(radians))
                self._positions[i] = radians
            except Exception as e:
                self.get_logger().error(f"I2C write failed on {j.name}: {e}")

    def _publish_state(self):
        msg = Float64MultiArray()
        msg.data = list(self._positions)
        self._pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Parking servos at home...")
        for j in self._joints:
            try:
                self._hw.set_pulse_us(j.channel, j.home_pulse_us())
            except Exception:
                pass
        time.sleep(0.5)
        self._hw.close()
        super().destroy_node()


# ── entry point ───────────────────────────────────────────────────────────────

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