# scripts/mimic_relay.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MimicRelay(Node):
    def __init__(self):
        super().__init__('mimic_relay')

        # Master -> [mimic joints] with their multiplier/offset from URDF
        self.mimic_map = {
            'left_gear_joint': {
                'right_link_joint':   {'multiplier':  1.0, 'offset': 0.0},
                'left_link_joint':    {'multiplier':  1.0, 'offset': 0.0},
                'right_gear_joint':   {'multiplier':  1.0, 'offset': 0.0},
                'right_finger_joint': {'multiplier':  1.0, 'offset': 0.0},
                'left_finger_joint':  {'multiplier':  1.0, 'offset': 0.0},
            }
        }

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.cb, 10)
        self.pub = self.create_publisher(
            JointState, '/joint_states', 10)

    def cb(self, msg):
        mimic_msg = JointState()
        mimic_msg.header.stamp = self.get_clock().now().to_msg()

        for master, mimics in self.mimic_map.items():
            if master in msg.name:
                idx = msg.name.index(master)
                master_pos = msg.position[idx]

                for mimic_joint, params in mimics.items():
                    mimic_pos = master_pos * params['multiplier'] + params['offset']
                    mimic_msg.name.append(mimic_joint)
                    mimic_msg.position.append(mimic_pos)

        if mimic_msg.name:
            self.pub.publish(mimic_msg)

def main():
    rclpy.init()
    node = MimicRelay()
    rclpy.spin(node)

rclpy.shutdown()