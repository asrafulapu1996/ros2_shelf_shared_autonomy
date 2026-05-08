"""
Keyboard trigger — replaces the gamepad for the picker node.

Publishes sensor_msgs/Joy to /joy so picker_node works without a physical gamepad.

  Press ENTER  →  simulates Start button press  (triggers pick or confirms release)
  Press q      →  quit

Run instead of: ros2 run joy joy_node
"""

import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


BUTTON_INDEX = 7   # must match grasp_button_index in picker_node (default 7)


class KeyboardTrigger(Node):

    def __init__(self):
        super().__init__('keyboard_trigger')
        self.declare_parameter('grasp_button_index', BUTTON_INDEX)
        self._btn = self.get_parameter('grasp_button_index').value

        self._pub = self.create_publisher(Joy, '/joy', 10)
        self.get_logger().info(
            f'Keyboard trigger ready  (button index {self._btn}).\n'
            '  Press ENTER to trigger pick / confirm release.\n'
            '  Press q + ENTER to quit.')

    def _publish(self, pressed: bool):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = []
        msg.buttons = [0] * (self._btn + 1)
        msg.buttons[self._btn] = 1 if pressed else 0
        self._pub.publish(msg)

    def run(self):
        try:
            while rclpy.ok():
                key = input('').strip().lower()
                if key == 'q':
                    break
                # Simulate rising edge: press then release
                self._publish(True)
                self._publish(False)
                self.get_logger().info('Button press sent → /joy')
        except (EOFError, KeyboardInterrupt):
            pass


def main():
    rclpy.init()
    node = KeyboardTrigger()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
