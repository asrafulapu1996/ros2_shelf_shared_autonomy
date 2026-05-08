import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class KeyboardSelector(Node):
    def __init__(self):
        super().__init__('keyboard_selector')
        self.pub = self.create_publisher(Int32, 'med_selection', 10)
        self.get_logger().info('Keyboard selector started. Enter 1-6 then Enter.')

    def run(self):
        try:
            while rclpy.ok():
                s = input('Select medicine (1-6): ').strip()
                if not s:
                    continue
                if s.lower() in ['q', 'quit', 'exit']:
                    break
                try:
                    v = int(s)
                except ValueError:
                    self.get_logger().warn('Invalid input, enter a number 1-6')
                    continue
                if 1 <= v <= 6:
                    msg = Int32()
                    msg.data = v
                    self.pub.publish(msg)
                    self.get_logger().info(f'Published selection {v}')
                else:
                    self.get_logger().warn('Number out of range (1-6)')
        except (EOFError, KeyboardInterrupt):
            pass


def main():
    rclpy.init()
    node = KeyboardSelector()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
