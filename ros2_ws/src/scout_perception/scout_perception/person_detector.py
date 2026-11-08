"""Person Detector Node."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PersonDetector(Node):
    """Person Detector Node."""

    def __init__(self):
        """Start the Person Detector Node."""
        super().__init__('person_detector')
        self.get_logger().info('PersonDetector Node started...')


def main(args=None):
    """Create the PersonDetector Node."""
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
