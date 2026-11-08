"""Pan Tracking Node."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PanTracker(Node):
    """Pan Tracking Class."""

    def __init__(self):
        """Create PanTracker Constructor."""
        super().__init__('pan_tracker')
        self.get_logger().info('PanTracker Node started...')


def main(args=None):
    """Create the PanTracker Node."""
    rclpy.init(args=args)
    node = PanTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
