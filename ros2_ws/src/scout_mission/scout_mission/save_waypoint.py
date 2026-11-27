"""Save the robot's current AMCL pose as a named waypoint."""

import sys

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scout_mission.waypoints import quaternion_to_yaw, save_waypoint


def main(args=None):
    """Usage: ros2 run scout_mission save_waypoint."""
    argv = rclpy.utilities.remove_ros_args(sys.argv if args is None else args)
    if len(argv) != 3:
        print('usage: save_waypoint NAME /abs/path/waypoints.yaml')
        return 2
    name, path = argv[1], argv[2]
    rclpy.init(args=args)
    node = Node('save_waypoint')
    got = []
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PoseWithCovarianceStamped,
                             'amcl_pose', got.append, qos)
    end = node.get_clock().now().nanoseconds + 5_000_000_000
    while not got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    code = 1
    if got:
        p = got[-1].pose.pose
        yaw = quaternion_to_yaw(p.orientation.x, p.orientation.y,
                                p.orientation.z, p.orientation.w)
        save_waypoint(path, name, p.position.x, p.position.y, yaw)
        print(f'saved {name}: x={p.position.x:.3f} y={
            p.position.y:.3f} yaw={yaw:.3f}')
        code = 0
    else:
        print('no /amcl_pose within 5 s: is navigation.launch.py running?')
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
