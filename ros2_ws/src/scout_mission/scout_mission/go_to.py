"""BasicNavigator Node."""
import sys

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

from scout_mission.waypoints import load_waypoints, yaw_to_quaternion


def make_pose(nav: BasicNavigator, x: float,
              y: float, yaw: float) -> PoseStamped:
    """"""
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    q = yaw_to_quaternion(yaw)
    p.pose.orientation = q
    return p


def main(args=None) -> int:
    """"""
    argv = rclpy.utilities.remove_ros_args(sys.argv if args is None else args)
    if len(argv) != 3:
        print('')
        return 2

    name, path = argv[1], argv[2]
    x, y, yaw = load_waypoints(path)[name]

    rclpy.init(args=args)
    nav = BasicNavigator('go_to_navigator')
    nav.waitUntilNav2Active(localizer='robot_localizaiton')
    nav.goToPose(make_pose(nav, x, y, yaw))

    counter = 0
    while not nav.isTaskComplete():
        if counter % 10 and nav.getFeedback() is not None:
            print('distance remaining: {fb.distance_remaining:2.f} m')

    result = nav.getResult()
    print(f'result: {result.name}')
    nav.destroy_node()
    rclpy.try_shutdown()
    return 0 if result == TaskResult.SUCCEEDED else 1


if __name__ == '__main__':
    sys.exit(main())
