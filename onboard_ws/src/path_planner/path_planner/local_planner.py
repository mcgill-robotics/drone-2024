import rclpy
from rclpy.node import Node

from custom_msgs.msg import VehicleInfo
from custom_msgs.msg import Action
from custom_msgs.msg import Waypoint

from custom_msgs.srv import SendAction
from custom_msgs.srv import RequestAction


class LocalCoordinates:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

    def set_xyz(self, msg: Waypoint):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z


class VfhPlanner(Node):

    def __init__(self):
        super().__init__('local_planner')
        self.targe_subscriber = self.create_subscription(
            Waypoint, "/target_position", self.waypoint_callback, 10)
        self.target_waypoint = LocalCoordinates()

        self.vehicle_info_subscriber = self.create_subscription(
            VehicleInfo, "/px4_monitoring/vehicle_info",
            self.vehicle_info_callback, 10)
        self.vehicle_info = VehicleInfo()

    def waypoint_callback(self, msg: Waypoint):
        self.target_waypoint.set_xyz(msg)

    def vehicle_info_callback(self, msg: VehicleInfo):
        self.vehicle_info = msg


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = VfhPlanner()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
