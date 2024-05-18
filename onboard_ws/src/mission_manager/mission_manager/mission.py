import rclpy
from rclpy.node import Node

from custom_msgs.msg import GlobalCoordinates
from custom_msgs.msg import ListGlobalCoordinates
from custom_msgs.msg import VehicleInfo
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

import pymap3d as pm


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)

        self.vehicle_info_sub = self.create_subscription(
            VehicleInfo, "/px4_monitoring/vehicle_info",
            self.vehicle_info_callback, self.qos_profile)
        self.curr_n, self.curr_e, self.curr_d = None, None, None
        self.lat0, self.lon0, self.alt0 = None, None, None

        self.mission_boundary_sub = self.create_subscription(
            ListGlobalCoordinates, "/mission_boundary",
            self.mission_boundary_callback, self.qos_profile)
        self.boundary_setter_pub = self.create_publisher(
            Polygon, "/boundary_setter", 10)

        self.lap_points_sub = self.create_subscription(
            ListGlobalCoordinates, "/mission_lap", self.mission_lap_callback,
            self.qos_profile)
        self.lap = None
        self.curr_lap_point_index = 0

        timer_period = 0.1
        self.create_timer(timer_period, self.perform_lap)

    def vehicle_info_callback(self, msg: VehicleInfo):
        self.curr_n, self.curr_e, self.curr_d = msg.x, msg.y, msg.z
        self.lat0, self.lon0, self.alt0 = msg.ref_la, msg.ref_lon, msg.ref_alt

    def get_ned(self, lat, lon, alt):
        if self.lat0 is None:
            self.get_logger().warn(
                "Not quite ready to take in coordinates for boundary, waiting on vehicle info..."
            )
            return None
        n, e, d = pm.geodetic2ned(lat, lon, alt, self.lat0, self.lon0,
                                  self.alt0)
        return n, e, d

    def mission_boundary_callback(self, msg: ListGlobalCoordinates):
        if self.lat0 is None:
            self.get_logger().warn(
                "Not quite ready to take in coordinates for boundary, waiting on vehicle info..."
            )
            return
        polygon = Polygon()
        polygon.points = []
        for glob in msg.coords:
            glob: GlobalCoordinates = glob
            n, e, d = self.get_ned(glob.latitude, glob.longitude,
                                   glob.altitude)
            point = Point32()
            point.x, point.y, point.z = n, e, d
            polygon.points.append(point)

        self.boundary_setter_pub.publish(polygon)

    def mission_lap_callback(self, msg: ListGlobalCoordinates):
        if self.lat0 is None:
            self.get_logger().warn(
                "Not quite ready to take in coordinates for boundary, waiting on vehicle info..."
            )
            return
        self.lap = []
        for glob in msg.coords:
            glob: GlobalCoordinates = glob
            n, e, d = self.get_ned(glob.latitude, glob.longitude,
                                   glob.altitude)
            self.lap.append((n, e, d))

    def perform_lap(self):
        if (self.curr_n is None or self.lap is None):
            self.get_logger().warn(
                "Not quite ready to perform lap yet, waiting on lap to be set and vehicle info..."
            )
            return

        pass


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MissionNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
