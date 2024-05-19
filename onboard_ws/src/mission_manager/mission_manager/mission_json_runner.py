import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from custom_msgs.msg import GlobalCoordinates
from custom_msgs.msg import ListGlobalCoordinates
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import json
import os


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)
        path = os.path.realpath(__file__)
        path = os.path.dirname(path)
        path = os.path.join(path, "../resource/sim_boundary.json")
        with open(path, "r") as fp:
            self.json = json.load(fp)
        self.boundary_publisher = self.create_publisher(
            ListGlobalCoordinates, '/mission_boundary', self.qos_profile)
        self.lap_publisher = self.create_publisher(ListGlobalCoordinates,
                                                   '/mission_lap',
                                                   self.qos_profile)

        # Boundary
        msg_list = ListGlobalCoordinates()
        msg_list.coords = []
        for entry in self.json["boundary"]:
            msg = GlobalCoordinates()
            msg.latitude = entry["latitude"]
            msg.longitude = entry["longtitude"]
            msg_list.coords.append(msg)
        self.boundary_publisher.publish(msg_list)

        # lap
        msg_list = ListGlobalCoordinates()
        msg_list.coords = []
        for entry in self.json["lap"]:
            msg = GlobalCoordinates()
            msg.latitude = entry["latitude"]
            msg.longitude = entry["longtitude"]
            msg.altitude = entry["altitude"]
            msg_list.coords.append(msg)
        self.lap_publisher.publish(msg_list)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically files
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
