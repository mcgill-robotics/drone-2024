import rclpy
from rclpy.node import Node
from custom_msgs.msg import Target
from custom_msgs.msg import VehicleInfo
from geometry_msgs.msg import Point32

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('dummy_computer_vision')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)

        self.curr_target_sub = self.create_subscription(
            Target, "/current_target", self.current_target_callback,
            self.qos_profile)
        self.curr_target: Target | None = None

        self.vehicle_info_sub = self.create_subscription(
            VehicleInfo, "/px4_monitoring/vehicle_info",
            self.vehicle_info_callback, self.qos_profile)
        self.vehicle_info: VehicleInfo | None = None

        self.target_position_publisher = self.create_publisher(
            Point32, "/target_location", self.qos_profile)

        timer_period = 0.5
        self.timer_1 = self.create_timer(timer_period, self.cv_tick)

    def find_target_position_in_curr_frame(self):
        # TODO: CV CODE, LOOK FOR self.curr_target in the next few frames (or just next frame)
        # If not in the frame, return None, if is in the frame(s), return its position

        # since this is a dummy node made for testing the mission node, i will just return current position + 5
        msg = Point32()
        msg.x = 94.0
        msg.y = 425.0
        msg.z = 0.0
        return msg

    def cv_tick(self):
        # Only publish a given position once, hence the 2 if statements and setting curr_target back to None
        if (self.curr_target is None):
            ## no current target, mission is doing something else (lap or something)
            return

        ## So here target is not None, aka a valid target
        target_position: Point32 | None = self.find_target_position_in_curr_frame(
        )
        if (target_position is not None):
            self.curr_target = None
            self.target_position_publisher.publish(target_position)

    def current_target_callback(self, msg: Target):
        self.curr_target = msg

    def vehicle_info_callback(self, msg: VehicleInfo):
        self.vehicle_info = msg


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
