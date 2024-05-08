import rclpy
from rclpy.node import Node

from custom_msgs.msg import VehicleInfo
from custom_msgs.msg import Action
from custom_msgs.msg import Waypoint

from custom_msgs.srv import SendAction
from custom_msgs.srv import RequestAction

from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class LocalCoordinates:

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def set_xyz(self, msg: Waypoint):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def __eq__(self, o):
        if (o is None):
            return False
        return self.x == o.x and self.y == o.y and self.z == o.z


class VfhPlanner(Node):

    def __init__(self):
        super().__init__('local_planner')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)

        self.targe_subscriber = self.create_subscription(
            Waypoint, "/target_position", self.waypoint_callback, qos_profile)
        self.goal_waypoint = LocalCoordinates()
        self.target_waypoint = LocalCoordinates()

        self.vehicle_info_subscriber = self.create_subscription(
            VehicleInfo, "/px4_monitoring/vehicle_info",
            self.vehicle_info_callback, qos_profile)
        self.vehicle_info = VehicleInfo()

        self.laser_scan_subscriber = self.create_subscription(
            LaserScan, "/laser_scan", self.laser_scan_callback, qos_profile)
        self.laser_scan: LaserScan | None = None

        self.enqueue_action_client = self.create_client(
            SendAction, "/px4_action_queue/append_action")
        self.action_req: SendAction.Request = SendAction.Request()

        while not self.enqueue_action_client.wait_for_service(1.0):
            self.get_logger().info('service not available, waiting again...')

        timer_period = 0.5
        self.timer_1 = self.create_timer(timer_period, self.controller_tick)
        self.timer_2 = self.create_timer(timer_period / 2, self.planner_tick)

    def send_action(self, action: Action):
        self.action_req.action = action
        return self.enqueue_action_client.call_async(self.action_req)

    def planner_tick(self):
        self.target_waypoint = self.goal_waypoint

    def controller_tick(self):
        self.get_logger().info(
            f"Current Target: ({self.goal_waypoint.x}, {self.goal_waypoint.y}, {self.goal_waypoint.z})\n"
        )
        if (self.goal_waypoint == LocalCoordinates(0, 0, 0)
                and self.vehicle_info.arming_state
                == VehicleInfo.ARMING_STATE_ARMED
                and self.vehicle_info.curr_action_obj.action
                != Action.ACTION_LAND):
            action = Action()
            action.action = Action.ACTION_LAND
            self.send_action(action)
            return
        elif (self.goal_waypoint != LocalCoordinates(0, 0, 0)
              and self.vehicle_info.arming_state
              == VehicleInfo.ARMING_STATE_DISARMED
              and self.vehicle_info.curr_action_obj.action
              != Action.ACTION_TAKE_OFF):
            action = Action()
            action.action = Action.ACTION_TAKE_OFF
            self.send_action(action)
        elif (self.target_waypoint != LocalCoordinates(0, 0, 0) and
              self.vehicle_info.arming_state == VehicleInfo.ARMING_STATE_ARMED
              and self.vehicle_info.curr_action_obj.action
              == Action.ACTION_NONE):
            ## This should be the part of the code that does obstacle avoidance and publishes action
            action = Action()
            action.action = Action.ACTION_WAYPOINT
            action.x = self.target_waypoint.x
            action.y = self.target_waypoint.y
            action.z = self.target_waypoint.z
            self.send_action(action)

    def waypoint_callback(self, msg: Waypoint):
        self.goal_waypoint.set_xyz(msg)

    def vehicle_info_callback(self, msg: VehicleInfo):
        self.vehicle_info = msg

    def laser_scan_callback(self, msg: LaserScan):
        self.laser_scan = msg


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = VfhPlanner()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
