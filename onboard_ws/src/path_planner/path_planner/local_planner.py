from .vfh_submodules.vfh import *
from custom_msgs.srv import SendAction
from custom_msgs.msg import Action
from custom_msgs.srv import RequestAction
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


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

        self.goal_waypoint_subscriber = self.create_subscription(
            Waypoint, "/goal_waypoint", self.waypoint_callback, qos_profile)
        self.goal_waypoint: LocalCoordinates | None = None
        self.target_waypoint = LocalCoordinates()

        self.vehicle_info_subscriber = self.create_subscription(
            VehicleInfo, "/px4_monitoring/vehicle_info",
            self.vehicle_info_callback, qos_profile)
        self.vehicle_info = VehicleInfo()

        self.laser_scan_subscriber = self.create_subscription(
            LaserScan, "/laser_scan", self.laser_scan_callback, 10)
        self.laser_scan: LaserScan | None = None
        self.latest_laser_scan: LaserScan | None = None

        self.boundary_subscriber = self.create_subscription(
            Polygon, "/boundary_setter", self.boundary_setter_callback, 10)
        self.obstacles: list[Obstacle] = []

        ## ADD MONOLITHS (for testing)
        # for i in range(3):
        #     for j in range(3):
        #         if (i == 1 and j == 1):
        #             continue
        #         monolith = Monolithe(i * 10 - 10, j * 10 - 10, 1, 1)
        #         self.obstacles.append(monolith)

        self.enqueue_action_client = self.create_client(
            SendAction, "/px4_action_queue/append_action")
        self.action_req: SendAction.Request = SendAction.Request()

        while not self.enqueue_action_client.wait_for_service(1.0):
            self.get_logger().info('service not available, waiting again...')

        self.popleft_action_client = self.create_client(
            RequestAction, "/px4_action_queue/popleft_action")
        self.action_req_req: RequestAction.Request = RequestAction.Request()

        while not self.popleft_action_client.wait_for_service(1.0):
            self.get_logger().info('service not available, waiting again...')

        timer_period = 0.5
        self.timer_1 = self.create_timer(timer_period, self.controller_tick)
        # self.goal_waypoint = LocalCoordinates(0.0, 10.0, -10.0)
        self.timer_2 = self.create_timer(timer_period / 2,
                                         self.planner_tick_v3)

        self.planner = VfhAlgorithm()

    def send_action(self, action: Action):
        self.action_req.action = action
        return self.enqueue_action_client.call_async(self.action_req)

    def popleft_action(self):
        return self.popleft_action_client.call_async(self.action_req_req)

    @staticmethod
    def angle_dist(ang1, ang2):
        a = ang2 - ang1
        return abs(((a + np.pi) % (2 * np.pi)) - np.pi)

    def planner_tick_v3(self):
        # if (self.vehicle_info is None or self.laser_scan is None
        #         or self.goal_waypoint is None):
        #     return
        if (self.vehicle_info is None or self.goal_waypoint is None
                or self.goal_waypoint == LocalCoordinates()):
            return
        self.target_waypoint = self.planner.generate_target(
            self.goal_waypoint, self.vehicle_info, self.laser_scan,
            self.obstacles, self.timer_2.timer_period_ns * 1e-9)

    @staticmethod
    def target_match_action(target: LocalCoordinates, msg: Action):
        command_bool = msg.action == Action.ACTION_WAYPOINT
        x_bool = abs(target.x - msg.x) < 1e-10
        y_bool = abs(target.y - msg.y) < 1e-10
        z_bool = abs(target.z - msg.z) < 1e-10
        yaw_bool = VfhPlanner.angle_dist(target.yaw, msg.yaw) < 1e-10
        speed_bool = abs(target.max_speed_h - msg.max_speed_h) < 1e-10
        return command_bool and x_bool and y_bool and z_bool and yaw_bool and speed_bool

    def controller_tick(self):
        # self.get_logger().info(
        #     f"Current goal : ({self.goal_waypoint.x}, {self.goal_waypoint.y}, {self.goal_waypoint.z})\n"
        # )
        if (self.goal_waypoint is None):
            return
        if (self.goal_waypoint == LocalCoordinates()
                and self.vehicle_info.arming_state
                == VehicleInfo.ARMING_STATE_ARMED
                and self.vehicle_info.curr_action_obj.action
                != Action.ACTION_LAND):
            self.popleft_action()
            action = Action()
            action.action = Action.ACTION_LAND
            self.send_action(action)
            return
        elif (self.goal_waypoint != LocalCoordinates()
              and self.vehicle_info.arming_state
              == VehicleInfo.ARMING_STATE_DISARMED
              and self.vehicle_info.curr_action_obj.action
              != Action.ACTION_TAKE_OFF):
            action = Action()
            action.action = Action.ACTION_TAKE_OFF
            self.send_action(action)
        elif (self.target_waypoint != LocalCoordinates() and
              self.vehicle_info.arming_state == VehicleInfo.ARMING_STATE_ARMED
              and self.vehicle_info.curr_action_obj.action
              != Action.ACTION_TAKE_OFF and
              self.vehicle_info.curr_action_obj.action != Action.ACTION_LAND):
            # This should be the part of the code that does obstacle avoidance and publishes action
            if (self.goal_waypoint.distance_to_xyz(self.target_waypoint.x,
                                                   self.target_waypoint.y,
                                                   self.target_waypoint.z)
                    <= 0.2):
                return

            if (not VfhPlanner.target_match_action(
                    self.target_waypoint, self.vehicle_info.curr_action_obj)):
                action = Action()
                action.action = Action.ACTION_WAYPOINT
                action.x = self.target_waypoint.x
                action.y = self.target_waypoint.y
                action.z = self.target_waypoint.z
                # action.yaw = self.target_waypoint.yaw

                action.yaw = self.target_waypoint.yaw
                action.max_speed_h = self.target_waypoint.max_speed_h
                print(f"sending action {action}")
                self.send_action(action)
            if (self.vehicle_info.curr_action_obj.action != Action.ACTION_NONE
                    and not VfhPlanner.target_match_action(
                        self.target_waypoint,
                        self.vehicle_info.curr_action_obj)):
                self.popleft_action()

    def waypoint_callback(self, msg: Waypoint):
        self.planner.change_max_speed(msg.max_speed_h)
        self.goal_waypoint = LocalCoordinates(msg.x, msg.y, msg.z,
                                              msg.max_speed_h)

    def vehicle_info_callback(self, msg: VehicleInfo):
        self.vehicle_info = msg
        self.laser_scan = self.latest_laser_scan

    def laser_scan_callback(self, msg: LaserScan):
        self.latest_laser_scan = msg

    def boundary_setter_callback(self, msg: Polygon):
        self.obstacles.append(Boundary(msg.points))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = VfhPlanner()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
