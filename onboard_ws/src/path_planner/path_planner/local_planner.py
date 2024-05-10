import rclpy
from rclpy.node import Node

from custom_msgs.msg import VehicleInfo
from custom_msgs.msg import Action
from custom_msgs.msg import Waypoint

from custom_msgs.srv import SendAction
from custom_msgs.srv import RequestAction

from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np


class LocalCoordinates:

    def __init__(self, x=0.0, y=0.0, z=0.0):
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

    def distance_to_xyz(self, x, y, z):
        return ((self.x - x)**2 + (self.y - y)**2 + (self.z - z)**2)**(1 / 2)


class VfhParam:

    def __init__(self,
                 a=0,
                 b=1,
                 certainty=1,
                 smoothing_window_size=5,
                 threshold=10,
                 s_max=25,
                 step_size=1,
                 v_max=10,
                 h_m=10):
        # Set a and b such that a - b * (range_max) =  0, with a, b > 0
        self.a = a
        self.b = b
        self.cert = certainty
        self.smoothing_win_size = smoothing_window_size
        self.num_slots = 360
        self.threshold = threshold
        self.s_max = s_max
        self.v_max = v_max
        self.h_m = h_m

    def adapt(self, msg: LaserScan):
        self.a = msg.range_max
        self.b = 1
        self.num_slots = int(2 * np.pi / msg.angle_increment)
        self.threshold = msg.range_max / 20
        self.s_max = self.num_slots // 4
        self.h_m = msg.range_max


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
        self.goal_waypoint = LocalCoordinates()
        self.target_waypoint = LocalCoordinates()

        self.vehicle_info_subscriber = self.create_subscription(
            VehicleInfo, "/px4_monitoring/vehicle_info",
            self.vehicle_info_callback, qos_profile)
        self.vehicle_info = VehicleInfo()

        self.laser_scan_subscriber = self.create_subscription(
            LaserScan, "/laser_scan", self.laser_scan_callback, 10)
        self.laser_scan: LaserScan | None = None

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
        self.timer_2 = self.create_timer(timer_period * 2,
                                         self.planner_tick_v2)

        self.vfh_params = VfhParam()

    def send_action(self, action: Action):
        self.action_req.action = action
        return self.enqueue_action_client.call_async(self.action_req)

    def popleft_action(self):
        return self.popleft_action_client.call_async(self.action_req_req)

    @staticmethod
    def map(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def planner_tick_v2(self):

        if (self.vehicle_info is None or self.laser_scan is None):
            return

        if (self.goal_waypoint.distance_to_xyz(self.vehicle_info.x,
                                               self.vehicle_info.y,
                                               self.vehicle_info.z) < 0.5):
            return

        # populate histogram
        histogram = np.full(self.vfh_params.num_slots,
                            self.laser_scan.range_min)
        yaw = self.vehicle_info.heading
        for i, reading in enumerate(self.laser_scan.ranges):
            # [0, 2*pi] angle for this reading
            angle = ((self.laser_scan.angle_min +
                      i * self.laser_scan.angle_increment) + yaw) % (2 * np.pi)
            in_histogram_index = int(
                VfhPlanner.map(angle, 0, 2 * np.pi, 0, len(histogram)))
            reading = min(reading, self.laser_scan.range_max)
            reading = max(reading, self.laser_scan.range_min)
            histogram[in_histogram_index] = reading
        histogram = self.vfh_params.cert**2 * (self.vfh_params.a -
                                               self.vfh_params.b * histogram)
        # smoothing
        smooth_histogram = np.copy(histogram)
        for i in range(len(histogram)):
            reachable_range = self.vfh_params.smoothing_win_size
            reachable_range = min(reachable_range, i + 1)
            reachable_range = min(reachable_range, len(histogram) - i)
            sum = histogram[i] * reachable_range
            denom = 2 * reachable_range + 1
            reachable_range -= 1
            for f in range(1, reachable_range):
                sum += (reachable_range - f) * (histogram[i + f] +
                                                histogram[i - f])
            smooth_histogram[i] = sum / denom

        # masking False if valley, True if obstacle
        obstacle_histogram = np.full(self.vfh_params.num_slots, False)
        obstacle_histogram[smooth_histogram > self.vfh_params.threshold] = True

        # get k_n
        target_angle = np.arctan2(
            self.goal_waypoint.y - self.vehicle_info.y,
            self.goal_waypoint.x - self.vehicle_info.x) % (2 * np.pi)

        k_targ = int(
            VfhPlanner.map(target_angle, 0, 2 * np.pi, 0,
                           len(obstacle_histogram)))
        k_i = None
        if (not obstacle_histogram[k_targ]):
            k_i = k_targ
        else:
            for i in range(len(obstacle_histogram) // 2):
                index_plus = (k_targ + i) % len(obstacle_histogram)
                index_minus = (k_targ - i) % len(obstacle_histogram)
                if (not obstacle_histogram[index_plus]):
                    k_i = index_plus
                    break
                elif (not obstacle_histogram[index_minus]):
                    k_i = index_minus
                    break
            if (k_i is None):
                self.get_logger().warn(
                    "Couldn't find k_i, aborting search...\n")
                return

        left_dist = 0
        right_dist = 0
        left_alive = True
        right_alive = True
        for i in range(1, self.vfh_params.s_max):
            if (left_alive):
                index = (k_i + i) % len(obstacle_histogram)
                if (not obstacle_histogram[index]):
                    left_dist += 1
                else:
                    left_alive = False
            if (right_alive):
                index = (k_i - i) % len(obstacle_histogram)
                if (not obstacle_histogram[index]):
                    right_dist += 1
                else:
                    right_alive = False
            if (not left_alive and not right_alive):
                break
        k_steer = None
        if (left_dist + right_dist >= self.vfh_params.s_max):
            k_r = k_i - right_dist
            k_l = k_i + left_dist
            k_steer = ((k_r + k_l) // 2) % len(obstacle_histogram)
        elif (left_dist > right_dist):
            k_steer = (k_i + left_dist // 2) % len(obstacle_histogram)
        elif (right_dist >= left_dist):
            k_steer = (k_i - right_dist // 2) % len(obstacle_histogram)

        angle_steer = VfhPlanner.map(k_steer, 0, len(obstacle_histogram), 0,
                                     2 * np.pi)

        seconds = self.timer_2.timer_period_ns * 1e-9
        h_p_c = smooth_histogram[k_steer]
        h_pp_c = min(h_p_c, self.vfh_params.h_m)
        v_p = self.vfh_params.v_max * (1 - h_pp_c / self.vfh_params.h_m)
        speed_step = v_p * seconds
        dist_step = self.goal_waypoint.distance_to_xyz(self.vehicle_info.x,
                                                       self.vehicle_info.y,
                                                       self.goal_waypoint.z)
        step_size = min(speed_step, dist_step)

        new_x = self.vehicle_info.x + step_size * np.cos(angle_steer)
        new_y = self.vehicle_info.y + step_size * np.sin(angle_steer)

        self.target_waypoint.x = new_x
        self.target_waypoint.y = new_y
        self.target_waypoint.z = self.goal_waypoint.z

        # self.get_logger().warn(
        #     f"new target: ({self.target_waypoint.x:.4f}, {self.target_waypoint.y:.4f}, {self.target_waypoint.z:.4f})"
        # )
        self.get_logger().warn(
            f"\n\ttarget_angle: {target_angle:.4f},\t angle_steer: {angle_steer:.4f}\n"
        )

    @staticmethod
    def target_match_action(target: LocalCoordinates, msg: Action):
        command_bool = msg.action == Action.ACTION_WAYPOINT
        x_bool = abs(target.x - msg.x) < 1e-10
        y_bool = abs(target.y - msg.y) < 1e-10
        z_bool = abs(target.z - msg.z) < 1e-10
        return command_bool and x_bool and y_bool and z_bool

    def controller_tick(self):
        # self.get_logger().info(
        #     f"Current goal : ({self.goal_waypoint.x}, {self.goal_waypoint.y}, {self.goal_waypoint.z})\n"
        # )
        if (self.goal_waypoint == LocalCoordinates()
                and self.vehicle_info.arming_state
                == VehicleInfo.ARMING_STATE_ARMED
                and self.vehicle_info.curr_action_obj.action
                != Action.ACTION_LAND):
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
            ## This should be the part of the code that does obstacle avoidance and publishes action

            if (not VfhPlanner.target_match_action(
                    self.target_waypoint, self.vehicle_info.curr_action_obj)):
                action = Action()
                action.action = Action.ACTION_WAYPOINT
                action.x = self.target_waypoint.x
                action.y = self.target_waypoint.y
                action.z = self.target_waypoint.z

                action.yaw = np.arctan2(
                    self.goal_waypoint.y - self.vehicle_info.y,
                    self.goal_waypoint.x - self.vehicle_info.x) % (2 * np.pi)
                self.send_action(action)
            if (self.vehicle_info.curr_action_obj.action
                    != Action.ACTION_NONE):
                self.popleft_action()

    def waypoint_callback(self, msg: Waypoint):
        self.goal_waypoint.set_xyz(msg)

    def vehicle_info_callback(self, msg: VehicleInfo):
        self.vehicle_info = msg

    def laser_scan_callback(self, msg: LaserScan):
        self.vfh_params.adapt(msg)
        self.laser_scan = msg


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = VfhPlanner()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
