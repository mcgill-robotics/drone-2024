from custom_msgs.msg import VehicleInfo
from custom_msgs.msg import Action
import matplotlib.pyplot as plt
from custom_msgs.msg import Waypoint
from custom_msgs.srv import SendAction
from custom_msgs.srv import RequestAction
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


class LocalCoordinates:

    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def set_xyz(self, msg: Waypoint):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def __eq__(self, o):
        if (o is None):
            return False
        return self.x == o.x and self.y == o.y and self.z == o.z

    def __str__(self):
        return f"(x, y, z, yaw): ({self.x:.4f}, {self.y:.4f}, {self.z:.4f}, {self.yaw:.4f})\n"

    def distance_to_xyz(self, x, y, z):
        return ((self.x - x)**2 + (self.y - y)**2 + (self.z - z)**2)**(1 / 2)


class VfhParam:

    def __init__(
            self,
            angular_resolution=0.0872664,  # 5 degrees, but in radians
            a=0,
            b=1,
            robot_radius=1.0,
            threshold_low=100,
            threshold_high=500,
            s_max=16,
            v_max=10,
            mu_1=5,
            mu_2=1,
            h_m=500):
        self.angular_resolution = angular_resolution
        self.num_slots = int(np.pi * 2 / angular_resolution)
        # Set a and b such that a - b * (range_max) =  0, with a, b > 0
        self.a = a
        self.b = b
        self.robot_radius = robot_radius

        self.threshold_low = threshold_low
        self.threshold_high = threshold_high
        self.binary_polar_histogram_n_m_1 = np.full(self.num_slots, False)

        self.s_max = s_max
        self.mu_target_diff = mu_1
        self.mu_steering_diff = mu_2
        self.last_steering_dir = None

        self.v_max = v_max
        self.h_m = h_m

    def adapt(self, active_region_width):
        self.b = 0.001
        self.a = self.b * ((active_region_width - 1) / 2)**2 + 1

    # def adapt(self, msg: LaserScan):
    #     self.b = 2
    #     self.a = self.b * (msg.range_max**2) + 1
    #     self.num_slots = int(2 * np.pi / msg.angle_increment)
    #     self.threshold_low = (abs(np.arctan2(self.robot_radius, msg.range_max))
    #                           / msg.angle_increment) * 8
    #     self.threshold_high = self.threshold_low * 10
    #     if (self.binary_polar_histogram_n_m_1 is None):
    #         self.binary_polar_histogram_n_m_1 = np.full(self.num_slots, False)
    #     self.s_max = self.num_slots // 8
    #     self.h_m = self.threshold_high
    #


class VfhAlgorithm:

    def __init__(self, space_size=120, cell_size=0.25, active_region_size=60):
        """
            builds the necessary data structures for the algorithm
            space_size and cell_size relate to the configuration space C
            assumes the robot is at the center of C for simplicity
            active_region_size relates to C* it is in meteres
            (orignial paper hade it in terms of number of cells)
        """
        self.params = VfhParam()
        # things at position x, y in the plane_hist are actually at x, y in NED (from the center)
        self.plane_hist = np.zeros(
            (int(space_size / cell_size), int(space_size / cell_size)))
        self.cell_size = cell_size
        self.curr_pos: LocalCoordinates | None = None
        self.q: list[float] | None = None
        self.active_region_width = int(active_region_size / cell_size)
        self.params.adapt(self.active_region_width)
        # grid_kws = {'width_ratios': (0.9, 0.05), 'wspace': 0.2}
        self.fig, (self.ax, self.ax2) = plt.subplots(1, 2)
        # self.ax_img = self.ax.imshow(self.plane_hist.T, cmap='hot')
        # self.ax.invert_yaxis()self.ax.plot(

    def quat_apply(self, v):

        def quat_mult(q1, q2):
            a = q1[0]
            u = np.array(q1[1:])
            b = q2[0]
            v = np.array(q2[1:])
            left = a * b - np.dot(u, v)
            right = a * v + b * u + np.cross(u, v)
            return [left, *right]

        def quat_conjugate(q):
            return [q[0], *(-np.array(q[1:]))]

        v_quat = [0, *v]
        q_cong = quat_conjugate(self.q)
        tmp = quat_mult(v_quat, q_cong)
        tmp = quat_mult(self.q, tmp)
        return tmp[1:]

    @staticmethod
    def is_valid_quat(array):
        if array is None:
            return False
        if not np.any(np.isnan(array)):
            return abs(1 - np.linalg.norm(array)) <= 1e-10
        return False

    @staticmethod
    def reverse_sigmoid(x, spread=1, offset=10):
        denom = 1 + np.power(np.e, -spread * (x - offset))
        return -1 / denom + 1

    def update_detections(self, msg: LaserScan):
        # Not ready to interpret readings
        if (not self.is_valid_quat(self.q)):
            return
        std_normal = [0, 0, 1]
        normal = np.array(self.quat_apply(std_normal))
        normal = normal / np.linalg.norm(normal)
        # print(f"Current reading certainty: {normal_diff_cert}")

        self.plane_hist -= 0.5
        self.plane_hist[self.plane_hist < 0] = 0
        for index, distance in enumerate(msg.ranges):
            if not (msg.range_min <= distance <= msg.range_max):
                continue
            # angle = (abs(msg.angle_min) + self.curr_pos.yaw -
            #          index * msg.angle_increment) % (2 * np.pi)
            # distance = distance * abs(dot)
            angle = (abs(msg.angle_min) -
                     index * msg.angle_increment) % (2 * np.pi)
            plane_dx = distance * np.cos(angle)
            plane_dy = distance * np.sin(angle)
            plane_dz = self.curr_pos.z
            plane_vec = np.array([plane_dx, plane_dy, plane_dz])
            transformed_vec = np.array(self.quat_apply(plane_vec))
            dx, dy = transformed_vec[0], transformed_vec[1]
            dx, dy = int(dx / self.cell_size), int(dy / self.cell_size)
            x, y = dx + self.plane_hist.shape[
                0] // 2, dy + self.plane_hist.shape[1] // 2
            if not (0 <= x <= self.plane_hist.shape[0]) or not (
                    0 <= y <= self.plane_hist.shape[1]):
                continue
            if (self.plane_hist[x, y] <= 5):
                height_diff = abs(self.curr_pos.z - transformed_vec[2])
                height_cert = self.reverse_sigmoid(
                    height_diff, offset=abs(self.curr_pos.z) / 4)
                distance_cert = self.reverse_sigmoid(distance,
                                                     offset=msg.range_max *
                                                     height_cert)
                # cert = distance_cert * normal_diff_cert
                # cert = distance_cert
                self.plane_hist[x, y] += distance_cert
                # self.plane_hist[x, y] += 1

    def update_position(self, msg: VehicleInfo):
        if (self.curr_pos is None):
            self.curr_pos = LocalCoordinates(msg.x, msg.y, msg.z, msg.heading)
            self.q = msg.q
        tx, ty = -int((msg.x - self.curr_pos.x) / self.cell_size), -int(
            (msg.y - self.curr_pos.y) / self.cell_size)
        N, M = self.plane_hist.shape
        image_translated = np.zeros_like(self.plane_hist)
        image_translated[max(tx, 0):M + min(tx, 0),
                         max(ty, 0):N +
                         min(ty, 0)] = self.plane_hist[-min(tx, 0):M -
                                                       max(tx, 0),
                                                       -min(ty, 0):N -
                                                       max(ty, 0)]
        self.plane_hist = image_translated
        self.curr_pos = LocalCoordinates(msg.x, msg.y, msg.z, msg.heading)
        self.q = msg.q

    def generate_smooth(self, active_region):
        # CCW angles , index 0 at angle 0
        polar_histogram = np.zeros(self.params.num_slots)
        x_0, y_0 = int(active_region.shape[0] / 2), int(
            active_region.shape[1] / 2)
        for x in range(active_region.shape[0]):
            for y in range(active_region.shape[1]):
                if (x == x_0 and y == y_0):
                    continue
                cert = active_region[x][y]
                dy = y - y_0
                dx = x - x_0
                dist = ((dx)**2 + (dy)**2)**(1 / 2)
                angle = np.arctan2(dy, dx) % (2 * np.pi)
                magn = cert**2 * (self.params.a - self.params.b * dist**2)
                gamma = np.arcsin(self.params.robot_radius / dist)
                if (magn != 0):
                    # print(f"Found magnitude for {x}, {y}, {magn}")
                    for k in range(self.params.num_slots):
                        if (angle - gamma <= k * self.params.angular_resolution
                                <= angle + gamma):
                            polar_histogram[k] += magn
                            # print(f"+1", end=" ")
                    # print("\n")
        return polar_histogram

    def generate_masked_histogram(self, smooth_histogram):
        res = np.copy(self.params.binary_polar_histogram_n_m_1)
        res[smooth_histogram > self.params.threshold_high] = True
        res[smooth_histogram < self.params.threshold_low] = False
        self.params.binary_polar_histogram_n_m_1 = res
        return res

    def retrieve_valleys(self, obstacle_histogram):
        valleys = []
        valley_tmp = []
        explored = 0
        index = 0
        # obstacles
        if (np.any(obstacle_histogram) and not np.all(obstacle_histogram)):
            # walk backwards to find first hill
            next_index = (index + 1) % len(obstacle_histogram)
            while not (obstacle_histogram[index]
                       and not obstacle_histogram[next_index]):
                index = (index - 1) % len(obstacle_histogram)
                next_index = (index + 1) % len(obstacle_histogram)

            while explored < len(obstacle_histogram):
                index = index % len(obstacle_histogram)
                next_index = (index + 1) % len(obstacle_histogram)
                if (obstacle_histogram[index]
                        and not obstacle_histogram[next_index]):
                    # adding the K_rights
                    valley_tmp.append(next_index)
                if (not obstacle_histogram[index]
                        and obstacle_histogram[next_index]):
                    # set k_left, reset k_tmp
                    valley_tmp.append(next_index)
                    valleys.append(valley_tmp)
                    valley_tmp = []
                index += 1
                explored += 1
        return valleys

    def sector_distance(self, s1, s2):
        return min(abs(s1 - s2), abs(s1 - s2 + self.params.num_slots),
                   abs(s1 - s2 - self.params.num_slots))

    def retrieve_candidates(self, valleys, target_sector):
        candidates = []
        if (len(valleys) == 0):
            candidates.append(target_sector)
            return candidates
        for k_right, k_left in valleys:
            distance = self.sector_distance(k_right, k_left)
            if (distance < self.params.s_max):
                # narrow
                candidates.append(
                    ((k_right + k_left) // 2) % self.params.num_slots)
            else:
                # wide
                c_r = (k_right +
                       self.params.s_max // 2) % self.params.num_slots
                c_l = (k_left - self.params.s_max // 2) % self.params.num_slots
                candidates.append(c_r)
                candidates.append(c_l)
                if (c_l < c_r):
                    # Wrap issue
                    if not (c_l <= target_sector <= c_r):
                        candidates.append(target_sector)
                else:
                    # no wrap issue
                    if (c_r <= target_sector <= c_l):
                        candidates.append(target_sector)
        return candidates

    def choose_candidate(self, candidates, target_sector):
        cost = []
        for candidate in candidates:
            cost_tmp = self.params.mu_target_diff * self.sector_distance(
                candidate, target_sector)
            if self.params.last_steering_dir is not None:
                cost_tmp += self.params.mu_steering_diff * self.sector_distance(
                    candidate, self.params.last_steering_dir)
            cost.append(cost_tmp)
        min_cost = np.min(cost)
        index = cost.index(min_cost)
        k_steer = candidates[index]
        self.params.last_steering_dir = k_steer
        return k_steer

    def calculate_movement_vec(self, candidate, dt_secs, smoothed_value,
                               goal_pos):
        angle = candidate * self.params.angular_resolution
        h_pp_c = min(smoothed_value, self.params.h_m)
        v_p = self.params.v_max * (1 - h_pp_c / self.params.h_m)
        speed_step = v_p * dt_secs

        dist_step = goal_pos.distance_to_xyz(self.curr_pos.x, self.curr_pos.y,
                                             goal_pos.z)

        step_size = min(speed_step, dist_step)

        return step_size * np.cos(angle), step_size * np.sin(angle)

    def generate_target(self, goal_pos: LocalCoordinates,
                        vehicle_info: VehicleInfo, laser_scan: LaserScan,
                        dt_secs: float):
        self.update_position(vehicle_info)
        self.update_detections(laser_scan)
        N, M = self.plane_hist.shape
        half_x = int(N / 2)
        half_y = int(M / 2)
        half_width = int(self.active_region_width / 2)
        active_region = self.plane_hist[half_x - half_width:half_x +
                                        half_width, half_y -
                                        half_width:half_y + half_width]
        # self.ax.imshow(self.plane_hist.T, cmap='hot')
        # self.ax.invert_yaxis()
        # # plt.show()
        # plt.pause(0.001)
        smooth_active = self.generate_smooth(active_region)
        obstacle_histogram = self.generate_masked_histogram(smooth_active)

        # self.ax2.imshow(active_region.T, cmap='hot')
        # self.ax2.invert_yaxis()
        # self.ax.clear()
        # self.ax.plot(
        #     np.arange(0, 2 * np.pi - self.params.angular_resolution,
        #               self.params.angular_resolution), obstacle_histogram)
        # plt.pause(0.001)

        valleys = self.retrieve_valleys(obstacle_histogram)

        print(f"Valleys: {valleys}\n")

        target_angle = np.arctan2(goal_pos.y - vehicle_info.y,
                                  goal_pos.x - vehicle_info.x) % (2 * np.pi)
        target_sector = int(target_angle / self.params.angular_resolution)
        candidates = self.retrieve_candidates(valleys, target_sector)
        chosen_candidate = self.choose_candidate(candidates, target_sector)
        dx, dy = self.calculate_movement_vec(chosen_candidate, dt_secs,
                                             smooth_active[chosen_candidate],
                                             goal_pos)

        new_x = self.curr_pos.x + dx
        new_y = self.curr_pos.y + dy
        new_z = goal_pos.z
        new_yaw = goal_pos.yaw
        new_target = LocalCoordinates(new_x, new_y, new_z, new_yaw)
        print(f"New target : {new_target}")
        return new_target


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
        self.timer_2 = self.create_timer(timer_period / 2,
                                         self.planner_tick_v3)

        self.planner = VfhAlgorithm()

    def send_action(self, action: Action):
        self.action_req.action = action
        return self.enqueue_action_client.call_async(self.action_req)

    def popleft_action(self):
        return self.popleft_action_client.call_async(self.action_req_req)

    def planner_tick_v3(self):
        if (self.vehicle_info is None or self.laser_scan is None):
            return
        self.target_waypoint = self.planner.generate_target(
            self.goal_waypoint, self.vehicle_info, self.laser_scan,
            self.timer_2.timer_period_ns * 1e-9)

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

                action.yaw = np.arctan2(
                    self.goal_waypoint.y - self.vehicle_info.y,
                    self.goal_waypoint.x - self.vehicle_info.x) % (2 * np.pi)
                self.send_action(action)
            if (self.vehicle_info.curr_action_obj.action != Action.ACTION_NONE
                    and not VfhPlanner.target_match_action(
                        self.target_waypoint,
                        self.vehicle_info.curr_action_obj)):
                self.popleft_action()

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
