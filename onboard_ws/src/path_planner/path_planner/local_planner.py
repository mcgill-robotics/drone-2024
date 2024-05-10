from custom_msgs.msg import VehicleInfo
from custom_msgs.msg import Action
import matplotlib.pyplot as plt
import seaborn as sns
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

    def __init__(self, x=0.0, y=0.0, z=0.0, yaw: float | None =None):
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

    def distance_to_xyz(self, x, y, z):
        return ((self.x - x)**2 + (self.y - y)**2 + (self.z - z)**2)**(1 / 2)




class VfhParam:

    def __init__(self,
                 a=0,
                 b=1,
                 certainty=1,
                 robot_radius=2.5,
                 threshold_low=10,
                 threshold_high=20,
                 s_max=25,
                 v_max=10,
                 mu_1=5,
                 mu_2=1,
                 h_m=10):
        # Set a and b such that a - b * (range_max) =  0, with a, b > 0
        self.a = a
        self.b = b
        self.cert = certainty
        self.robot_radius = robot_radius

        self.num_slots = 360
        self.threshold_low = threshold_low
        self.threshold_high = threshold_high
        self.binary_polar_histogram_n_m_1 = None

        self.s_max = s_max
        self.mu_target_diff = mu_1
        self.mu_steering_diff = mu_2
        self.last_steering_dir = None

        self.v_max = v_max
        self.h_m = h_m

    def adapt(self, msg: LaserScan):
        self.b = 2
        self.a = self.b * (msg.range_max**2) + 1
        self.num_slots = int(2 * np.pi / msg.angle_increment)
        self.threshold_low = (abs(np.arctan2(self.robot_radius, msg.range_max))
                              / msg.angle_increment) * 8
        self.threshold_high = self.threshold_low * 10
        if (self.binary_polar_histogram_n_m_1 is None):
            self.binary_polar_histogram_n_m_1 = np.full(self.num_slots, False)
        self.s_max = self.num_slots // 8
        self.h_m = self.threshold_high


class VfhAlgorithm:

    
    def quat_apply(self, v):
        def quat_mult(q1, q2):
            a = q1[0]
            u = np.array(q1[1:])
            b = q2[0]
            v = np.array(q2[1:])
            left = a*b - np.dot(u,v) 
            right = a*v + b*u + np.cross(u, v)
            return [left, *right]
        def quat_conjugate(q):
            return [q[0], *(-np.array(q[1:]))]
        v_quat = [0, *v]
        q_cong = quat_conjugate(self.q)
        tmp = quat_mult(v_quat, q_cong)
        tmp = quat_mult(self.q, tmp)
        return tmp[1:]

    def __init__(self, space_size=60, cell_size=0.25):
        """
            builds the necessary data structures for the algorithm
            space_size and cell_size relate to the configuration space C
            assumes the robot is at the center of C for simplicity
        """
        self.params = VfhParam()
        self.plane_hist = np.zeros(
            (int(space_size / cell_size), int(space_size / cell_size)))
        self.cell_size = cell_size
        self.curr_pos: LocalCoordinates | None = None
        self.q: list[float] | None= None
        grid_kws = {'width_ratios': (0.9, 0.05), 'wspace': 0.2}
        self.fig, (self.ax, self.cbar_ax) = plt.subplots(1, 2, gridspec_kw = grid_kws)

    def update_detections(self, msg: LaserScan):
        # Not ready to interpret readings
        if (self.q is None):
            return
        std_normal = [0, 0, 1]
        normal = np.array(self.quat_apply(std_normal)) 
        normal = normal / np.linalg.norm(normal)
        reading_cert = abs(np.dot(std_normal, normal))
        print(f"Current reading certainty: {reading_cert}")
        
        self.plane_hist -= 0.5
        self.plane_hist[self.plane_hist < 0] = 0
        for index, distance in enumerate(msg.ranges):
            distance = min(distance, msg.range_max)
            distance = max(distance, msg.range_min)
            angle = (index * msg.angle_increment + msg.angle_min + self.curr_pos.yaw) % (2*np.pi)
            dx = distance * np.cos(angle)
            dy = distance * np.sin(angle)
            dx, dy = int(dx / self.cell_size), int(dy / self.cell_size)
            x, y = dx + self.plane_hist.shape[0] // 2, dy + self.plane_hist.shape[1] // 2
            if (self.plane_hist[x, y] <= 10):
                self.plane_hist[x, y] += reading_cert

        self.plane_hist[0, 0] = 20
        self.plane_hist[10, 10] = 20
        sns.heatmap(data=self.plane_hist, ax=self.ax, cbar_ax=self.cbar_ax)
        plt.draw()
        # plt.show()
        plt.pause(0.1)



    def update_position(self, msg: VehicleInfo):
        if (self.curr_pos is None):
            self.curr_pos = LocalCoordinates(msg.x, msg.y, msg.z, msg.heading)
            self.q = msg.q
        tx, ty = int((msg.x - self.curr_pos.x) / self.cell_size), int((msg.y - self.curr_pos.y) / self.cell_size)
        N, M = self.plane_hist.shape
        image_translated = np.zeros_like(self.plane_hist)
        image_translated[max(tx,0):M+min(tx,0), max(ty,0):N+min(ty,0)] = self.plane_hist[-min(tx,0):M-max(tx,0), -min(ty,0):N-max(ty,0)]
        self.plane_hist = image_translated
        self.curr_pos = LocalCoordinates(msg.x, msg.y, msg.z, msg.heading)
        self.q = msg.q
        

    def generate_target(self, goal_pos: LocalCoordinates):
        
        pass


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
        # self.timer_1 = self.create_timer(timer_period, self.controller_tick)
        # self.goal_waypoint = LocalCoordinates(0.0, 10.0, -10.0)
        self.timer_2 = self.create_timer(timer_period * 2,
                                         self.planner_tick_v3)

        self.planner = VfhAlgorithm()

    def send_action(self, action: Action):
        self.action_req.action = action
        return self.enqueue_action_client.call_async(self.action_req)

    def popleft_action(self):
        return self.popleft_action_client.call_async(self.action_req_req)

    @staticmethod
    def map(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def sector_distance(self, s1, s2):
        return min(abs(s1 - s2), abs(s1 - s2 + self.vfh_params.num_slots),
                   abs(s1 - s2 - self.vfh_params.num_slots))

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

        distances = np.copy(histogram)
        histogram = self.vfh_params.cert**2 * (
            self.vfh_params.a - self.vfh_params.b * histogram**2)
        # smoothing
        smooth_histogram = np.zeros(len(histogram))
        for i in range(len(histogram)):
            angle_coverage = np.arctan2(self.vfh_params.robot_radius,
                                        distances[i]) % (2 * np.pi)
            min_angle = (i * self.laser_scan.angle_increment - angle_coverage)
            max_angle = (i * self.laser_scan.angle_increment + angle_coverage)
            index_angle = VfhPlanner.map(i, 0, len(histogram), 0, 2 * np.pi)
            index_distance = distances[i]
            for angle in np.arange(min_angle, max_angle,
                                   self.laser_scan.angle_increment):
                in_histogram_index = int(
                    VfhPlanner.map(angle % (2 * np.pi), 0, 2 * np.pi, 0,
                                   len(histogram)))
                other_point_distance = distances[in_histogram_index]
                p1 = LocalCoordinates(index_distance * np.cos(index_angle),
                                      index_distance * np.sin(index_angle),
                                      0.0)
                p2 = LocalCoordinates(other_point_distance * np.cos(angle),
                                      other_point_distance * np.sin(angle),
                                      0.0)
                if (p1.distance_to_xyz(p2.x, p2.y, p2.z)
                        < self.vfh_params.robot_radius):
                    smooth_histogram[i] += histogram[in_histogram_index]

        # masking False if valley, True if obstacle
        obstacle_histogram = np.copy(
            self.vfh_params.binary_polar_histogram_n_m_1)
        obstacle_histogram[smooth_histogram >
                           self.vfh_params.threshold_high] = True
        obstacle_histogram[smooth_histogram <
                           self.vfh_params.threshold_low] = False
        self.vfh_params.binary_polar_histogram_n_m_1 = np.copy(
            obstacle_histogram)

        # find valleys
        valleys = []
        valley_tmp = []
        explored = 0
        index = 0
        # obstacles
        self.get_logger().info("-" * 40 + "\n")
        self.get_logger().warn(
            f"thresholds: ({self.vfh_params.threshold_low}, {self.vfh_params.threshold_high})\n"
        )
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
        self.get_logger().warn(f"found valleys: {valleys}\n")

        # get k_n
        target_angle = np.arctan2(
            self.goal_waypoint.y - self.vehicle_info.y,
            self.goal_waypoint.x - self.vehicle_info.x) % (2 * np.pi)

        k_targ = int(
            VfhPlanner.map(target_angle, 0, 2 * np.pi, 0,
                           len(obstacle_histogram)))
        candidates = []
        if (len(valleys) == 0):
            candidates.append(k_targ)
        for k_right, k_left in valleys:
            if k_left < k_right:
                k_left += len(obstacle_histogram)
            distance = k_left - k_right
            if (distance < self.vfh_params.s_max):
                # narrow
                candidates.append(
                    ((k_right + k_left) // 2) % len(obstacle_histogram))
            else:
                # wide
                c_r = (k_right +
                       self.vfh_params.s_max // 2) % len(obstacle_histogram)
                c_l = (k_left -
                       self.vfh_params.s_max // 2) % len(obstacle_histogram)
                candidates.append(c_r)
                candidates.append(c_l)
                if (c_l < c_r):
                    # Wrap issue
                    if not (c_l <= k_targ <= c_r):
                        c_t = k_targ
                        candidates.append(c_t)
                else:
                    # no wrap issue
                    if (c_r <= k_targ <= c_l):
                        c_t = k_targ
                        candidates.append(c_t)
        self.get_logger().warn(f"Candidates : {candidates}\n")
        cost = []
        for candidate in candidates:
            cost_tmp = self.vfh_params.mu_target_diff * self.sector_distance(
                candidate, k_targ)
            if self.vfh_params.last_steering_dir is not None:
                cost_tmp += self.vfh_params.mu_steering_diff * self.sector_distance(
                    candidate, self.vfh_params.last_steering_dir)
            cost.append(cost_tmp)
        min_cost = np.min(cost)
        index = cost.index(min_cost)
        k_steer = candidates[index]
        self.vfh_params.last_steering_dir = k_steer

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
        self.target_waypoint.yaw = target_angle

        self.get_logger().warn(
            f"new target: ({self.target_waypoint.x:.4f}, {self.target_waypoint.y:.4f}, {self.target_waypoint.z:.4f})"
        )
        self.get_logger().warn(
            f"\n\tk_targ : {k_targ},\t k_steer: {k_steer}\n")

    def planner_tick_v3(self):
        target = self.planner.generate_target(self.goal_waypoint)

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

            if (not VfhPlanner.target_match_action(
                    self.target_waypoint, self.vehicle_info.curr_action_obj)):
                action = Action()
                action.action = Action.ACTION_WAYPOINT
                action.x = self.target_waypoint.x
                action.y = self.target_waypoint.y
                action.z = self.target_waypoint.z
                action.yaw = self.target_waypoint.yaw

                # action.yaw = np.arctan2(
                #     self.goal_waypoint.y - self.vehicle_info.y,
                #     self.goal_waypoint.x - self.vehicle_info.x) % (2 * np.pi)
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
        self.planner.update_position(msg)

    def laser_scan_callback(self, msg: LaserScan):
        self.laser_scan = msg
        self.planner.update_detections(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = VfhPlanner()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
