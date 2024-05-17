import time
import matplotlib.pyplot as plt
from custom_msgs.msg import VehicleInfo
from sensor_msgs.msg import LaserScan
from .geometry import *

# plt.ion()


class VfhParam:

    def __init__(
            self,
            angular_resolution=0.0872664,  # 5 degrees, but in radians
            a=0,
            b=1,
            robot_radius=1.5,
            threshold_low=1,
            threshold_high=5,
            s_max=20,
            v_max=8,
            mu_1=5,
            mu_2=2,
            mu_3=2,
            h_m=4.5):
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
        self.mu_valley_width = mu_3
        self.last_steering_dir = None

        self.v_max = v_max
        self.h_m = h_m

    def adapt(self, active_region_width):
        self.b = 0.001
        self.a = self.b * ((2**(1 / 2)) * (active_region_width + 4) / 2)**2 + 1


class VfhAlgorithm:

    def __init__(self, space_size=120, cell_size=0.5, active_region_size=30):
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
        self.min_active_region_width = int(active_region_size / cell_size)
        self.params.adapt(self.min_active_region_width)
        # grid_kws = {'width_ratios': (0.9, 0.05), 'wspace': 0.2}
        self.fig, ((self.ax, self.ax2), (self.ax3,
                                         self.ax4)) = plt.subplots(2, 2)
        # self.fig.canvas.draw_idle()
        # self.fig.canvas.start_event_loop(0.001)
        self.last_time = None
        self.max_val_grid_cell = 1
        # self.ax_img = self.ax.imshow(self.plane_hist.T, cmap='hot')
        # self.ax.invert_yaxis()self.ax.plot(

    def change_max_speed(self, new_max_speed):
        self.params.v_max = new_max_speed

    @staticmethod
    def quat_conjugate(q):
        quat = np.array([q[0], *(-np.array(q[1:]))])
        return quat / np.linalg.norm(quat)

    def quat_apply(self, v):

        def quat_mult(q1, q2):
            a = q1[0]
            u = np.array(q1[1:])
            b = q2[0]
            v = np.array(q2[1:])
            left = a * b - np.dot(u, v)
            right = a * v + b * u + np.cross(u, v)
            return [left, *right]

        v_quat = [0, *v]
        q_cong = self.quat_conjugate(self.q)
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
    def reverse_sigmoid(x, spread=1, offset=10.):
        denom = 1 + np.power(np.e, -spread * (x - offset))
        return -1 / denom + 1

    def update_detections(self, msg: LaserScan):
        # Not ready to interpret readings
        if (not self.is_valid_quat(self.q)):
            return
        # print(f"Current reading certainty: {normal_diff_cert}")
        max_val = self.max_val_grid_cell
        i_vec = [1, 0, 0]
        j_vec = [0, 1, 0]
        i_trans = np.array(self.quat_apply(i_vec))
        i_trans = i_trans / np.linalg.norm(i_trans)
        j_trans = np.array(self.quat_apply(j_vec))
        j_trans = j_trans / np.linalg.norm(j_trans)
        for index, distance in enumerate(msg.ranges):
            if not (msg.range_min <= distance <= msg.range_max):
                continue
            # angle = (abs(msg.angle_min) + self.curr_pos.yaw -
            #          index * msg.angle_increment) % (2 * np.pi)
            # distance = distance * abs(dot)
            angle = (abs(msg.angle_min) -
                     index * msg.angle_increment) % (2 * np.pi)
            reading_vec = distance * (np.cos(angle) * i_trans + np.sin(angle) * j_trans)\
                + np.array([0, 0, self.curr_pos.z])

            dx, dy = reading_vec[0], reading_vec[1]
            if (abs(reading_vec[2]) <= 1.0):
                print("GROUND DETECTED!", end=" ")
                continue
            dx, dy = int(dx / self.cell_size), int(dy / self.cell_size)
            x, y = dx + self.plane_hist.shape[
                0] // 2, dy + self.plane_hist.shape[1] // 2
            if not (0 <= x <= self.plane_hist.shape[0]) or not (
                    0 <= y <= self.plane_hist.shape[1]):
                continue
            if (self.plane_hist[x, y] <= max_val):
                # height_diff = abs(self.curr_pos.z - reading_vec[2])
                # height_cert = self.reverse_sigmoid(
                #     height_diff, offset=abs(self.curr_pos.z) / 4)
                # distance_cert = self.reverse_sigmoid(distance,
                #                                      offset=msg.range_max *
                #                                      height_cert)
                # cert = distance_cert * normal_diff_cert
                # cert = distance_cert
                self.plane_hist[
                    x, y] += self.max_val_grid_cell / 5 if self.plane_hist[
                        x,
                        y] < max_val - self.max_val_grid_cell / 5 else self.max_val_grid_cell / 10
                # self.plane_hist[x, y] += 1

        self.plane_hist -= self.max_val_grid_cell / 10
        self.plane_hist[self.plane_hist < 0] = 0
        self.plane_hist[self.plane_hist > max_val] = max_val

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
                if (dist <= self.params.robot_radius / self.cell_size):
                    if (cert <= 1e-10):
                        continue
                    ## MAYDAY
                    gamma = np.pi
                else:
                    gamma = np.arcsin(self.params.robot_radius /
                                      (dist * self.cell_size))
                if (magn != 0):
                    # print(f"Found magnitude for {x}, {y}, {magn}")
                    for k in range(self.params.num_slots):
                        r = angle - gamma
                        l = angle + gamma
                        sec_angle = k * self.params.angular_resolution
                        d_r_a = self.modulo_r_l_distance(
                            r, sec_angle, 2 * np.pi)
                        d_a_l = self.modulo_r_l_distance(
                            sec_angle, l, 2 * np.pi)
                        d_r_l = self.modulo_r_l_distance(r, l, 2 * np.pi)
                        if (d_r_a + d_a_l <= d_r_l + 1e-10):
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
        if (np.all(obstacle_histogram)):
            print("ERMMMM AM COOKED! [Walls surround the drone]")
            return None
        if (np.any(obstacle_histogram)):
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
                elif (not obstacle_histogram[index]
                      and obstacle_histogram[next_index]):
                    # set k_left, reset k_tmp
                    valley_tmp.append(index)
                    valleys.append(valley_tmp)
                    valley_tmp = []
                index += 1
                explored += 1
        return valleys

    # shortest ditance
    def sector_distance(self, s1, s2):
        return min(abs(s1 - s2), abs(s1 - s2 + self.params.num_slots),
                   abs(s1 - s2 - self.params.num_slots))

    # s1 -> s2 distance
    def sector_r_l_distance(self, r, l):
        if (l >= r):
            return l - r
        return self.params.num_slots - self.sector_r_l_distance(l, r)

    def modulo_r_l_distance(self, r, l, mod):
        r = r % mod
        l = l % mod
        if (l >= r):
            return l - r
        return mod - self.modulo_r_l_distance(l, r, mod)

    def retrieve_candidates(self, valleys, target_sector):
        candidates = []
        candidate_origin = []
        if (len(valleys) == 0):
            candidates.append(target_sector)
            candidate_origin.append(None)
            return candidates, candidate_origin
        for k_right, k_left in valleys:
            distance = self.sector_r_l_distance(k_right, k_left)
            if (distance < self.params.s_max):
                d_r_l = self.sector_r_l_distance(k_right, k_left)
                # narrow
                candidates.append(
                    int((k_right + d_r_l / 2)) % self.params.num_slots)
                candidate_origin.append([k_right, k_left])
            else:
                # wide
                c_r = (k_right +
                       self.params.s_max // 2) % self.params.num_slots
                c_l = (k_left - self.params.s_max // 2) % self.params.num_slots
                candidates.append(c_r)
                candidates.append(c_l)
                candidate_origin.append([k_right, k_left])
                candidate_origin.append([k_right, k_left])
                d_r_t = self.sector_r_l_distance(c_r, target_sector)
                d_t_l = self.sector_r_l_distance(target_sector, c_l)
                d_r_l = self.sector_r_l_distance(c_r, c_l)
                if (d_r_t + d_t_l <= d_r_l):
                    candidates.append(target_sector)
                    candidate_origin.append([k_right, k_left])

        return candidates, candidate_origin

    def choose_candidate(self, candidates, candidate_origins, target_sector):
        cost = []
        for index, candidate in enumerate(candidates):
            cost_tmp = self.params.mu_target_diff * self.sector_distance(
                candidate, target_sector)
            if self.params.last_steering_dir is not None:
                cost_tmp += self.params.mu_steering_diff * self.sector_distance(
                    candidate, self.params.last_steering_dir)
            if candidate_origins[index] is not None:
                cost_tmp += self.params.mu_valley_width \
                    * 1 / ( 1e-10 + self.sector_r_l_distance(candidate_origins[index][0], candidate_origins[index][1]))

            cost.append(cost_tmp)
        min_cost = np.min(cost)
        index = cost.index(min_cost)
        k_steer = candidates[index]
        self.params.last_steering_dir = k_steer
        return k_steer, cost

    def get_time_diff(self, dt_secs):
        if (self.last_time is None):
            self.last_time = time.time()
            return dt_secs
        curr = time.time()
        diff = curr - self.last_time
        self.last_time = curr
        return diff + dt_secs

    def calculate_movement_vec(self, candidate, dt_secs, smoothed_value,
                               goal_pos):
        angle = candidate * self.params.angular_resolution
        h_pp_c = min(smoothed_value, self.params.h_m)
        reduc = (1 - h_pp_c / self.params.h_m)
        print(f"ANGLE :{angle:.4f}, smoothed_value :{smoothed_value:.4f}")
        print(f"REDUCTION FACTOR FOR SPEED: {reduc*100:.4f}%")
        v_p = self.params.v_max * reduc
        # speed_step = v_p * self.get_time_diff(dt_secs)

        dist_step = goal_pos.distance_to_xyz(self.curr_pos.x, self.curr_pos.y,
                                             goal_pos.z)

        # step_size = min(speed_step, dist_step)
        step_size = min(20, dist_step)

        return step_size * np.cos(angle), step_size * np.sin(angle), v_p

    def add_obstacles_to_region(self, region: np.ndarray, obstacles):
        N, M = region.shape
        res = region.copy()
        for x in range(N):
            for y in range(M):
                dx = x - N // 2
                dy = y - M // 2
                dx, dy = dx * self.cell_size, dy * self.cell_size
                real_x = self.curr_pos.x + dx
                real_y = self.curr_pos.y + dy
                for obstacle in obstacles:
                    if (obstacle.contains(real_x, real_y)):
                        res[x][y] = self.max_val_grid_cell
        return res

    def generate_target(self, goal_pos: LocalCoordinates,
                        vehicle_info: VehicleInfo,  laser_scan: LaserScan,
                        obstacles: list[Obstacle], dt_secs: float):
        self.update_position(vehicle_info)
        # self.update_detections(laser_scan)
        dist_from_goal = ((goal_pos.x - self.curr_pos.x)**2 + (goal_pos.y - self.curr_pos.y)**2)**(1/2) // self.cell_size 
        self.active_region_width = min(self.min_active_region_width, max(dist_from_goal * 2 + 1, 3))
        self.params.adapt(self.active_region_width)
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
        active_region_with_obstacles = self.add_obstacles_to_region(
            active_region, obstacles)

        smooth_active = self.generate_smooth(active_region_with_obstacles)
        obstacle_histogram = self.generate_masked_histogram(smooth_active)

        # self.ax4.imshow(self.plane_hist.T, cmap='hot')
        # self.ax4.invert_yaxis()
        # self.ax3.imshow(active_region_with_obstacles.T, cmap='hot')
        # self.ax3.invert_yaxis()
        # self.ax2.clear()
        # self.ax2.plot(range(self.params.num_slots), smooth_active)
        # self.ax.clear()
        # self.ax.plot(range(self.params.num_slots), obstacle_histogram)
        #
        # self.fig.canvas.draw_idle()
        # self.fig.canvas.start_event_loop(0.001)
        # #
        valleys = self.retrieve_valleys(obstacle_histogram)

        print(f"Valleys: {valleys}\n")
        target_angle = np.arctan2(goal_pos.y - vehicle_info.y,
                                  goal_pos.x - vehicle_info.x) % (2 * np.pi)
        target_to_print_angle = np.arctan2(goal_pos.y - vehicle_info.y,
                                           goal_pos.x - vehicle_info.x)
        if (valleys is None):
            return LocalCoordinates(
                self.curr_pos.x,
                self.curr_pos.y,
                self.curr_pos.z,
            )
        # if (VfhPlanner.angle_dist(target_angle,vehicle_info.heading % (2 * np.pi)) >= np.pi / 4):
        #     print(f"\t\ttarget angle: {target_to_print_angle}, curr angle: {vehicle_info.heading}")
        target_sector = int(target_angle / self.params.angular_resolution)
        candidates, candidate_origin = self.retrieve_candidates(
            valleys, target_sector)
        chosen_candidate, costs = self.choose_candidate(
            candidates, candidate_origin, target_sector)
        print(f"Candidates : {candidates}")
        print(f"Candidate Costs: {costs}")
        print(f"Candidates Origins : {candidate_origin}")
        print(f"Candidate : {chosen_candidate}")
        print(f"Target : {target_sector}")
        dx, dy, v_p = self.calculate_movement_vec(
            chosen_candidate, dt_secs, smooth_active[chosen_candidate],
            goal_pos)

        new_x = self.curr_pos.x + dx
        new_y = self.curr_pos.y + dy
        new_z = goal_pos.z
        new_yaw = target_to_print_angle
        new_target = LocalCoordinates(new_x, new_y, new_z, new_yaw, v_p)
        print(f"New target : {new_target}")
        return new_target
