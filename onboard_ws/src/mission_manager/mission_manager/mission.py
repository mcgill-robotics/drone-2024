import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from custom_msgs.msg import Waypoint
from custom_msgs.msg import Target
from custom_msgs.msg import ListTargets
from custom_msgs.msg import GlobalCoordinates
from custom_msgs.msg import ListGlobalCoordinates
from custom_msgs.msg import VehicleInfo
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import pymap3d as pm
import numpy as np


class MissionNode(Node):
    WAITING = 0
    PERFORMIN_LAP = 1
    SURVEY_AREA = 2
    PERFORMING_AIRDROP = 3
    RTH = 4

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

        self.survey_zone_sub = self.create_subscription(
            ListGlobalCoordinates, "/survey_zone", self.survey_zone_callback,
            self.qos_profile)
        self.survey_zone = None
        self.survey_path = None
        self.survey_path_curr_index = 0
        self.survey_zone_toggle = False

        self.lap_points_sub = self.create_subscription(
            ListGlobalCoordinates, "/mission_lap", self.mission_lap_callback,
            self.qos_profile)
        self.lap = None
        self.curr_lap_point_index = 0
        self.lap_toggle = False

        self.airdrop_toggle = False
        self.rth_toggle = False

        self.survey_toggle = False
        self.at_survey_zone = False

        self.target_list_sub = self.create_subscription(
            ListTargets, "/mission_targets", self.targets_callback,
            self.qos_profile)
        self.targets = None
        self.top_target_published = False

        self.target_location_sub = self.create_subscription(
            Point32, "/target_location", self.target_location_callback,
            self.qos_profile)
        self.target_location: Point32 | None = None

        self.lap_tolerance_sub = self.create_subscription(
            Float32, "/mission_lap_tolerance", self.lap_tolerance_callback,
            self.qos_profile)
        self.curr_tolerance = 3.0

        self.mission_time_sub = self.create_subscription(
            Float32, "/mission_time", self.mission_time_callback,
            self.qos_profile)
        self.mission_time = 28 * 60

        self.mission_status_pub = self.create_publisher(
            String, "/mission_status", self.qos_profile)

        self.boundary_setter_pub = self.create_publisher(
            Polygon, "/boundary_setter", 10)

        self.goal_waypoint_pub = self.create_publisher(Waypoint,
                                                       "/goal_waypoint",
                                                       self.qos_profile)

        self.curr_target_pub = self.create_publisher(Target, "/current_target",
                                                     self.qos_profile)

        self.airdrop_pub = self.create_publisher(Int32, "/drop_bottle", 10)

        self.start_sub = self.create_subscription(Empty, "/start",
                                                  self.start_callback,
                                                  self.qos_profile)
        self.start_time = None
        self.state = MissionNode.WAITING

        timer_period = 0.5
        self.timer_1 = self.create_timer(timer_period, self.mission_tick)
        self.timer_2 = self.create_timer(timer_period * 2, self.periodic_task)

    def describe_state(self):
        self.warn_pub("Nothing to report")

    def periodic_task(self):
        self.describe_state()
        if (self.start_time is not None):
            curr_time = self.get_clock().now().seconds_nanoseconds()
            time_diff = (curr_time[0] + curr_time[1] * 1e-9) - (
                self.start_time[0] + self.start_time[1] * 1e-9)
            if (time_diff >= self.mission_time):
                self.state = self.RTH

        # TODO: Stop mission if batery percentage is below a certain threshold

    def warn_pub(self, string):
        self.get_logger().warn(string)
        string_msg = String()
        string_msg.data = f"State: {'WAITING' if self.state == self.WAITING else 'PERFORMING_LAP' if self.state == self.PERFORMIN_LAP else 'SURVEY_AREA' if self.state == self.SURVEY_AREA else 'PERFORMING_AIRDROP' if self.state == self.PERFORMING_AIRDROP else 'RTH'}, {string}"
        self.mission_status_pub.publish(string_msg)

    def vehicle_info_callback(self, msg: VehicleInfo):
        self.curr_n, self.curr_e, self.curr_d = msg.x, msg.y, msg.z
        self.lat0, self.lon0, self.alt0 = msg.ref_lat, msg.ref_lon, msg.ref_alt

    def get_ned(self, lat, lon, alt):
        if self.lat0 is None:
            self.warn_pub(
                "Not quite ready to translate coordinates to ned, waiting on vehicle info..."
            )
            return None
        n, e, d = pm.geodetic2ned(lat, lon, alt, self.lat0, self.lon0,
                                  self.alt0)
        return n, e, d

    def mission_boundary_callback(self, msg: ListGlobalCoordinates):
        if self.lat0 is None:
            self.warn_pub(
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

    def survey_zone_callback(self, msg: ListGlobalCoordinates):
        if self.lat0 is None:
            self.warn_pub(
                "Not quite ready to take in coordinates for survey zone, waiting on vehicle info..."
            )
            return
        self.survey_zone = []
        for glob in msg.coords:
            glob: GlobalCoordinates = glob
            n, e, d = self.get_ned(glob.latitude, glob.longitude,
                                   glob.altitude)
            self.survey_zone.append((n, e, d))

        ## Assumes rectangular survey zone (survey zone with 4 points)

        self.survey_path = []
        self.survey_path_curr_index = 0
        num_subs = 10
        a = np.array(self.survey_zone[0])
        b = np.array(self.survey_zone[1])
        c = np.array(self.survey_zone[3])

        ab = b - a
        ac = c - a
        ab, ac = ab, ac if np.linalg.norm(ab) > np.linalg.norm(ac) else ac, ab
        ac_step = ac / num_subs
        for i in range(int(num_subs // 2)):
            origin = a + 2 * i * ac_step
            self.survey_path.append(origin)
            self.survey_path.append(origin + ab)
            self.survey_path.append(origin + ab + ac_step)
            self.survey_path.append(origin + ac_step)

        self.get_logger().warn(f"Survey lap: {self.survey_path}")

    def mission_lap_callback(self, msg: ListGlobalCoordinates):
        if self.lat0 is None:
            info_string = "Not quite ready to take in coordinates for laps, waiting on vehicle info..."
            self.warn_pub(info_string)
            return
        self.lap = []
        self.curr_lap_point_index = 0
        for glob in msg.coords:
            glob: GlobalCoordinates = glob
            n, e, d = self.get_ned(glob.latitude, glob.longitude,
                                   glob.altitude)
            self.lap.append((n, e, d))

    def targets_callback(self, msg: ListTargets):
        self.targets = []
        self.top_target_published = False
        for target in msg.targets:
            self.targets.append(target)

    def target_location_callback(self, msg: Point32):
        self.target_location = msg

    def lap_tolerance_callback(self, msg: Float32):
        if (msg.data <= 0.):
            self.warn_pub(
                "Lap tolerance must be a positive floating point number")
            return
        self.curr_tolerance = msg.data

    def mission_time_callback(self, msg: Float32):
        if (msg.data <= 0.):
            self.warn_pub(
                "Mission time must be a positive floating point number, representing seconds of allowed mission time"
            )
            return
        self.mission_time = msg.data

    def start_callback(self, msg: Empty):
        if (self.start_time is None):
            self.start_time = self.get_clock().now().seconds_nanoseconds()
        if (self.state == self.WAITING):
            self.state = self.PERFORMIN_LAP

    def distance_from_curr_lap_dest(self):
        target_n, target_e, target_d = self.lap[self.curr_lap_point_index]
        return ((target_n - self.curr_n)**2 + (target_e - self.curr_e)**2 +
                (target_d - self.curr_d)**2)**(1 / 2)

    def get_max_speed_h(self):
        return 10.0

    def perform_lap(self):
        if (self.curr_n is None):
            self.warn_pub(
                "Not quite ready to perform lap yet, waiting on vehicle info..."
            )
            return

        if (self.lap is None):
            self.warn_pub(
                "Not quite ready to perform lap yet, lap wasn't set.")
            return

        if (self.distance_from_curr_lap_dest() <= self.curr_tolerance):
            if (self.curr_lap_point_index == len(self.lap) - 1):
                self.state = self.SURVEY_AREA
            self.curr_lap_point_index = (self.curr_lap_point_index + 1) % len(
                self.lap)
            self.lap_toggle = False
            return
        if (not self.lap_toggle):
            msg = Waypoint()
            (n, e, d) = self.lap[self.curr_lap_point_index]
            msg.x, msg.y, msg.z = n, e, d
            msg.max_speed_h = self.get_max_speed_h()
            self.goal_waypoint_pub.publish(msg)
            self.warn_pub(
                f"Trying to go to {self.lap[self.curr_lap_point_index]}, curr_index: {self.curr_lap_point_index}"
            )
            self.lap_toggle = True

    def distance_from(self, n, e, d):
        return ((n - self.curr_n)**2 + (e - self.curr_e)**2 +
                (d - self.curr_d)**2)**(1 / 2)

    def perform_survey(self):
        if (self.survey_zone is None):
            self.warn_pub(
                "Not quite ready to perform survey, survey zone wasn't set.")
            return
        if (self.target_location is not None):
            self.state = self.PERFORMING_AIRDROP
            self.survey_zone_toggle = False
            self.at_survey_zone = False
            self.survey_toggle = False
            return
        if (not self.top_target_published):
            self.curr_target_pub.publish(self.targets[0])
            self.top_target_published = True
        # TODO: SURVEY THE AREA
        survey_point = self.survey_zone[0]
        survey_origin = (*survey_point[:2], self.curr_d)

        # goto survey area
        if (not self.survey_zone_toggle):
            msg = Waypoint()
            msg.x, msg.y, msg.z = survey_origin
            msg.max_speed_h = self.get_max_speed_h()
            self.goal_waypoint_pub.publish(msg)
            self.survey_zone_toggle = True
        elif (self.distance_from(survey_origin[0], survey_origin[1],
                                 survey_origin[2]) < self.curr_tolerance):
            self.at_survey_zone = True
        if (self.at_survey_zone and self.survey_zone_toggle):
            tar_n, tar_e, tar_d = self.survey_path[self.survey_path_curr_index]
            # TODO: PATROL AT 1 m/s
            if (self.distance_from(tar_n, tar_e, self.curr_d) <= 0.5):
                self.survey_path_curr_index = (self.survey_path_curr_index +
                                               1) % len(self.survey_path)
                self.survey_toggle = False
                return
            if (not self.survey_toggle):
                msg = Waypoint()
                (n, e, d) = self.survey_path[self.survey_path_curr_index]
                msg.x, msg.y, msg.z = n, e, self.curr_d
                msg.max_speed_h = 2.0
                self.get_logger().warn(
                    f"Trying to go to: {msg.x}, {msg.y}, {msg.z}")
                self.goal_waypoint_pub.publish(msg)
                self.survey_toggle = True

    def perform_airdrop(self):
        if (self.distance_from(self.target_location.x, self.target_location.y,
                               self.curr_d) >= self.curr_tolerance):
            if (not self.airdrop_toggle):
                msg = Waypoint()
                (
                    n, e, d
                ) = self.target_location.x, self.target_location.y, self.curr_d
                msg.x, msg.y, msg.z = n, e, d
                msg.max_speed_h = self.get_max_speed_h()
                self.goal_waypoint_pub.publish(msg)
                self.airdrop_toggle = True
            return
        self.airdrop_toggle = False
        msg = Int32()
        msg.data = self.targets[0].target_hub
        self.airdrop_pub.publish(msg)

        dur = Duration(seconds=60)
        self.get_clock().sleep_for(dur)
        self.target_location = None
        self.top_target_published = False
        self.targets.pop(0)
        self.warn_pub(f"Performed airdrop! {len(self.targets)} targets left.")
        if (len(self.targets) > 0):
            self.state = self.PERFORMIN_LAP
        else:
            self.state = self.RTH

    def perform_rth(self):
        self.curr_lap_point_index = 0
        self.survey_path_curr_index = 0
        if (self.distance_from(0, 0, self.curr_d) >= 1.0):
            if (not self.rth_toggle):
                msg = Waypoint()
                (n, e, d) = 0.0, 0.0, self.curr_d
                msg.x, msg.y, msg.z = n, e, d
                msg.max_speed_h = self.get_max_speed_h()
                self.goal_waypoint_pub.publish(msg)
                self.rth_toggle = True
            return

        self.rth_toggle = False
        msg = Waypoint()
        (n, e, d) = 0.0, 0.0, 0.0
        msg.x, msg.y, msg.z = n, e, d
        msg.max_speed_h = self.get_max_speed_h()
        self.goal_waypoint_pub.publish(msg)

        self.state = self.WAITING

    def mission_tick(self):
        if (self.state == self.WAITING):
            return
        if (self.state == self.PERFORMIN_LAP):
            self.perform_lap()
        if (self.state == self.SURVEY_AREA):
            self.perform_survey()
        if (self.state == self.PERFORMING_AIRDROP):
            self.perform_airdrop()
        if (self.state == self.RTH):
            self.perform_rth()


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
