#!/usr/bin/env python

from typing import Deque
import collections
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import GotoSetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import BatteryStatus
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleOdometry
from custom_msgs.srv import RequestAction
from custom_msgs.srv import SendAction
from custom_msgs.msg import Action
from custom_msgs.msg import VehicleInfo
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.clock import Clock
from rclpy.node import Node
import rclpy

__autor__ = "Imad Issafras"
__contact__ = "imad.issafras@outlook.com"

import os


class OffboardControl(Node):

    class Action:

        def __init__(self, des: str, action_type: int, action: Action) -> None:
            self.description = des
            self.action_type = action_type
            self.action_obj: Action = action

        def perform(self, offboard_object: "OffboardControl"):
            pass

        @staticmethod
        def construct_action(action_type,
                             x=None,
                             y=None,
                             z=None,
                             yaw=None,
                             max_speed_h=None,
                             action=None) -> "OffboardControl.Action":
            if (action_type == Action.ACTION_TAKE_OFF):
                return OffboardControl.TakeOff(action)
            elif (action_type == Action.ACTION_WAYPOINT):
                return OffboardControl.Waypoint(x, y, z, yaw, max_speed_h, action)
            elif (action_type == Action.ACTION_LAND):
                return OffboardControl.Landing(action)
            else:
                print(f"[WARNING] {action_type} is not a valid action type id")

    class TakeOff(Action):

        def __init__(self, action: Action):
            super().__init__("Take off", Action.ACTION_TAKE_OFF, action)
            self.setup_done = False

        def perform(self, offboard_object: "OffboardControl"):
            offboard_object.takeoff(self)

    class Waypoint(Action):

        def __init__(self, x, y, z, yaw, max_speed_h, action: Action) -> None:
            super().__init__(f"Waypoint to ({x}, {y}, {z})",
                             Action.ACTION_WAYPOINT, action)
            self.x = x
            self.y = y
            self.z = z
            self.yaw = yaw
            self.max_speed_h = max_speed_h

        def perform(self, offboard_object: "OffboardControl"):
            offboard_object.waypoint(self.x, self.y, self.z, self.yaw, self.max_speed_h)

    class Landing(Action):

        def __init__(self, action: Action):
            super().__init__("Landing", Action.ACTION_LAND, action)

        def perform(self, offboard_object: "OffboardControl"):
            offboard_object.land()

    def __init__(self):
        super().__init__('px4_controller')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)

        # PX4 pubs / subs
        # PX4 SUBS
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile=qos_profile)
        self.vehicle_status: VehicleStatus | None = None

        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile=qos_profile)
        self.vehicle_local_position: VehicleLocalPosition | None = None

        self.vehicle_battery_status_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.vehicle_battery_status_callback,
            qos_profile=qos_profile)
        self.vehicle_battery_status: BatteryStatus | None = None

        self.vehicle_command_ack_sub = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self.vehicle_command_ack_callback, qos_profile)
        self.vehicle_command_ack: VehicleCommandAck | None = None

        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback, qos_profile)
        self.vehicle_attitude: VehicleAttitude | None = None

        self.vehicle_odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odom_callback, qos_profile 
        )
        self.vehicle_odom: VehicleOdometry | None = None

        # PX4 PUBS
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.goto_setpoint_pub = self.create_publisher(
            GotoSetpoint, '/fmu/in/goto_setpoint', qos_profile)

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # custom pubs / subs
        self.vehicle_info_pub = self.create_publisher(
            VehicleInfo, '/px4_monitoring/vehicle_info', qos_profile)

        self.enqueue_action_service = self.create_service(
            SendAction, '/px4_action_queue/append_action',
            self.enqueue_action_callback)

        self.pop_action_service = self.create_service(
            RequestAction, '/px4_action_queue/popleft_action',
            self.popleft_action_callback)

        # publishers timing
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.action_queue: Deque[OffboardControl.Action] = collections.deque(
            [])

        timer_period_actions = 0.2
        self.timer_action = self.create_timer(timer_period_actions,
                                              self.action_executor)

    def offboard_mode_publish(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.offboard_mode_pub.publish(offboard_msg)

    def print_vehicle_info(self, info: VehicleInfo):
        dash_length = os.get_terminal_size()[0]
        string = "\n" + ('-' * dash_length) + "\n"
        string += f"Time: {info.stamp}\n"
        string += f"(N, E, D): ({info.x:.4f}, {info.y:.4f}, {info.z:.4f})\n"
        string += f"(VN, VE, VD): ({info.vx:.4f}, {info.vy:.4f}, {info.vz:.4f})\n"
        string += f"(AN, AE, AD): ({info.ax:.4f}, {info.ay:.4f}, {info.az:.4f})\n"
        string += f"heading: {info.heading}\n"
        string += f"(ref_lat, ref_lon, ref_alt): ({info.ref_lat:.4f}, {info.ref_lon:.4f}, {info.ref_alt:.4f})\n"
        string += f"q: ({info.q[0]:.4f}, {info.q[1]:.4f}, {info.q[2]:.4f}, {info.q[3]:.4f})\n"
        string += f"angular_velocity: ({info.angular_velocity[0]:.4f}, {info.angular_velocity[1]:.4f}, {info.angular_velocity[2]:.4f})\n"
        string += f"power level: {info.powerlevel:.2f}%\t"
        mode_string = "Armed" if info.arming_state == VehicleInfo.ARMING_STATE_ARMED else "Disarmed"
        string += f"arming state: {info.arming_state}, {mode_string}\n"
        string += f"current action: {info.current_action}\n"
        string += '-' * dash_length
        self.get_logger().info(string)

    def vehicle_info_publish(self):
        vehicle_info = VehicleInfo()
        vehicle_info.stamp = self.get_clock().now().to_msg()

        # position and velocity
        if (self.vehicle_local_position is not None):
            vehicle_info.x = self.vehicle_local_position.x
            vehicle_info.y = self.vehicle_local_position.y
            vehicle_info.z = self.vehicle_local_position.z

            vehicle_info.vx = self.vehicle_local_position.vx
            vehicle_info.vy = self.vehicle_local_position.vy
            vehicle_info.vz = self.vehicle_local_position.vz

            vehicle_info.ax = self.vehicle_local_position.ax
            vehicle_info.ay = self.vehicle_local_position.ay
            vehicle_info.az = self.vehicle_local_position.az

            vehicle_info.ref_lat = self.vehicle_local_position.ref_lat
            vehicle_info.ref_lon = self.vehicle_local_position.ref_lon
            vehicle_info.ref_alt = self.vehicle_local_position.ref_alt

            vehicle_info.heading = self.vehicle_local_position.heading

        # Attitude
        if (self.vehicle_attitude is not None):
            vehicle_info.q = self.vehicle_attitude.q
        if (self.vehicle_odom is not None):
            vehicle_info.angular_velocity = self.vehicle_odom.angular_velocity
        # mode
        if (self.vehicle_status is not None):
            vehicle_info.arming_state = self.vehicle_status.arming_state
        # power
        if (self.vehicle_battery_status is not None):
            vehicle_info.powerlevel = self.vehicle_battery_status.remaining * 100
        # actions
        if (len(self.action_queue) == 0):
            vehicle_info.current_action = "No action to be done"
            vehicle_info.curr_action_obj = Action()
        else:
            vehicle_info.current_action = self.action_queue[0].description
            vehicle_info.curr_action_obj = self.action_queue[0].action_obj

        self.print_vehicle_info(vehicle_info)

        self.vehicle_info_pub.publish(vehicle_info)

    def timer_callback(self):
        self.offboard_mode_publish()
        self.vehicle_info_publish()

    def send_vehicle_command(self,
                             cmd,
                             param1=None,
                             param2=None,
                             param3=None,
                             param4=None,
                             param5=None,
                             param6=None,
                             param7=None):
        vehicle_command = VehicleCommand()
        vehicle_command.command = cmd  # command ID

        for i in range(1, 8):
            parameter_input = vars()[f"param{i}"]
            if parameter_input is not None:
                vehicle_command.__setattr__(f"param{i}", parameter_input)

        vehicle_command.target_system = 1  # system which should execute the command
        # component which should execute the command, 0 for all components
        vehicle_command.target_component = 1
        vehicle_command.source_system = 1  # system sending the command
        vehicle_command.source_component = 1  # component sending the command
        vehicle_command.from_external = True
        vehicle_command.timestamp = int(Clock().now().nanoseconds /
                                        1000)  # time in microseconds

        self.vehicle_command_pub.publish(vehicle_command)

    def action_executor(self):
        if (len(self.action_queue) > 0):
            self.action_queue[0].perform(self)

    def takeoff(self, action_ob: TakeOff):
        """
            Takeoff action, only completes when within 0.1 m of the target height
        """

        target_height = -5.
        trajectorySetpoint = GotoSetpoint()
        trajectorySetpoint.position = [0.0, 0.0, target_height]
        self.goto_setpoint_pub.publish(trajectorySetpoint)

        if (not action_ob.setup_done):
            # Setup
            # ARM
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                      1., 6.)
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=float(VehicleCommand.ARMING_ACTION_ARM))
            action_ob.setup_done = True

        # set wapoint to be up high

        trajectorySetpoint = GotoSetpoint()
        trajectorySetpoint.position = [0.0, 0.0, target_height]
        self.goto_setpoint_pub.publish(trajectorySetpoint)

        if (abs(self.vehicle_local_position.z - target_height) <= 0.3):
            self.action_queue.popleft()

    def current_tolerance(self):
        """
            defines the distance tolerances for Quad Copter configuartion
        """
        return 3.

    def waypoint(self, x, y, z, yaw, max_speed_h):
        """
            Waypoint action, only completes when within current_tolarance from the target waypoint
        """
        trajectorySetpoint = GotoSetpoint()
        trajectorySetpoint.position = [x, y, z]
        if (yaw is not None):
            trajectorySetpoint.flag_control_heading = True
            trajectorySetpoint.heading = yaw
            # trajectorySetpoint.flag_set_max_heading_rate =  True
            # trajectorySetpoint.max_heading_rate = 0.78539816339 / 8 # (pi / 4) rad / s 
        if (max_speed_h is not None):
            trajectorySetpoint.flag_set_max_horizontal_speed = True
            trajectorySetpoint.max_horizontal_speed = max_speed_h
        self.goto_setpoint_pub.publish(trajectorySetpoint)
        # if (((self.vehicle_local_position.x - x)**(2) +
        #      (self.vehicle_local_position.y - y)**(2) +
        #      (self.vehicle_local_position.z - z)**(2))**(1 / 2)
        #         <= self.current_tolerance()):
        #     self.action_queue.popleft()

    def land(self):
        """
            Landing Action, only completes when drone is disarmed
        """
        if (self.vehicle_command_ack is None
                or self.vehicle_command_ack.command
                != VehicleCommand.VEHICLE_CMD_NAV_LAND
                or self.vehicle_command_ack.result
                != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED):
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if (self.vehicle_status.arming_state ==
                VehicleStatus.ARMING_STATE_DISARMED):
            self.action_queue.popleft()

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    def vehicle_battery_status_callback(self, msg: BatteryStatus):
        self.vehicle_battery_status = msg

    def vehicle_command_ack_callback(self, msg: VehicleCommandAck):
        self.vehicle_command_ack = msg

    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        self.vehicle_attitude = msg

    def vehicle_odom_callback(self, msg: VehicleOdometry):
        self.vehicle_odom = msg

    def enqueue_action_callback(self, req, res):
        action = OffboardControl.Action.construct_action(
            req.action.action, req.action.x, req.action.y, req.action.z,
            req.action.yaw, req.action.max_speed_h, req.action)
        self.action_queue.append(action)
        res.success = True
        return res

    def popleft_action_callback(self, req, res):
        if (len(self.action_queue) == 0):
            res.action.action = Action.ACTION_NONE
            return res
        action: OffboardControl.Action = self.action_queue.popleft()
        res.action.action = action.action_type
        res.action.x = action.x
        res.action.y = action.y
        res.action.z = action.z
        res.action.yaw = action.yaw
        res.action.max_speed_h = action.max_speed_h 
        return res


def main(args=None):

    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
