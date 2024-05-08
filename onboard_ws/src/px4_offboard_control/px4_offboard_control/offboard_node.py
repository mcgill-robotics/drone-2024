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
                             action=None) -> "OffboardControl.Action":
            if (action_type == Action.ACTION_TAKE_OFF):
                return OffboardControl.TakeOff(action)
            elif (action_type == Action.ACTION_WAYPOINT):
                return OffboardControl.Waypoint(x, y, z, action)
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

        def __init__(self, x, y, z, action: Action) -> None:
            super().__init__(f"Waypoint to ({x}, {y}, {z})",
                             Action.ACTION_WAYPOINT, action)
            self.x = x
            self.y = y
            self.z = z

        def perform(self, offboard_object: "OffboardControl"):
            offboard_object.waypoint(self.x, self.y, self.z)

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
            RequestAction, '/px4_action_queue/popright_action',
            self.popright_action_callback)

        # publishers timing
        timer_period = 0.1
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
        string += f"(N, E, D): ({info.x:.4f}, {info.y:.4f}, {info.z:.4f})\n"
        string += f"(VN, VE, VD): ({info.vx:.4f}, {info.vy:.4f}, {info.vz:.4f})\n"
        string += f"(ref_lat, ref_lon, ref_alt): ({info.ref_lat:.4f}, {info.ref_lon:.4f}, {info.ref_alt:.4f})\n"
        string += f"q: ({info.q[0]:.4f}, {info.q[1]:.4f}, {info.q[2]:.4f}, {info.q[3]:.4f})\n"
        string += f"power level: {info.powerlevel:.2f}%\t"
        mode_string = "Armed" if info.arming_state == VehicleInfo.ARMING_STATE_ARMED else "Disarmed"
        string += f"arming state: {info.arming_state}, {mode_string}\n"
        string += f"current action: {info.current_action}\n"
        string += '-' * dash_length
        self.get_logger().info(string)

    def vehicle_info_publish(self):
        vehicle_info = VehicleInfo()

        # position and velocity
        if (self.vehicle_local_position is not None):
            vehicle_info.x = self.vehicle_local_position.x
            vehicle_info.y = self.vehicle_local_position.y
            vehicle_info.z = self.vehicle_local_position.z

            vehicle_info.vx = self.vehicle_local_position.vx
            vehicle_info.vy = self.vehicle_local_position.vy
            vehicle_info.vz = self.vehicle_local_position.vz

            vehicle_info.ref_lat = self.vehicle_local_position.ref_lat
            vehicle_info.ref_lon = self.vehicle_local_position.ref_lon
            vehicle_info.ref_alt = self.vehicle_local_position.ref_alt

        # Attitude
        if (self.vehicle_attitude is not None):
            vehicle_info.q = self.vehicle_attitude.q
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

        if (abs(self.vehicle_local_position.z - target_height) <= 0.1):
            self.action_queue.popleft()

    def current_tolerance(self):
        """
            defines the distance tolerances for Quad Copter configuartion
        """
        return 3.

    def waypoint(self, x, y, z, yaw=0.0):
        """
            Waypoint action, only completes when within current_tolarance from the target waypoint
        """
        trajectorySetpoint = GotoSetpoint()
        trajectorySetpoint.position = [x, y, z]
        self.goto_setpoint_pub.publish(trajectorySetpoint)
        if (((self.vehicle_local_position.x - x)**(2) +
             (self.vehicle_local_position.y - y)**(2) +
             (self.vehicle_local_position.z - z)**(2))**(1 / 2)
                <= self.current_tolerance()):
            self.action_queue.popleft()

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

    def enqueue_action_callback(self, req, res):
        action = OffboardControl.Action.construct_action(
            req.action.action, req.action.x, req.action.y, req.action.z,
            req.action)
        self.action_queue.append(action)
        res.success = True
        return res

    def popright_action_callback(self, req, res):
        if (len(self.action_queue) == 0):
            res.action.action = Action.ACTION_NONE
            return res
        action: OffboardControl.Action = self.action_queue.pop()
        res.action.action = action.action_type
        res.action.x = action.x
        res.action.y = action.y
        res.action.z = action.z
        return res


def main(args=None):

    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
