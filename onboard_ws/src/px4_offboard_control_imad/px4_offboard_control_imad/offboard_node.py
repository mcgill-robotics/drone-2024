#!/usr/bin/env python


__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand
from std_srvs.srv import Empty
from custom_msgs.srv import SendFloat
from custom_msgs.srv import SendWaypoint

class OffboardControl(Node):

    def __init__(self):
        # SETUP
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

         # OFFBOARD HEALTH COMFIRMATION
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        timer_period = 0.1  # seconds
        self._off_board_setpoint_counter = 0
        self.timer = self.create_timer(timer_period, self.time_callback)

       # MONITORING
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile
        )
        self.printed = False
        

        # PX4 COMMAND
        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
            )
        self.publish_trajectory_setpoint = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )

        # SERVICES
        self.take_off_service = self.create_service(
            Empty, "/custom_arming", self.arm)
        self.arming_service = self.create_service(
            SendFloat, "/custom_take_off", self.take_off)
        self.waypoint_service = self.create_service(
            SendWaypoint, "/custom_waypoint_control", self.set_waypoint
        )
        self.land_service = self.create_service(
            Empty, "/custom_landing", self.land)
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.x = 0.0
        self.y = 0.0
        self.z = -5.0



    def time_callback(self):
        if (self._off_board_setpoint_counter == 10):
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.arm(None, None)
        
        self.send_offboard_signal()

        self.send_trajectory_setpoint()
        
        if (self._off_board_setpoint_counter < 11):
            self._off_board_setpoint_counter += 1
        

    def vehicle_status_callback(self, msg : VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    def vehicle_local_position_callback(self, msg : VehicleLocalPosition):
        time_bool = int(self.get_clock().now().nanoseconds // 1e9) % 5 == 0
        if (time_bool and not self.printed):
            self.printed = True
            self.get_logger().info(f"Vehicle Local Postion = {msg}")
        elif (not time_bool):
            self.printed = False

    def send_offboard_signal(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)

    def send_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.x, self.y, self.z]
        msg.yaw = -3.14
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publish_trajectory_setpoint.publish(msg)

    def send_vehicle_command(self, cmd, param1=None, param2=None, param3=None, 
                             param4=None, param5=None, param6=None, param7=None):
        vehicle_command = VehicleCommand()

        vehicle_command.command = cmd  # command ID

        for i in range(1, 8):
            parameter_input = vars()[f"param{i}"]
            if parameter_input is not None:
                vehicle_command.__setattr__(f"param{i}", parameter_input)

        vehicle_command.target_system = 1  # system which should execute the command
        vehicle_command.target_component = 1  # component which should execute the command, 0 for all components
        vehicle_command.source_system = 1  # system sending the command
        vehicle_command.source_component = 1  # component sending the command
        vehicle_command.from_external = True
        vehicle_command.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds

        self.publisher_vehicle_command.publish(vehicle_command)

    # Empty srv type
    def arm(self, request, response):
        # arm
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.)
        return response
    
        # Empty srv type
    def disarm(self, request, response):
        # disarm
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.)
        self.get_logger().warn("Drone is armed")
        return response
    
    def set_waypoint(self, request, response):
        self.x = request.x
        self.y = request.y
        self.z = request.z
        return response

   
    # SendFloat srv type
    def take_off(self, request, response):
        # arm
        msg = TrajectorySetpoint = TrajectorySetpoint()
        msg.position = {0.0, 0.0, -5.}
        msg.yaw = -3.14; #[-PI:PI]
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.get_logger().warn(f"Setting takeoff altitude to {request.num}")

        response.success = True
        return response
    
    def land(self, request, response):
        vehicle_command = VehicleCommand()
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.publisher_vehicle_command.publish(vehicle_command)
        return response

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
