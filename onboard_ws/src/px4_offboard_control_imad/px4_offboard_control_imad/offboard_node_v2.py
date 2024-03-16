#!/usr/bin/env python

__autor__="Imad Issafras"
__contact__="imad.issafras@outlook.com"

# ROS imports
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# PX4 messages
## In
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import BatteryStatus

## Out
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand

# Other types
from custom_msgs.msg import VehicleInfo

# Other imports 
import collections

class OffboardControl(Node):
    def __init__(self):
        super().__init__('px4_controller')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # PX4 pubs / subs
        ## PX4 SUBS
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile=qos_profile
            )
        self.vehicle_status : VehicleStatus | None  = None
        
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile=qos_profile
        )
        self.vehicle_local_position : VehicleLocalPosition | None = None

        self.vehicle_battery_status_sub = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status',
            self.vehicle_battery_status_callback,
            qos_profile=qos_profile
        )
        self.vehicle_battery_status : BatteryStatus | None = None

        self.vehicle_command_ack_sub = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self.vehicle_command_ack_callback,
            qos_profile=qos_profile
        )
        self.vehicle_command_ack : VehicleCommandAck | None = None 

        ## PX4 PUBS
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        # custom pubs / subs
        self.vehicle_info_pub = self.create_publisher(
            VehicleInfo, '/vehicle_info', qos_profile
        )

        # publishers timing
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.action_queue = collections.deque([])

    def offboard_mode_publish(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.offboard_mode_pub.publish(offboard_msg)
    
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
        # mode 
        if (self.vehicle_status is not None):
            vehicle_info = self.vehicle_status.vehicle_type
        # power
        if (self.vehicle_battery_status is not None):
            vehicle_info.powerlevel = 1.0 - self.vehicle_battery_status.remaining
        # actions
        if (len(self.action_queue) == 0):
            vehicle_info.current_action = "No action to be done"
        else:
            vehicle_info.current_action = self.action_queue[0].description

        self._logger.info(vehicle_info)
        self.vehicle_info_pub.publish()

    def timer_callback(self):
        self.offboard_mode_publish()
        self.vehicle_info_publish()
        

    def vehicle_status_callback(self, msg : VehicleStatus):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg : VehicleLocalPosition):
        self.vehicle_local_position = msg

    def vehicle_battery_status_callback(self, msg : BatteryStatus):
        self.vehicle_battery_status = msg

    def vehicle_command_ack_callback(self, msg : VehicleCommandAck):
        self.vehicle_command_ack = msg

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
