import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from custom_msgs.msg import GlobalCoordinates
from custom_msgs.msg import ListGlobalCoordinates
from custom_msgs.msg import Target
from custom_msgs.msg import ListTargets
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import json
import os
from pathlib import Path


class MinimalPublisher(Node):

    def make_waypoint_dict(self, latitude, longitude, altitude):
        return {
            "AMSLAltAboveTerrain": None,
            "Altitude": 50,
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 16,
            "doJumpId": 14,
            "frame": 3,
            "params": [0, 0, 0, None, latitude, longitude, altitude],
            "type": "SimpleItem"
        }

    def __init__(self):
        super().__init__('mission_detail_publisher')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)
        path = os.path.realpath(__file__)
        path = os.path.dirname(path)
        path = os.path.join(path, "../resource/big_field_mission.json")
        with open(path, "r") as fp:
            self.json = json.load(fp)
        self.boundary_publisher = self.create_publisher(
            ListGlobalCoordinates, '/mission_boundary', self.qos_profile)
        self.lap_publisher = self.create_publisher(ListGlobalCoordinates,
                                                   '/mission_lap',
                                                   self.qos_profile)
        self.survey_zone_publisher = self.create_publisher(
            ListGlobalCoordinates, '/survey_zone', self.qos_profile)

        self.targets_publisher = self.create_publisher(ListTargets,
                                                       '/mission_targets',
                                                       self.qos_profile)
        survey_polygon = []
        geofence_polygon = []
        survey_object = {
            "TransectStyleComplexItem": {
                "CameraCalc": {
                    "AdjustedFootprintFrontal": 25,
                    "AdjustedFootprintSide": 25,
                    "CameraName": "Manual (no camera specs)",
                    "DistanceMode": 1,
                    "DistanceToSurface": 50,
                    "version": 2
                },
                "CameraShots": 2,
                "CameraTriggerInTurnAround": True,
                "HoverAndCapture": False,
                "Items": [],
                "Refly90Degrees": False,
                "TurnAroundDistance": 10,
                "VisualTransectPoints": survey_polygon,
                "version": 2
            },
            "angle": 0,
            "complexItemType": "survey",
            "entryLocation": 0,
            "flyAlternateTransects": False,
            "polygon": survey_polygon,
            "type": "ComplexItem",
            "version": 4
        }

        objects = []

        self.plan_dict = {
            "fileType": "Plan",
            "geoFence": {
                "circles": [],
                "polygons": [{
                    "inclusion": True,
                    "polygon": geofence_polygon,
                    "version": 1
                }],
                "version":
                2
            },
            "groundStation": "QGroundControl",
            "mission": {
                "cruiseSpeed": 15,
                "firmwareType": 12,
                "globalPlanAltitudeMode": 1,
                "hoverSpeed": 5,
                "items": objects,
                "plannedHomePosition": [47.3977419, 8.545594, 487.989],
                "vehicleType": 2,
                "version": 2
            },
            "rallyPoints": {
                "points": [],
                "version": 2
            },
            "version": 1
        }

        # Boundary
        msg_list = ListGlobalCoordinates()
        msg_list.coords = []
        for entry in self.json["boundary"]:
            msg = GlobalCoordinates()
            msg.latitude = entry["latitude"]
            msg.longitude = entry["longitude"]
            msg_list.coords.append(msg)
            geofence_polygon.append([entry["latitude"], entry["longitude"]])
        self.boundary_publisher.publish(msg_list)

        # lap
        msg_list = ListGlobalCoordinates()
        msg_list.coords = []
        for entry in self.json["lap"]:
            msg = GlobalCoordinates()
            msg.latitude = entry["latitude"]
            msg.longitude = entry["longitude"]
            msg.altitude = entry["altitude"]
            objects.append(
                self.make_waypoint_dict(entry["latitude"], entry["longitude"],
                                        entry["altitude"]))
            msg_list.coords.append(msg)
        self.lap_publisher.publish(msg_list)

        # survey zone
        msg_list = ListGlobalCoordinates()
        msg_list.coords = []
        for entry in self.json["survey_area"]:
            msg = GlobalCoordinates()
            msg.latitude = entry["latitude"]
            msg.longitude = entry["longitude"]
            msg_list.coords.append(msg)
            survey_polygon.append([entry["latitude"], entry["longitude"]])
        self.survey_zone_publisher.publish(msg_list)

        # targets
        msg_list = ListTargets()
        msg_list.targets = []
        for entry in self.json["targets"]:
            msg = Target()
            msg.target_type = entry["target_type"]
            msg.target_hub = entry["target_hub"]
            msg.shape = entry["shape"]
            msg.shape_color = entry["shape_color"]
            msg.alpha_num = ord(entry["alpha_num"][0])
            msg.alpha_num_color = entry["alpha_num_color"]
            msg_list.targets.append(msg)
        self.targets_publisher.publish(msg_list)

        objects.append(survey_object)

        # Remember to publish to /start topic to start the mission
        with open(Path.home() / "generated_plan.plan", "w") as fp:
            json.dump(self.plan_dict, fp)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically files
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
