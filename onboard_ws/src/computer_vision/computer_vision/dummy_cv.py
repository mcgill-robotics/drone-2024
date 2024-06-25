import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from custom_msgs.msg import Target
from custom_msgs.msg import VehicleInfo
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

import cv2
import numpy as np
from ultralytics import YOLO
from .utils.cv_util import read_color_on_shape
from .utils.loc_util import get_n_e_d_from_pixel
import os


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('dummy_computer_vision')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)

        self.curr_target_sub = self.create_subscription(
            Target, "/current_target", self.current_target_callback,
            self.qos_profile)
        self.curr_target: Target | None = None

        self.vehicle_info_sub = self.create_subscription(
            VehicleInfo, "/px4_monitoring/vehicle_info",
            self.vehicle_info_callback, self.qos_profile)
        self.vehicle_info: VehicleInfo | None = None

        self.target_position_publisher = self.create_publisher(
            Point32, "/target_location", self.qos_profile)

        timer_period = 0.5
        self.timer_1 = self.create_timer(timer_period, self.cv_tick)

        path = os.path.realpath(__file__)
        path = os.path.dirname(path)
        pman = 'runs/detect/train5/weights/best.pt'
        plet = 'runs/detect/train10/weights/best.pt'
        path_man_shape = os.path.join(path, pman)
        path_letter = os.path.join(path, plet)

        self.man_shape_model = YOLO(path_man_shape)
        self.letter_model = YOLO(path_letter)

        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1 ! nvvidconv ! appsink",
            cv2.CAP_GSTREAMER)

        if (self.cap.isOpened()):
            print("The camera is successfully opened")
        else:
            print("Could not open the camera")
            exit()
        sensor_diagonal = 7.9 * 1e-3  # in m
        self.sensor_side = sensor_diagonal / (2**(1 / 2))
        self.focal_length = 16 * 1e-3  # in m
        self.nx = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.ny = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        frameRate = int(self.cap.get(cv2.CAP_PROP_FPS))
        print(f"w, h, fr: {self.nx}, {self.ny}, {frameRate}")

        # Camera feed
        self.image_publisher = self.create_publisher(Image, "/video_feed", 10)
        self.bridge = CvBridge()
        timer_period2 = 0.1
        self.timer = self.create_timer(timer_period2, self.publish_image)

    def publish_image(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if (not ret):
                print("Unable to get camera frame")
                return

            # # processes image data and converts to ros 2 message
            # msg = Image()
            # msg.header.stamp = Node.get_clock(self).now().to_msg()
            # msg.header.frame_id = 'ANI717'
            # msg.height = self.ny
            # msg.width = self.nx
            # msg.encoding = "bgr8"
            # msg.is_bigendian = False
            # msg.step = np.shape(frame)[2] * self.nx
            # msg.data = np.array(frame).tobytes()

            # publishes message
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding="passthrough"))

    def find_target_position_in_curr_frame(self):
        x, y, z = self.vehicle_info.x, self.vehicle_info.y, self.vehicle_info.z
        heading = self.vehicle_info.heading

        target_type = self.curr_target.target_type
        ret, frame = self.cap.read()
        if not ret:
            print("Can't receive frame, skipping")
            return None
        man_shape_detections = self.man_shape_model(frame)
        # Nothing in this frame !
        if len(man_shape_detections[0].boxes.data.tolist()) == 0:
            return None

        for detection in man_shape_detections[0].boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = detection
            x_center, y_center = (x1 + x2) / 2, (y1 + y2) / 2
            detected_target_type = Target.TARGET_TYPE_EMERGENT if class_id == 3 else Target.TARGET_TYPE_STANDARD
            # look if any detections have our target
            if (target_type != detected_target_type):
                continue
            else:
                if detected_target_type == Target.TARGET_TYPE_EMERGENT:
                    # TODO: MAKE SURE X AND Y ARE LOGICAL AND NOT REVERSED (OpenCV has them reversed in storage)
                    msg = Point32()
                    msg.x, msg.y, msg.z = get_n_e_d_from_pixel(
                        x, y, z, heading, x_center, y_center, self.nx, self.ny,
                        self.sensor_side, self.focal_length)
                    return msg
                else:
                    detected_shape = Target.SHAPE_CIRCLE if class_id == 0 else \
                    Target.SHAPE_CROSS if class_id == 1 else \
                    Target.SHAPE_PENTAGON if class_id == 2 else \
                    Target.SHAPE_QUARTER_CIRCLE if class_id == 4 else \
                    Target.SHAPE_RECTANGLE if class_id == 5 else \
                    Target.SHAPE_SEMI_CIRCLE if class_id == 6 else \
                    Target.SHAPE_STAR if class_id == 7 else \
                    Target.SHAPE_TRIANGLE
                    target_shape = self.curr_target.shape  ## The shape variable is actually a number that represents the shape
                    if (detected_shape != target_shape):
                        continue
                    # TODO: GET TEXT AND COLOR FROM THE REST OF THE CODE

                    target_shape_color = self.curr_target.shape_color  # Same as shape, different values for the different colors

                    # Target shape colors values
                    # uint32 COLOR_WHITE = 1
                    # uint32 COLOR_BLACK = 2
                    # uint32 COLOR_RED = 3
                    # uint32 COLOR_BLUE = 4
                    # uint32 COLOR_GREEN = 5
                    # uint32 COLOR_PURPLE = 6
                    # uint32 COLOR_BROWN = 7
                    # uint32 COLOR_ORANGE = 8
                    target_letter = self.curr_target.alpha_num  # This is either an ASCII integer representing
                    # the value or the char, am not sure and am too lazy to test sorry emma
                    target_letter_color = self.curr_target.alpha_num_color  # Same numbers as shape color

                    # TODO: The rest of the code

        # Nothing found, carry on
        return None

    def cv_tick(self):
        # Only publish a given position once, hence the 2 if statements and setting curr_target back to None
        if (self.curr_target is None):
            ## no current target, mission is doing something else (lap or something)
            return

        ## So here target is not None, aka a valid target
        target_position: Point32 | None = self.find_target_position_in_curr_frame(
        )
        if (target_position is not None):
            self.curr_target = None
            self.target_position_publisher.publish(target_position)

    def current_target_callback(self, msg: Target):
        self.curr_target = msg

    def vehicle_info_callback(self, msg: VehicleInfo):
        self.vehicle_info = msg


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
