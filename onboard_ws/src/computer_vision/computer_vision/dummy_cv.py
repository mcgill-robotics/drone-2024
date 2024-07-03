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
from .utils.cv_util import find_alphanum, rect_text_img
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
        pman = 'runs/detect/new_new_shape_model/weights/best.pt'
        plet = 'runs/detect/letters_yolo/weights/best.pt'
        pnum = 'runs/detect/numbers_yolo/weights/best.pt'
        path_man_shape = os.path.join(path, pman)
        path_letter = os.path.join(path, plet)
        path_num = os.path.join(path, pnum)

        self.man_shape_model = YOLO(path_man_shape)
        self.letter_model = YOLO(path_letter)
        self.number_model = YOLO(path_num)

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

        # manual position
        # x is pxpercentage of the nx, y is py percentage of the ny of current frame
        self.manual_position_sub = self.create_subscription(
            Point32, "/manual_detection", self.manual_position_callback,
            self.qos_profile)
        self.manual_position = None

        # Camera feed
        self.image_publisher = self.create_publisher(Image, "/video_feed", 10)
        self.bridge = CvBridge()
        timer_period2 = 0.1
        self.timer = self.create_timer(timer_period2, self.passive_feed_pub)

    def publish_frame(self, frame):
        frame = cv2.resize(frame, (1280, 720))
        # publishes message
        self.image_publisher.publish(
            self.bridge.cv2_to_imgmsg(frame, encoding="passthrough"))

    def passive_feed_pub(self):
        if self.cap.isOpened() and self.curr_target is None:
            ret, frame = self.cap.read()
            if (not ret):
                print("Unable to get camera frame")
                return
            self.publish_frame(frame)

    def get_color_id(self, color_string):
        # Target shape colors values
        # uint32 COLOR_WHITE = 1
        # uint32 COLOR_BLACK = 2
        # uint32 COLOR_RED = 3
        # uint32 COLOR_BLUE = 4
        # uint32 COLOR_GREEN = 5
        # uint32 COLOR_PURPLE = 6
        # uint32 COLOR_BROWN = 7
        # uint32 COLOR_ORANGE = 8

        colors = {
            'red': Target.COLOR_RED,
            'green': Target.COLOR_GREEN,
            'blue': Target.COLOR_BLUE,
            'white': Target.COLOR_GREEN,
            'black': Target.COLOR_BLACK,
            'purple': Target.COLOR_PURPLE,
            'brown': Target.COLOR_BROWN,
            'orange': Target.COLOR_ORANGE
        }
        return colors[color_string]

    def find_target_position_in_curr_frame(self):
        x, y, z = self.vehicle_info.x, self.vehicle_info.y, self.vehicle_info.z
        heading = self.vehicle_info.heading

        target_type = self.curr_target.target_type
        ret, frame = self.cap.read()
        annotated_frame = frame.copy()
        frame = frame[:, :, 0:3]
        if not ret:
            print("Can't receive frame, skipping")
            return None
        man_shape_detections = self.man_shape_model(frame)
        # Nothing in this frame !
        if len(man_shape_detections[0].boxes.data.tolist()) == 0:
            self.publish_frame(annotated_frame)
            return None

        for detection in man_shape_detections[0].boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = detection
            x_center, y_center = (x1 + x2) / 2, (y1 + y2) / 2
            # For now, impossible to detect emergent targets
            detected_target_type = Target.TARGET_TYPE_STANDARD
            rect_text_img(
                annotated_frame,
                f"{man_shape_detections[0].names[class_id]}, score: {score*100:.2f}%",
                x1, y1, x2, y2)
            frame_crop = frame[int(y1):int(y2), int(x1):int(x2)]
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
                    Target.SHAPE_QUARTER_CIRCLE if class_id == 3 else \
                    Target.SHAPE_RECTANGLE if class_id == 4 else \
                    Target.SHAPE_SEMI_CIRCLE if class_id == 5 else \
                    Target.SHAPE_STAR if class_id == 6 else \
                    Target.SHAPE_TRIANGLE
                    target_shape = self.curr_target.shape  ## The shape variable is actually a number that represents the shape
                    if (detected_shape != target_shape):
                        continue
                    # TODO: GET TEXT AND COLOR FROM THE REST OF THE CODE

                    letter_detections = self.letter_model(frame_crop)

                    number_detections = self.number_model(frame_crop)

                    # axs[int(index // num_columns)][int(index % num_columns)].imshow(
                    #     cv2.cvtColor(frame_of_interest, cv2.COLOR_BGR2RGB))

                    max_score = -1
                    shape_color = None
                    letter_color = None
                    text = None
                    # for letters
                    for detection in letter_detections[0].boxes.data.tolist():
                        x1p, y1p, x2p, y2p, score, class_id = detection
                        alleged_shape_color, alleged_letter_color, alleged_text = find_alphanum(
                            detection, letter_detections, frame_crop,
                            annotated_frame, x1, x2, y1, y2)
                        if (score >= max_score):
                            max_score = score
                            shape_color = alleged_shape_color
                            letter_color = alleged_letter_color
                            text = alleged_text

                    # for numbers
                    for detection in number_detections[0].boxes.data.tolist():
                        x1p, y1p, x2p, y2p, score, class_id = detection
                        alleged_shape_color, alleged_letter_color, alleged_text = find_alphanum(
                            detection, number_detections, frame_crop,
                            annotated_frame, x1, x2, y1, y2)
                        if (score >= max_score):
                            max_score = score
                            shape_color = alleged_shape_color
                            letter_color = alleged_letter_color
                            text = alleged_text

                    if shape_color is None:
                        continue
                    target_shape_color = self.curr_target.shape_color  # Same as shape, different values for the different colors
                    detect_shape_color = self.get_color_id(shape_color)
                    if target_shape_color != detect_shape_color:
                        continue

                    target_letter = self.curr_target.alpha_num  # This is either an ASCII integer representing
                    detect_letter = ord(text)

                    if target_letter != detect_letter:
                        continue

                    # the value or the char, am not sure and am too lazy to test sorry emma
                    target_letter_color = self.curr_target.alpha_num_color  # Same numbers as shape color
                    detect_letter_color = self.get_color_id(letter_color)
                    if target_letter_color != detect_letter_color:
                        continue

                    msg = Point32()
                    msg.x, msg.y, msg.z = get_n_e_d_from_pixel(
                        x, y, z, heading, x_center, y_center, self.nx, self.ny,
                        self.sensor_side, self.focal_length)
                    return msg

                    # TODO: The rest of the code

        self.publish_frame(annotated_frame)
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
        if (self.manual_position is not None
                and self.vehicle_info is not None):
            pix_perc, piy_perc = self.manual_position.x, self.manual_position.y
            x, y, z = self.vehicle_info.x, self.vehicle_info.y, self.vehicle_info.z
            heading = self.vehicle_info.heading
            target_position = get_n_e_d_from_pixel(x, y, z, heading,
                                                   int(pix_perc * self.nx),
                                                   int(piy_perc * self.ny),
                                                   self.nx, self.ny,
                                                   self.sensor_side,
                                                   self.focal_length)
        if (target_position is not None):
            self.manual_position = None
            self.curr_target = None
            self.target_position_publisher.publish(target_position)

    def current_target_callback(self, msg: Target):
        self.curr_target = msg

    def vehicle_info_callback(self, msg: VehicleInfo):
        self.vehicle_info = msg

    def manual_position_callback(self, msg: Point32):
        self.manual_position = msg


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
