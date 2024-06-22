import rclpy
from rclpy.node import Node
from custom_msgs.msg import Target
from custom_msgs.msg import VehicleInfo
from geometry_msgs.msg import Point32

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

import cv2
import numpy as np
from ultralytics import YOLO
from util import get_shape, read_letter_and_color_on_shape
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

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print("Can't open camera")
            exit()

        width = int(640 * 3.6)
        height = int(480 * 3.6)
        fr = 5
        ok = self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if (not ok):
            print("Could not change image width")
            exit()
        ok = self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if (not ok):
            print("Could not change image height")
            exit()
        #
        ok = self.cap.set(cv2.CAP_PROP_FPS, fr)
        if (not ok):
            print("Could not change camera frame rate")
            exit()

        frameWidth = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frameHeight = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        frameRate = int(self.cap.get(cv2.CAP_PROP_FPS))
        print(f"w, h, fr: {frameWidth}, {frameHeight}, {frameRate}")

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

    def find_target_position_in_curr_frame(self):
        # TODO: CV CODE, LOOK FOR self.curr_target in the next few frames (or just next frame)
        # If not in the frame, return None, if is in the frame(s), return its position
        x, y, z = self.vehicle_info.x, self.vehicle_info.y, self.vehicle_info.z

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
            detected_target_type = Target.TARGET_TYPE_EMERGENT if class_id == 3 else Target.TARGET_TYPE_STANDARD
            # look if any detections have our target
            if (target_type != detected_target_type):
                continue
            else:
                if detected_target_type == Target.TARGET_TYPE_EMERGENT:
                    # PERSON WE ARE LOOKING FOR !!!!!
                    msg = Point32()
                    msg.x, msg.y, msg.z = get_n_e_d_from_pix()
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
        self.q = msg.q


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
