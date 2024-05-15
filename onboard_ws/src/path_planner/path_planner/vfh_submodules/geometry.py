from custom_msgs.msg import Waypoint
from geometry_msgs.msg import Polygon
from collections import namedtuple
import numpy as np


class LocalCoordinates:

    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0, max_speed_h=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.max_speed_h = max_speed_h

    def set_xyz(self, msg: Waypoint):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def __eq__(self, o):
        if (o is None):
            return False
        return self.x == o.x and self.y == o.y and self.z == o.z

    def __str__(self):
        return f"(x, y, z, yaw): ({self.x:.4f}, {self.y:.4f}, {self.z:.4f}, {self.yaw:.4f})\n"

    def distance_to_xyz(self, x, y, z):
        return ((self.x - x)**2 + (self.y - y)**2 + (self.z - z)**2)**(1 / 2)


class Obstacle:

    def contains(self, x, y):
        print("AHHH LEAVE ME ALONE AM AN ABSTRACT CLASS !!!")
        return False


class Monolithe(Obstacle):

    def __init__(self, center_x, center_y, width, depth) -> None:
        super().__init__()
        self.center_x, self.center_y = (center_x, center_y)
        self.width, self.depth = (width, depth)

    def contains(self, x, y):
        return (self.center_x - self.width / 2) <= x <= (self.center_x + self.width / 2) \
            and (self.center_y - self.depth / 2) <= y <= (self.center_y + self.depth / 2)


class Boundary(Obstacle):

    class Segment:

        def __init__(self, start_point, end_point):
            self.origin = np.array([start_point.x, start_point.y])
            self.direction = np.array(
                [end_point.x - start_point.x, end_point.y - start_point.y])

        def find_intersection(self, other_seg: 'Boundary.Segment'):
            p = self.origin
            r = self.direction
            q = other_seg.origin
            s = other_seg.direction

            denom = np.cross(r, s)
            num_this = np.cross((q - p), s)
            num_other = np.cross((q - p), r)
            if abs(denom) <= 1e-10:
                return -1, -1
            t = num_this / denom
            u = num_other / denom
            return t, u

        def __str__(self):
            return f"{self.origin} + t * {self.direction}"

    def __init__(self, list_points: Polygon):
        self.segments = []
        for i in range(len(list_points)):
            seg = None
            if i == (len(list_points) - 1):
                seg = self.Segment(list_points[i], list_points[0])
            else:
                seg = self.Segment(list_points[i], list_points[i + 1])
            self.segments.append(seg)

    def contains(self, x, y):
        point = namedtuple('point', ['x', 'y'])
        seg = self.Segment(point(x, y), point(x + 1, y))
        intersection_count = 0
        for segment in self.segments:
            t, u = seg.find_intersection(segment)
            if (t >= 0 and 0 <= u <= 1):
                intersection_count += 1
        return intersection_count % 2 == 0
