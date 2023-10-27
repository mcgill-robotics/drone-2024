#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import uuid
import rospy
import string
from stl import Mesh
from geometry_msgs.msg import Point
from interop.msg import Color, Shape
from random import choice, randint, random
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from drone_gazebo.srv import SpawnObject, SpawnObjectResponse
from odlc.model_generator import (cross, rectangle, regular_polygon, star,
                                  trapezoid)

# Global model count.
model_count = 0

# Available alphanumeric characters.
available_alphanumeric = list(string.ascii_uppercase + string.digits)

# Available RGBA colors [0, 1].
# yapf: disable
rgba_colors = {
    Color.WHITE: (1.0, 1.0, 1.0, 1.0),
    Color.BLACK: (0.0, 0.0, 0.0, 1.0),
    Color.GRAY: (0.5, 0.5, 0.5, 1.0),
    Color.RED: (1.0, 0.0, 0.0, 1.0),
    Color.BLUE: (0.0, 0.0, 1.0, 1.0),
    Color.GREEN: (0.0, 1.0, 0.0, 1.0),
    Color.YELLOW: (1.0, 1.0, 0.0, 1.0),
    Color.PURPLE: (0.5, 0.0, 0.5, 1.0),
    Color.BROWN: (0.55, 0.27, 0.07, 1.0),
    Color.ORANGE: (1.0, 0.65, 0.0, 1.0),
}

available_colors = list(rgba_colors)

mesh_generators = {
    Shape.CIRCLE:
        lambda: regular_polygon(100),
    Shape.SEMICIRCLE:
        lambda: regular_polygon(100, sector_ratio=0.5),
    Shape.QUARTER_CIRCLE:
        lambda: regular_polygon(100, sector_ratio=0.25),
    Shape.TRIANGLE:
        lambda: regular_polygon(3),
    Shape.RECTANGLE:
        lambda: rectangle(randint(1, 10), randint(1, 10)),
    Shape.SQUARE:
        lambda: rectangle(1, 1),
    Shape.TRAPEZOID:
        lambda: trapezoid(randint(1, 10), randint(1, 10), random()),
    Shape.PENTAGON:
        lambda: regular_polygon(5),
    Shape.HEXAGON:
        lambda: regular_polygon(6),
    Shape.HEPTAGON:
        lambda: regular_polygon(7),
    Shape.OCTAGON:
        lambda: regular_polygon(8),
    Shape.STAR:
        lambda: star(5),
    Shape.CROSS:
        lambda: cross(randint(1, 10), randint(1, 10), random() * 3),
}
available_shapes = list(mesh_generators)
# yapf: enable


def generate_shape(seed_shape=None):
    """Generates and saves a random STL shape.

    Args:
        seed_shape: Shape seed instead of choosing one at random.

    Returns:
        Tuple of (shape, path to STL).
    """
    name = str(uuid.uuid1())
    path = os.path.join("/tmp", name) + ".stl"
    shape = choice(available_shapes) if seed_shape is None else seed_shape
    mesh = mesh_generators[shape]()
    mesh.save(path)
    return shape, path


def generate_alphanumeric(seed_alphanumeric=None):
    """Randomly chooses a letter or number to spawn.

    Args:
        seed_alphanumeric: Alphanumeric seed instead of choosing one at random.

    Returns:
        Tuple of (alphanumeric, path to STL).
    """
    if seed_alphanumeric is None:
        alphanumeric = choice(available_alphanumeric)
    else:
        alphanumeric = seed_alphanumeric

    path = os.path.join(
        os.environ.get("ROBOTIC_PATH"), os.environ.get("ROBOT"), "catkin_ws",
        "src", "drone_gazebo", "models", "alphanumeric",
        "{}.stl".format(alphanumeric))
    return alphanumeric, path


def generate_model_name():
    """Generates a new unique model name."""
    global model_count
    model_name = "object_{}".format(model_count)
    model_count += 1
    return model_name


def generate_position():
    """Generates a random position."""
    position = Point()
    position.x = random() * 20 - 10
    position.y = random() * 20 - 10

    # Constant offset off the ground.
    position.z = 0.05

    return position


def generate_pose(shape, shape_path, alphanumeric_path, shape_scale,
                  alphanumeric_scale):
    """Generates a pose for an alphanumeric w.r.t. to a given shape.

    This ensures that the alphanumeric is centered above it"s associated shape.

    Args:
        shape: The shape.
        shape_path: The path to the shape mesh.
        alphanumeric_path: The path to the alphanumeric mesh.
        shape_scale: The shape"s scale.
        alphanumeric_scale: The alphanumeric"s scale.

    Returns:
        A tuple of the alphanumeric"s (x, y, z, roll, pitch, yaw).
    """
    m_shape = Mesh.from_file(shape_path)
    m_alphanumeric = Mesh.from_file(alphanumeric_path)

    # Finding min and max coordinates of shape and alphanumeric
    shape_min = m_shape.min_
    shape_max = m_shape.max_
    alphanumeric_min = m_alphanumeric.min_
    alphanumeric_max = m_alphanumeric.max_

    # Finding the centers of both the shape and alphanumeric.
    # The z-value of the center of the alphanumeric should be disregarded.
    shape_center = ((shape_min + shape_max) / 2) * shape_scale
    alphanumeric_center = ((alphanumeric_min + alphanumeric_max) / 2)
    alphanumeric_center *= alphanumeric_scale
    pose_xyz = shape_center - alphanumeric_center

    # For quarter-circles, pentagons,  heptagons, octagons, stars, and triangles
    # we need to change the yaw of the alphanumeric wrt to the shape.
    yaw_dict = {
        "quarter_circle": -0.4,
        "pentagon": -0.4,
        "heptagon": 0.25,
        "octagon": -0.4,
        "star": -0.4,
        "triangle": -1.571,
    }

    yaw = 0
    if shape in yaw_dict:
        yaw = yaw_dict[shape]
    else:
        yaw = 0

    return (pose_xyz[0], pose_xyz[1], 0.05, 0, 0, yaw)


def generate_model_xml(shape, model_name, shape_path, alphanumeric_path,
                       background_color, alphanumeric_color):
    """Generates the model XML in the SDF format.

    Args:
        model_name: Model name.
        path: Path to STL file.
        color: Color of model.

    Returns:
        Model XML.
    """
    base_xml = """
    <?xml version="1.0"?>
    <sdf version="1.4">
      <model name="{model_name}">
        <static>true</static>
        <link name="shape">
          <visual name="visual_shape">
            <cast_shadows>false</cast_shadows>
            <geometry>
              <mesh>
                <uri>{shape_path}</uri>
                <scale>{shape_scale} {shape_scale} {shape_scale}</scale>
              </mesh>
            </geometry>
            <material>
              <ambient>{r_shape} {g_shape} {b_shape} {a_shape}</ambient>
              <diffuse>{r_shape} {g_shape} {b_shape} {a_shape}</diffuse>
            </material>
          </visual>
        </link>
        <link name="alphanumeric">
         <pose>{x_alphanumeric} {y_alphanumeric} {z_alphanumeric} {roll_alphanumeric} {pitch_alphanumeric} {yaw_alphanumeric}</pose>
           <visual name="visual_alphanumeric">
             <cast_shadows>false</cast_shadows>
               <transparency>0.0</transparency>
                 <geometry>
                   <mesh>
                     <uri>{alphanumeric_path}</uri>
                     <scale>{alphanumeric_scale} {alphanumeric_scale} 0.001</scale>
                   </mesh>
                 </geometry>
               <material>
                 <ambient>{r_alphanumeric} {g_alphanumeric} {b_alphanumeric} {a_alphanumeric}</ambient>
                 <diffuse>{r_alphanumeric} {g_alphanumeric} {b_alphanumeric} {a_alphanumeric}</diffuse>
               </material>
          </visual>
        </link>
      </model>
    </sdf>
    """
    r_shape, g_shape, b_shape, a_shape = rgba_colors[background_color]
    r_alphanumeric, g_alphanumeric, b_alphanumeric, a_alphanumeric = rgba_colors[
        alphanumeric_color]
    shape_scale = random() * 3
    alphanumeric_scale = shape_scale * 0.01
    x_alphanumeric, y_alphanumeric, z_alphanumeric, roll_alphanumeric, pitch_alphanumeric, yaw_alphanumeric = generate_pose(
        shape, shape_path, alphanumeric_path, shape_scale, alphanumeric_scale)
    return base_xml.format(
        model_name=model_name,
        shape_path=shape_path,
        alphanumeric_path=alphanumeric_path,
        shape_scale=shape_scale,
        alphanumeric_scale=alphanumeric_scale,
        r_shape=r_shape,
        g_shape=g_shape,
        b_shape=b_shape,
        a_shape=a_shape,
        r_alphanumeric=r_alphanumeric,
        g_alphanumeric=g_alphanumeric,
        b_alphanumeric=b_alphanumeric,
        a_alphanumeric=a_alphanumeric,
        x_alphanumeric=x_alphanumeric,
        y_alphanumeric=y_alphanumeric,
        z_alphanumeric=z_alphanumeric,
        roll_alphanumeric=roll_alphanumeric,
        pitch_alphanumeric=pitch_alphanumeric,
        yaw_alphanumeric=yaw_alphanumeric)


def spawn_object(req):
    """Spawns a random object in the simulator.

    Args:
        req: SpawnObjectRequest.

    Returns:
        SpawnObjectResponse.
    """
    response = SpawnObjectResponse()
    override = req.override
    ground_truth = response.ground_truth

    # Generate shape.
    seed_shape = override.shape.data if override.shape.data else None
    shape, shape_path = generate_shape(seed_shape)
    ground_truth.shape.data = shape

    # Generate alphanumeric.
    seed_alphanumeric = override.alphanumeric if override.alphanumeric else None
    alphanumeric, alphanumeric_path = generate_alphanumeric(seed_alphanumeric)
    ground_truth.alphanumeric = alphanumeric

    # Generate colors.
    if override.background_color.data:
        background_color = override.background_color.data
    else:
        background_color = choice(available_colors)
    ground_truth.background_color.data = background_color

    if override.alphanumeric_color.data:
        alphanumeric_color = override.alphanumeric_color.data
    else:
        alphanumeric_color = choice(available_colors)
        while background_color == alphanumeric_color:
            alphanumeric_color = choice(available_colors)
    ground_truth.alphanumeric_color.data = alphanumeric_color

    # Generate position.
    # TODO: Restrict the position within a search_grid provided and set the
    # proper GPS coordinates in the ground truth.
    position = generate_position()

    # Generate model name.
    model_name = generate_model_name()

    # Spawn model.
    spawn_req = SpawnModelRequest()
    spawn_req.model_name = model_name
    spawn_req.model_xml = generate_model_xml(
        shape, model_name, shape_path, alphanumeric_path, background_color,
        alphanumeric_color)
    spawn_req.initial_pose.position = position
    spawn_req.initial_pose.orientation.w = 1
    spawn_response = spawn_service.call(spawn_req)

    # Set up response.
    response.success = spawn_response.success
    response.message = spawn_response.status_message
    response.model_name = model_name
    response.ground_truth = ground_truth

    return response


if __name__ == "__main__":
    rospy.init_node("object_generator")

    spawn_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    rospy.Service("~spawn", SpawnObject, spawn_object)

    rospy.spin()
