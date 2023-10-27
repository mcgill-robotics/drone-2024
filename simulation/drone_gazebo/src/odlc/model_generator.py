# -*- coding: utf-8 -*-

import numpy as np
from stl import Mesh
from matplotlib import pyplot
from mpl_toolkits import mplot3d


def plot(mesh):
    """Plots a mesh in 3D.

    Args:
        mesh: Mesh to plot.
    """
    figure = pyplot.figure()
    axes = mplot3d.Axes3D(figure)

    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh.vectors))

    scale = mesh.points.flatten(-1)
    axes.auto_scale_xyz(scale, scale, scale)

    pyplot.show()


def regular_polygon(sides, radius=1.0, sector_ratio=1.0):
    """Generates a regular polygon mesh.

    Args:
        sides: Number of sides.
        radius: Radius of polygon.
        sector_ratio: Ratio of the polygon to generate.

    Returns:
        Mesh.
    """
    theta = 0
    dtheta = 2 * np.pi / sides

    sides = int(sides * sector_ratio)
    data = np.zeros(sides, dtype=Mesh.dtype)

    for i in range(sides):
        data["vectors"][i] = radius * \
            np.array([[0, 0, 0],
                      [np.cos(theta), np.sin(theta), 0],
                      [np.cos(theta + dtheta), np.sin(theta + dtheta), 0]])
        theta += dtheta

    return Mesh(data)


def rectangle(width, height):
    """Generates a rectanglular mesh.

    Args:
        width: Width of the rectangle.
        height: Height of the rectangle.

    Returns:
        Mesh.
    """
    data = np.zeros(2, dtype=Mesh.dtype)

    # yapf: disable
    data["vectors"][0] = np.array([[-width / 2.0, -height / 2.0, 0],
                                   [width / 2.0, -height / 2.0, 0],
                                   [width / 2.0, height / 2.0, 0]])
    data["vectors"][1] = np.array([[width / 2.0, height / 2.0, 0],
                                   [-width / 2.0, height / 2.0, 0],
                                   [-width / 2.0, -height / 2.0, 0]])
    # yapf: enable

    return Mesh(data)


def star(points, radius=1.0, inner_ratio=0.53):
    """Generates a star mesh.

    Args:
        points: Number of points in the star.
        radius: Size of the star.
        inner_ratio: Inner ratio.

    Returns:
        Mesh.
    """
    inner_radius = radius * inner_ratio
    data = np.zeros(2 * points, dtype=Mesh.dtype)

    theta = 0
    dtheta = 2 * np.pi / points
    for i in range(points):
        origin = [0, 0, 0]
        peak = radius * np.array([np.cos(theta), np.sin(theta), 0])
        left = inner_radius * np.array(
            [np.cos(theta + dtheta / 2),
             np.sin(theta + dtheta / 2), 0])
        right = inner_radius * np.array(
            [np.cos(theta - dtheta / 2),
             np.sin(theta - dtheta / 2), 0])
        data["vectors"][2 * i] = np.array([origin, right, peak])
        data["vectors"][2 * i + 1] = np.array([origin, peak, left])

        theta += dtheta

    return Mesh(data)


def combine(mesh1, mesh2):
    """Combines two meshes together.

    Args:
        mesh1: First mesh.
        mesh2: Second mesh.

    Returns:
        Mesh - sum of its parts.
    """
    mesh1_size = len(mesh1.data)
    mesh2_size = len(mesh2.data)
    data = np.zeros(mesh1_size + mesh2_size, dtype=Mesh.dtype)

    for i in range(mesh1_size):
        data["vectors"][i] = mesh1.data["vectors"][i]

    for i in range(mesh2_size):
        data["vectors"][mesh1_size + i] = mesh2.data["vectors"][i]

    return Mesh(data)


def cross(width, height, thickness=0.5):
    """Generates a cross mesh.

    Args:
        width: Width of the cross.
        height: Height of the cross.
        thickness: Thickness of the cross.

    Returns:
        Mesh.
    """
    return combine(rectangle(width, thickness), rectangle(thickness, height))


def trapezoid(width, height, side_ratio=0.6):
    """Generates a trapezoid mesh.

    Args:
        width: Width of the cross.
        height: Height of the cross.
        side_ratio: Ratio of small width to width.

    Returns:
        Mesh.
    """
    small_width = width * side_ratio
    data = np.zeros(2, dtype=Mesh.dtype)

    # yapf: disable
    data["vectors"][0] = np.array([[-width / 2.0, -height / 2.0, 0],
                                   [width / 2.0, -height / 2.0, 0],
                                   [small_width / 2.0, height / 2.0, 0]])
    data["vectors"][1] = np.array([[small_width / 2.0, height / 2.0, 0],
                                   [-small_width / 2.0, height / 2.0, 0],
                                   [-width / 2.0, -height / 2.0, 0]])
    # yapf: enable

    return Mesh(data)
