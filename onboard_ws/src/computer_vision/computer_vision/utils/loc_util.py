import numpy as np


def get_n_e_d_from_pixel(n, e, d, heading, pix, piy, nx, ny, sensor_side,
                         focal_length):
    cam_vec = get_camera_vec(pix, piy, nx, ny, sensor_side, focal_length, d)
    return get_world_vec(cam_vec, n, e, d, heading)


def get_camera_vec(pix, piy, nx, ny, sensor_side, focal_length,
                   distance_from_plane):
    """
        Returns the vector from a camera at origin 0,0,0 that would intersect with a plane at
        distance distance_from_plane,
        at the pixel pix, piy out of nx,ny pixels of an image taken from a camera with a square sensor,
        sensor side measuring sensor_side (in m), and with a lense with focal length focal_length (in m)
    """
    # assumes target is at height 0.
    u_vec = np.array([1, 0, 0])
    v_vec = np.array([0, 1, 0])
    w_vec = np.array([0, 0, 1])
    e_vec = np.array([0, 0, 0])
    l, b = -sensor_side / 2, -sensor_side / 2
    r, t = sensor_side / 2, sensor_side / 2
    distance = focal_length
    u = l + (r - l) * (pix + 0.5) / nx
    v = b + (t - b) * (piy + 0.5) / ny

    s = e_vec + u * u_vec + v * v_vec - abs(distance) * w_vec
    p = e_vec
    d = s - e_vec

    intersect_t = (abs(distance_from_plane) - p[2]) / d[2]

    return p + intersect_t * d


def get_world_vec(cam_vec, n, e, d, heading):
    homogeneous_cam_vec = np.array([*cam_vec, 1])
    up = np.array([1, 0, 0])
    up = np.array([
        np.cos(heading) * up[0] - np.sin(heading) * up[1],
        np.sin(heading) * up[0] + np.cos(heading) * up[1], up[2]
    ])
    at = np.array([n, e, 0])
    eye = np.array([n, e, d])
    l = at - eye
    l = l / np.linalg.norm(l)
    v = up / np.linalg.norm(up)
    u = np.cross(v, l)
    u = u / np.linalg.norm(u)
    w = l

    trans = np.array([[*u, 0], [*v, 0], [*w, 0], [*eye, 1]])
    trans = trans.T
    return np.matmul(trans, homogeneous_cam_vec)[0:3]
