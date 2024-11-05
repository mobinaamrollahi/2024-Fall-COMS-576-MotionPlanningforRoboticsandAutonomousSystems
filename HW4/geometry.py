import math
import numpy as np

def get_nearest_point_on_line(start_config, end_config, q_rand, tol=1e-3):
    """Compute the nearest point on a line described by start_config and end_config to q_rand.

    Note that a point on the line can be parametrized by
        s(t) = start_config + t(end_config - start_config).
    s(t) is on the line segment between start_config and end_config iff t \in [0, 1].

    The point on the line closest to q_rand is s(t*) where
        t* = <p-start_config, end_config-start_config> / ||end_config - start_config||^2

    @return (s*, t*) where s* = s(t*)
    """
    ls = end_config - start_config  # The line segment from start_config to end_config
    len_lend_config = np.dot(ls, ls)  # the squared length of ls

    # If the line segment is too short, just return 0
    if len_lend_config < tol:
        return (start_config, 0)

    tstar = np.dot(q_rand - start_config, ls) / len_lend_config
    if tstar <= tol:
        return (start_config, 0)
    if tstar >= 1 - tol:
        return (end_config, 1)

    return (start_config + tstar * ls, tstar)


def get_euclidean_distance(start_config, end_config):
    """Compute the norm ||end_config - start_config||"""
    norm = np.array(end_config) - np.array(start_config)
    return np.linalg.norm(norm)


def is_inside_circle(c, r, p):
    """Return whether point p is inside a circle with radius r, centered at c"""
    return (p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2 <= r ** 2
