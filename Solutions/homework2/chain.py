import sys, argparse, math
import numpy as np
from draw_chain import plot_chain



def get_link_positions(config, W, L, D):
    """Compute the positions of the links and the joints of a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link

    @return: a tuple (joint_positions, link_vertices) where
        * joint_positions is a list [p_1, ..., p_{m+1}] where p_i is the position [x,y] of the joint between A_i and A_{i-1}
        * link_vertices is a list [V_1, ..., V_m] where V_i is the list of [x,y] positions of vertices of A_i
    """

    if len(config) == 0:
        return ([], [])

    joint_positions = [np.array([0, 0, 1])]
    link_vertices = []

    link_vertices_body = [
        np.array([-(L - D) / 2, -W / 2, 1]),
        np.array([D + (L - D) / 2, -W / 2, 1]),
        np.array([D + (L - D) / 2, W / 2, 1]),
        np.array([-(L - D) / 2, W / 2, 1]),
    ]
    joint_body = np.array([D, 0, 1])
    trans_mat = np.array(
        [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
        ]
    )

    for i in range(len(config)):
        a = D if i > 0 else 0
        trans_mat = np.matmul(trans_mat, get_trans_mat(config[i], a))
        joint = np.matmul(trans_mat, joint_body)
        vertices = [
            np.matmul(trans_mat, link_vertex) for link_vertex in link_vertices_body
        ]
        joint_positions.append(joint)
        link_vertices.append(vertices)

    return (joint_positions, link_vertices)


def get_trans_mat(theta, a):
    """Return the homogeneous transformation matrix"""
    return np.array(
        [
            [math.cos(theta), -math.sin(theta), a],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1],
        ]
    )


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Display the arrangement of 2D kinematic chain"
    )
    parser.add_argument(
        "config",
        metavar="config",
        type=float,
        nargs="+",
        help="chain configuration theta_1, ..., theta_m",
    )
    parser.add_argument(
        "-W", type=float, required=True, dest="W", help="the width of each link"
    )
    parser.add_argument(
        "-L", type=float, required=True, dest="L", help="the length of each link"
    )
    parser.add_argument(
        "-D",
        type=float,
        required=True,
        dest="D",
        help="the distance between the two points of attachment",
    )

    args = parser.parse_args(sys.argv[1:])

    return args


if __name__ == "__main__":
    args = parse_args()
    (joint_positions, link_vertices) = get_link_positions(args.config, args.W, args.L, args.D)

    print()
    print("joint positions:")
    print(joint_positions)
    print()
    print("link vertices:")
    print(link_vertices)

    plot_chain(joint_positions, link_vertices)
