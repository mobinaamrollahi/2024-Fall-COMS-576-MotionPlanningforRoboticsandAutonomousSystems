import sys, argparse
import math
import matplotlib.pyplot as plt
from planning import *
from obstacle import *
from draw_cspace import drawrrt, drawprm

ALG_RRT = "rrt"
ALG_PRM = "prm"


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Run sampling-based motion planning algorithm"
    )
    parser.add_argument(
        "--alg",
        choices=[ALG_RRT, ALG_PRM],
        required=False,
        default=ALG_PRM,
        dest="alg",
        help="algorithm, default to rrt",
    )
    args = parser.parse_args(sys.argv[1:])
    return args

if __name__ == "__main__":
    # Parameters
    cspace = [(-3, 3), (-1, 1)]
    # qI = ConfigDubins((-2, -0.5, 0))
    # qG = ConfigDubins((2, -0.5, math.pi/2))
    qI = [-2, -0.5, 0]
    qG = [2, -0.5, math.pi/2]
    dt = 0.2
    k = 15,
    c = [(0, -1), (0, 1)]  # the center of each circle
    rho = 0.5

    # obstacles
    o1 = CircularObstacle(x = 0, y = -1, r = 1 - dt)
    o2 = CircularObstacle(x = 0, y = 1, r = 1 - dt)

    obstacles = [o1, o2]
    # for obstacle in obstacles: print(obstacle.x, obstacle.y, obstacle.r)
    obs_boundaries = [obs.get_boundary() for obs in obstacles]

    setup = DubinsCarPathPlanning(
        cspace = cspace,
        qI = qI,
        qG = qG,
        turning_rad = rho,
        obstacles = obstacles,
        Xmin= -3,
        Xmax= 3, 
        Ymin= -1, 
        Ymax= 1,
    )

    args = parse_args()

    if args.alg == ALG_RRT:
        (G, root, goal) = setup.rrt(
            pG=0.1,
            numIt=200,
            tol=1e-3,
            step_size=0.1
        )
        fig, ax = plt.subplots()
        # Print all nodes (vertices) in the graph
        # print("Vertices in the graph:", list(G.nodes))
        # Check if the node exists in the graph
        if tuple(qG) in G:
            print(f"The node goal is in the graph.")
        else:
            print(f"The node goal is NOT in the graph.")
        print("root and goal", root, goal)
        path = []
        if root is not None:
            print("Hello")
            # if nx.has_path(G, root, goal):
            path = nx.shortest_path(G, tuple(qI), tuple(qG))
            print("Path between root and goal", path)

        drawrrt(ax, cspace, obs_boundaries, qI, qG, G, path, title = "RRT planning")
        ax.set_xlabel(r"$x$", fontsize=20)
        ax.set_ylabel(r"$y$", fontsize=20)
        plt.show()

    else:
        (G, root, goal) = setup.prm(
            k=15,
            numIt=30,
            tol=1e-3,
            step_size=0.1
        )
        # print("Nodes in the graph:", G.nodes)
        # print("Edges in the graph:", G.edges)
        # node1 = list(G.nodes())[0]  # Take the first node in the graph
        # node2 = list(G.nodes())[250]  # Take the second node in the graph
        # path = nx.shortest_path(G, node1, node2)
        # print("Path between node1 and node2:", path)
        if tuple(root) in G:
            print("qI is in the graph", root)
        else:
            print("qI is NOT in the graph", goal)

        if tuple(goal) in G:
            print("qG is in the graph", goal)
        else:
            print("qG is NOT in the graph")

        fig, ax = plt.subplots()
        path = []
        if root is not None and goal is not None:
            if nx.has_path(G, root, goal):
                path = nx.shortest_path(G, root, goal)
                print("Path between root and goal", path)
        
        drawprm(ax, cspace, obs_boundaries, tuple(qI), tuple(qG), G, 0.1, title = "PRM planning")
        ax.set_xlabel(r"$x$", fontsize=20)
        ax.set_ylabel(r"$y$", fontsize=20)
        plt.show()
        
        
