import math
import matplotlib.pyplot as plt
import networkx as nx

def draw(ax, cspace, obstacles, region_radius, qI, qG, G, path, title="", algorithm=""):
    """Plot the C-space, obstacles, qI, qG, and graph on the axis ax
    @type ax: axes.Axes, created, e.g., fig, ax = plt.subplots()
    @type cspace: a list [(xmin, xmax), (ymin, ymax)] indicating that the C-space
        is given by [xmin, xmax] \times [ymin, ymax].
    @type G: an object with draw(ax) method. This object represents a graph to be drawn.
    @type obstacles: a list [obs_1, ..., obs_m] of obstacles, where obs_i is a list of coordinates
        on the boundary of the i^{th} obstacle.
    @type qI: a tuple (x, y), indicating the initial configuration.
    @type qG: a tuple (x, y), indicating the goal configuration
    @type path: a list of tuples specifying the sequence of configurations visited along the path
    @type title: a string, indicating the title of the plot
    """
    print(f"In the draw space.")
    draw_cspace(ax, cspace, obstacles)
    if algorithm == "RRT":
        G.draw(ax)
    if algorithm == "RRTS":
        # Draw the edges
        for u, v in G.edges():
            if u in G.nodes and v in G.nodes:
                # If both u and v are nodes in the graph, do something
                # print(f"Node {u} and Node {v} are in the graph.")
                ax.plot(u[0], u[1], "k.",  linewidth=0.2)
                ax.plot(v[0], v[1], "k.", linewidth=0.2)
                ax.plot((u[0], v[0]), (u[1], v[1]), color = 'black', linewidth=1)
    if qI is not None:
        if len(qI) == 2:
            ax.plot(qI[0], qI[1], "bx", markersize=5)
        elif len(qI) == 3:
            ax.plot(
                qI[0],
                qI[1],
                marker=(3, 0, qI[2] * 180 / math.pi - 90),
                markersize=5,
                linestyle="None",
                markerfacecolor="blue",
                markeredgecolor="blue",
            )
    if qG is not None:
        if len(qG) == 2:
            ax.plot(qG[0], qG[1], "bo", markersize=5)

        elif len(qG) == 3:
            ax.plot(
                qG[0],
                qG[1],
                marker=(3, 0, qG[2] * 180 / math.pi - 90),
                markersize=5,
                linestyle="None",
                markerfacecolor="red",
                markeredgecolor="red",
            )
        goal_circle = plt.Circle((qG[0], qG[1]), region_radius, edgecolor='black', facecolor='magenta', fill=True, alpha=0.9, linewidth=0.5)  # Adjust alpha for transparency
        ax.add_patch(goal_circle)

    if len(path) > 0:
        ax.plot(
            [state[0] for state in path],
            [state[1] for state in path],
            "g-",
            linewidth=3,
        )
    if len(title) > 0:
        ax.set_title(title, fontsize=10)


def draw_cspace(ax, cspace, obstacles, tick_step=[1, 1]):
    """Draw the C-space and C-space obstacles on the axis ax

    @type cspace: a list [(xmin, xmax), (ymin, ymax)] indicating that the C-space
        is given by [xmin, xmax] \times [ymin, ymax].
    @type obstacles: a list [obs_1, ..., obs_m] of obstacles, where obs_i is a list of coordinates
        on the boundary of the i^{th} obstacle.
    """
    for obs in obstacles:
        x, y, radius = obs
        circle = plt.Circle((x, y), radius, color='r', fill=False, linewidth=0.5)  # Red circle, not filled
        ax.add_patch(circle)

    ax.set(xlim=cspace[0], ylim=cspace[1])
    ax.set_aspect("equal", adjustable="box")
