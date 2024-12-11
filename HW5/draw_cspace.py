import math
import numpy as np
import dubins
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt
import networkx as nx

def drawrrt(ax, cspace, obstacles, qI, qG, G, path, title=""):
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

    draw_cspace(ax, cspace, obstacles)
    drawrrtgraph(G, ax)
    if qI is not None:
        if len(qI) == 2:
            ax.plot(qI[0], qI[1], "bx", markersize=5)
        elif len(qI) == 3:
            ax.plot(
                qI[0],
                qI[1],
                marker=(3, 0, qI[2] * 180 / math.pi - 90),
                markersize=15,
                linestyle="None",
                markerfacecolor="blue",
                markeredgecolor="blue",
            )
    if qG is not None:
        if len(qI) == 2:
            ax.plot(qG[0], qG[1], "bo", markersize=5)
        elif len(qG) == 3:
            ax.plot(
                qG[0],
                qG[1],
                marker=(3, 0, qG[2] * 180 / math.pi - 90),
                markersize=15,
                linestyle="None",
                markerfacecolor="red",
                markeredgecolor="red",
            )
    if len(path) > 0:
        ax.plot(
            [state[0] for state in path],
            [state[1] for state in path],
            "b-",
            linewidth=3,
        )
    if len(title) > 0:
        ax.set_title(title, fontsize=20)

def drawprm(ax, cspace, obstacles, qI, qG, G, step_size, title=""):
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

    draw_cspace(ax, cspace, obstacles)
    drawprmgraph(G, ax, qI, qG, step_size, turning_rad = 0.5)
    if qI is not None:
        if len(qI) == 2:
            ax.plot(qI[0], qI[1], "bx", markersize=5)
        elif len(qI) == 3:
            ax.plot(
                qI[0],
                qI[1],
                marker=(3, 0, qI[2] * 180 / math.pi - 90),
                markersize=15,
                linestyle="None",
                markerfacecolor="blue",
                markeredgecolor="blue",
            )
    if qG is not None:
        if len(qI) == 2:
            ax.plot(qG[0], qG[1], "bo", markersize=5)
        elif len(qG) == 3:
            ax.plot(
                qG[0],
                qG[1],
                marker=(3, 0, qG[2] * 180 / math.pi - 90),
                markersize=15,
                linestyle="None",
                markerfacecolor="red",
                markeredgecolor="red",
            )
    if len(title) > 0:
        ax.set_title(title, fontsize=20)


def draw_cspace(ax, cspace, obstacles, tick_step=[1, 1]):
    """Draw the C-space and C-space obstacles on the axis ax

    @type cspace: a list [(xmin, xmax), (ymin, ymax)] indicating that the C-space
        is given by [xmin, xmax] \times [ymin, ymax].
    @type obstacles: a list [obs_1, ..., obs_m] of obstacles, where obs_i is a list of coordinates
        on the boundary of the i^{th} obstacle.
    """
    for obs in obstacles:
        ax.plot([v[0] for v in obs], [v[1] for v in obs], "r-", linewidth=3)

    ax.set_xticks(
        range(math.ceil(cspace[0][0]), math.floor(cspace[0][1]) + 1, tick_step[0])
    )
    ax.set_yticks(
        range(math.ceil(cspace[1][0]), math.floor(cspace[1][1]) + 1, tick_step[1])
    )
    ax.set(xlim=cspace[0], ylim=cspace[1])
    ax.set_aspect("equal", adjustable="box")
    ax.tick_params(axis="x", labelsize=20)
    ax.tick_params(axis="y", labelsize=20)

def drawrrtgraph(G, ax):
    """Draw the graph on the axis ax"""
    # Draw nodes (vertices)
    for node, data in G.nodes(data=True):  # Iterate through nodes and their attributes
        state = node  # Assuming nodes are in form (x, y, theta)
        
        # Check if tipping_point is 1, and plot the marker (orientation arrow) for that node
        if 'tipping_point' in data and data['tipping_point'] == 0:
            # If tipping_point is 1, draw with orientation marker
            if len(state) == 3:
                ax.plot(
                    state[0],
                    state[1],
                    marker=(3, 0, state[2] * 180 / math.pi - 90),  # Orientation marker
                    markersize=8,
                    linestyle="None",
                    markerfacecolor="black",
                    markeredgecolor="black",
                )
        else:
            # If no tipping_point or tipping_point is 0, draw as simple 2D points
            ax.plot(state[0], state[1], "k.", markersize=0.5, linewidth=2)

    # Draw edges (connections between nodes)
    for (node1, node2) in G.edges:
        state1 = node1  # Example: (x1, y1, theta1)
        state2 = node2  # Example: (x2, y2, theta2)
        ax.plot([state1[0], state2[0]], [state1[1], state2[1]], "k-", linewidth=1)  # Plot edges as lines

def drawprmgraph(G, ax, qI, qG, step_size, turning_rad):
    for node in G.nodes:
        path = nx.shortest_path(G, qI, qG)
        # print("path is:", path)
        for e in G.edges:
            u, v = e  # e is a tuple of the form (u, v)
            # print(f"Edge {e} connects vertex {u} to vertex {v}")
            c = 'black'
            if e[0] in path and e[1] in path:
                # print("if e[0] in path and e[1] in path:")
                c = 'blue'
            # print("e[0] and e[1]", np.array(e[0]), np.array(e[1]))          

            curve = dubins.shortest_path(np.array(e[0]), np.array(e[1]), turning_rad)

            # Sample points along the Dubins path
            configurations, _ = curve.sample_many(step_size)
            configurations.append(e[1])

            # Define an array for tipping_point attribute
            tipping_points = [0] * (len(configurations) - 2) + [1]
            # print("Tipping Points", tipping_points)

            # Plot the path samples
            for idx, configuration in enumerate(configurations[:-1]):
                s1 = configuration[:-1] 
                s2 = configurations[idx + 1][:-1]
                
                # Use the tipping_point for debugging or other purposes
                # print(f"Configuration: {configuration}, Tipping Point: {tipping_points[idx]}")
                # Check if tipping_point is 1, and plot the marker (orientation arrow) for that node
                if tipping_points[idx] == 1:
                    # If tipping_point is 1, draw with orientation marker
                    if len(configuration) == 3:
                        ax.plot(
                            configuration[0],
                            configuration[1],
                            marker=(3, 0, configuration[2] * 180 / math.pi - 90),  # Orientation marker
                            markersize=8,
                            linestyle="None",
                            markerfacecolor="black",
                            markeredgecolor="black",
                        )
                else:
                    # If no tipping_point or tipping_point is 0, draw as simple 2D points
                    ax.plot(s1[0], s1[1], "k.", markersize=0.5, linewidth=1.5)

                # Draw the segment
                ax.plot((s1[0], s2[0]), (s1[1], s2[1]), c, linewidth=1.5)
