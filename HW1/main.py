import collections
from collections import deque
from queue import Queue
import heapq
import math
import numpy as np
import json 
from derived_classes import StateSpacefor2D, ActionSpacefor2D, StateTransitionfor2D
from derived_queues import QueueAstar, QueueBFS


def fsearch(X, U, f, xI, XG, alg):
    """
    Args:
    X = State Space
    U = Action Space
    f = State Transition Function
    xI = Initial State
    XG = Goal State
    alg = The Planning Algorithm
    """
    if alg == 'bfs':
        Q = QueueBFS()
    elif alg == 'astar':
        Q = QueueAstar(X, XG)
        
    visited = set()
    Q.insert(xI, None)
    parent = {}

    while Q.queue:
        #if not Q.queue:  # Check if the queue is empty
            #print("Queue is empty in fsearch")
            # break
        x = Q.pop()  

        if x in visited:
            continue
        visited.add(x)

        if x == XG:
            # print("State", x)
            # Reconstruct path
            path = []
            while x is not None:
                path.insert(0, x)
                x = Q.parent[x]
            return {"visited": visited, "path": path}
        for u in U(x):
            x_prime = f(x, u)
            if x_prime not in visited:
                Q.insert(x_prime, x)  

    return {"visited": visited, "path": None}
    
# Entry point of the program
if __name__ == "__main__":
    # Load data from input.json
    with open('input.json', 'r') as file:
        data = json.load(file)

    # Assign the values from the JSON to the variables
    X_max = data['X_max']
    Y_max = data['Y_max']
    obstacles = [tuple(x) for x in data['obstacles']]
    xI = tuple(data['xI'])
    XG = tuple(data['XG'])

    # Print the loaded data
    # print("X_max:", X_max, "Y_max:", Y_max, "obstacles:", obstacles, "Initial State:", xI, "Goal State:", XG)

    X = StateSpacefor2D(X_max, Y_max, obstacles)
    f = StateTransitionfor2D()
    U = ActionSpacefor2D(X, f)
    
    # Run fsearch with BFS
    result_bfs = fsearch(X, U, f, xI, XG, 'bfs')
    print("BFS results:")
    print(result_bfs)

    # Run fsearch with A* algorithm
    result_astar = fsearch(X, U, f, xI, XG, 'astar')
    print("AStar results:")
    print(result_astar)
