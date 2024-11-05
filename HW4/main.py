import argparse
import json
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from planners import (
    RRT,
    PRM,
    StraightEdgeCreator,
    EuclideanDistanceComputator,
    EmptyCollisionChecker,
    ObstacleCollisionChecker,
)
from drawer import Drawer

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Run RRT variations and PRM")
    
    # Add dt parameter
    parser.add_argument(
        "-dt",
        type=float,
        required=False,
        default=0.02,  # default value from your JSON
        help="Deviation from radius (default: 0.02)",
    )
    
    # Add algorithm selection parameter
    parser.add_argument(
        "-algorithm",
        choices=['simple_RRT', 'RRT_with_obstacles', 'single_tree_search_RRT', 'PRM'],  # replace with your actual algorithm names
        required=False,
        default='simple_RRT',
        help="choose which algorithm to run (default: simple_RRT)",
    )

    # Add arguments for PRM parameters
    parser.add_argument("-K", type=int, default=15,
                       help="number of nearest neighbors for PRM (default: 15)")
    parser.add_argument("-step_size", type=float, default=0.1,
                       help="step size for RRT extension (default: 0.1)")
    parser.add_argument("-N", type=int, default=1000,
                       help="number of iterations (default: 1000)")
    parser.add_argument("-p", type=float, default=0.1,
                       help="sampling probability for RRT (default: 0.1)")
    
    return parser.parse_args()

def parse_json():
    with open('input.json', 'r') as f:
        data = json.load(f)
    
    C = data['C']      # List of lists: [[float, float], [float, float]]
    O = data['O']      # List of lists: [[float, float], [float, float]]
    radius = data['radius'] - args.dt  # float
    qI = data['qI']        # List: [float, float]
    qG = data['qG']        # List: [float, float]
    
    return C, O, radius, qI, qG

if __name__ == '__main__':
    # Parse command line arguments
    args = parse_args()
    
    # Get parameters from input.json
    (C, O, radius, qI, qG) = parse_json()

    Plot = Drawer(C, O, radius)
    
    print(f"Using dt = {args.dt}")
    print(f"Running algorithm: {args.algorithm}")
    print(f"Obstacle radius: {radius}")
    print(f"Obstacle itself: {O}")
    
    # You can then use the algorithm variable to determine which RRT variant to run
    if args.algorithm == 'simple_RRT':
        # Run simple RRT. No obstacles list needed. "step_size" will be a very big number.
        (G, _, _) = RRT(
            C=C,
            qI=qI,
            qG=None,
            edge_creator = StraightEdgeCreator(args.step_size),
            distance_computator = ObstacleCollisionChecker(O),
            collision_checker=EmptyCollisionChecker(),
            N=args.N,)
        Plot.draw(qI, qG, G, path = None, algorithm='simple_RRT')

    elif args.algorithm == 'RRT_with_obstacles':
        # Run RRT considering obstacles
        print("Starting RRT with obstacles...")
        (G, _, _) = RRT(
            C=C,
            qI=qI,
            qG=None,
            edge_creator = StraightEdgeCreator(args.step_size),
            distance_computator = ObstacleCollisionChecker(O),
            collision_checker = EuclideanDistanceComputator(),
            N=args.N,)
        Plot.draw(qI, qG, G, path = None, algorithm='RRT_with_obstacles')
        

    elif args.algorithm == 'single_tree_search_RRT':
        # Run RRT equipped with single-tree search
        (G, root, goal) = PRM(
            C=C,
            qI=qI,
            qG=qG,
            edge_creator = StraightEdgeCreator(args.step_size),
            distance_computator = ObstacleCollisionChecker(O),
            collision_checker = EuclideanDistanceComputator(),
            N=args.N,)
        path = []
        if root is not None and goal is not None:
            path = G.get_path(root, goal)
        Plot.draw(qI, qG, G, path = path, algorithm='RRT_with_obstacles')

    elif args.algorithm == 'PRM':
        # Run probabilistic roadmap
        (G, root, goal) = PRM(
            cspace=C,
            qI=qI,
            qG=qG,
            edge_creator = StraightEdgeCreator(args.step_size),
            distance_computator = ObstacleCollisionChecker(O),
            collision_checker = EuclideanDistanceComputator(),
            k=args.K,
            N=args.N,)
        path = []
        if root is not None and goal is not None:
            path = G.get_path(root, goal)
        Plot.draw(qI, qG, G, path = path, algorithm='PRM')
