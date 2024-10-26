import argparse
import json
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from planners import RRT, PRM
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
    parser.add_argument("-N", type=int, default=1000,
                       help="number of nodes for PRM (default: 1000)")
    parser.add_argument("-K", type=int, default=15,
                       help="number of nearest neighbors for PRM (default: 15)")
    parser.add_argument("-step_size", type=float, default=0.1,
                       help="step size for RRT extension (default: 0.1)")
    parser.add_argument("-n_iterations", type=int, default=1000,
                       help="number of iterations for RRT (default: 1000)")
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
        tree = RRT(C, [], radius, qI, step_size=float('inf'), n_iterations=args.n_iterations).explore()
        Plot.visualize_rrt(tree, qI, qG, algorithm='simple_RRT', path = None)

    elif args.algorithm == 'RRT_with_obstacles':
        # Run RRT considering obstacles
        print("Starting RRT with obstacles...")
        tree = RRT(C, O, radius, qI, step_size=args.step_size, n_iterations=args.n_iterations).explore()
        print("tree", tree)
        Plot.visualize_rrt(tree, qI, qG, algorithm='RRT_with_obstacles', path = None)
        print("RRT exploration complete!")

    elif args.algorithm == 'single_tree_search_RRT':
        # Run RRT equipped with single-tree search
        path, tree = RRT(C, O, radius, qI, step_size=100, n_iterations=args.n_iterations).plan_to_goal(qG, p=args.p)
        Plot.visualize_rrt(tree, qI, qG, algorithm='single_tree_search_RRT', path = path)

    elif args.algorithm == 'PRM':
        # Run probabilistic roadmap
        path, roadmap = PRM(C, O, radius, N=args.N, K=args.K, step_size=args.step_size).plan_path(qI, qG)
        Plot.visualize_prm(roadmap, qI, qG, algorithm='PRM', path = path)
