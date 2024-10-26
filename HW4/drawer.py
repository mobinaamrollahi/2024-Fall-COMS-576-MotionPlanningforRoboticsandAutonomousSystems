import matplotlib.pyplot as plt
import numpy as np
from typing import List, Optional
from abstract_base import AbstractTreeGraph, AbstractRoadmapGraph

class Drawer:
    def __init__(self, C: List[List[float]], O: List[List[float]], radius: float):
        """
        Initialize Drawer with configuration space parameters
        Args:
            C: Configuration space bounds [[x_min, x_max], [y_min, y_max]]
            O: Obstacle centers (empty list if no obstacles)
            radius: Obstacle radius
        """
        self.C = C
        self.O = O
        self.radius = radius
        
    def setup_plot(self):
        """Set up the basic plot with grid and obstacles"""
        plt.figure(figsize=(6, 2))

        x_min, x_max = self.C[0][0], self.C[0][1]
        y_min, y_max = self.C[1][0], self.C[1][1]

        # Create ticks at unit intervals
        x_ticks = np.arange(np.floor(x_min), np.ceil(x_max) + 1)
        y_ticks = np.arange(np.floor(y_min), np.ceil(y_max) + 1)

        # Set plot bounds with some padding
        plt.xlim(self.C[0][0], self.C[0][1])
        plt.ylim(self.C[1][0], self.C[1][1])
        plt.xticks(x_ticks)
        plt.yticks(y_ticks)
        
        # Create grid
        # plt.grid(True, color='black', alpha=0.2)
        # plt.axhline(y=0, color='black', alpha=0.5)
        # plt.axvline(x=0, color='black', alpha=0.5)
        
        # Plot obstacles if they exist
        if self.O:
            self._plot_obstacles()
    
    def _plot_obstacles(self):
        """Plot half-circle obstacles"""
        for center in self.O:
            # Create points for upper or lower half circle based on center y-coordinate
            theta = np.linspace(0, np.pi, 100) if center[1] < 0 else np.linspace(np.pi, 2*np.pi, 100)
            x = center[0] + self.radius * np.cos(theta)
            y = center[1] + self.radius * np.sin(theta)
            
            # Plot half circle
            plt.plot(x, y, 'r-', linewidth=2)
    
    def plot_point(self, point: List[float], style: str):
        """Plot a point with specified style"""
        plt.plot(point[0], point[1], style)
    
    def plot_tree(self, tree: AbstractTreeGraph):
        """Plot tree vertices and edges"""
        print(f"Plotting tree with {len(tree.vertices)} vertices")

        # Plot vertices
        for config in tree.vertices.values():
            plt.plot(config[0], config[1], 'k.', markersize=2)
        
        # Plot edges
        for v_id, edges in tree.edges.items():
            v1 = tree.get_vertex_config(v_id)
            for neighbor_id, _ in edges:
                v2 = tree.get_vertex_config(neighbor_id)
                plt.plot([v1[0], v2[0]], [v1[1], v2[1]], 'k-', linewidth=0.5)
    
    def plot_path(self, path: List[List[float]]):
        """Plot path as thick blue line"""
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], 'b-', linewidth=2)
    
    def plot_roadmap(self, roadmap: AbstractRoadmapGraph):
        """Plot PRM roadmap"""
        # Plot vertices
        for config in roadmap.vertices.values():
            plt.plot(config[0], config[1], 'k.', markersize=2)
        
        # Plot edges
        for v_id, edges in roadmap.edges.items():
            v1 = roadmap.get_vertex_config(v_id)
            for neighbor_id, _ in edges:
                v2 = roadmap.get_vertex_config(neighbor_id)
                plt.plot([v1[0], v2[0]], [v1[1], v2[1]], 'k-', linewidth=0.5)
    
    def visualize_rrt(self, tree: AbstractTreeGraph, qI: List[float], 
                     qG: Optional[List[float]] = None, algorithm: str = None, path: Optional[List[List[float]]] = None):
        """Visualize RRT tree with optional goal and path"""
        self.setup_plot()
        self.plot_tree(tree)
        
        # Plot start point as a blue cross
        plt.plot(qI[0], qI[1], 'bx', markersize=8, label='Start')
        
        # Plot goal point if provided
        if qG is not None:
            plt.plot(qG[0], qG[1], 'bo', markersize=5, label='Goal')
        
        # Plot path if provided
        # Plot path if provided and valid
        if path is not None:
            path = np.array(path)
            plt.plot(path[:, 0], path[:, 1], '-', color='blue', 
                    linewidth=1.5, label='Path', zorder=10)
        
         # Set title based on algorithm
        if algorithm == 'simple_RRT':
            plt.title('RRT exploration, neglecting obstacles')
        elif algorithm == 'RRT_with_obstacles':
            plt.title('RRT exploration, considering obstacles')
        elif algorithm == 'single_tree_search_RRT':
            plt.title('RRT Planning')

        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis('equal')
        plt.show()
    
    def visualize_prm(self, roadmap: AbstractRoadmapGraph, qI: List[float], 
                  qG: List[float], algorithm: str = None, path: Optional[List[List[float]]] = None):
        """Visualize PRM roadmap and path"""
        self.setup_plot()
        
        # Plot roadmap edges (thin black lines)
        for v_id, edges in roadmap.edges.items():
            v1 = roadmap.get_vertex_config(v_id)
            for neighbor_id, _ in edges:
                v2 = roadmap.get_vertex_config(neighbor_id)
                plt.plot([v1[0], v2[0]], [v1[1], v2[1]], 'k-', linewidth=0.5, alpha=0.5)
        
        # Plot roadmap vertices (small black dots)
        for config in roadmap.vertices.values():
            plt.plot(config[0], config[1], 'k.', markersize=2)
        
        # Plot start point (blue cross)
        plt.plot(qI[0], qI[1], 'bX', markersize=8, label='Start')
        
        # Plot goal point (blue circle)
        plt.plot(qG[0], qG[1], 'bo', markersize=5, label='Goal')
        
        # Plot path if provided and valid
        if path is not None and len(path) > 0:
            path = np.array(path)
            plt.plot(path[:, 0], path[:, 1], '-', color='blue', 
                    linewidth=1.5, label='Path', zorder=10)
        
        # Set title based on algorithm
        if algorithm == 'PRM':
            plt.title('PRM Planning')
        else:
            plt.title('Path Planning')
        
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis('equal')
        plt.grid(True)
        plt.show()