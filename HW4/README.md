# Path Planning with RRT and PRM

The project consists of several Python files:

- `main.py`: Main entry point that handles command-line arguments and runs the algorithms
- `abstract_base.py`: Contains abstract base classes defining interfaces for Graph and Edge

## Class Overview

- **AbstractEdge**: 
  - Start and end configurations.
  - Calculate the traversal cost.
  - Find intermediate points along the edge.

- **AbstractGraph**: 
  - Add vertices and edges.
  - Vertex configurations and neighbors.
  - Obtain paths based on vertex sequences.

- **AbstractTreeGraph**: Extends `AbstractGraph` for tree structures, adding:
  - Methods to retrieve paths to the root.
  - Parent relationships between vertices.
  - Useful for tree-based algorithms like RRT.

- **AbstractRoadmapGraph**: Extends `AbstractGraph` for roadmap-based graphs with methods to:
  - Connect vertices to their nearest neighbors.
  - Find paths between vertices, as used in PRM.

- `graph_impl.py`: Implements concrete Graph and Edge classes as Tree and Edge

## Classes

### Edge
The `Edge` class represents an edge in the graph:
- **Initialization**: Takes `start_config` and `end_config` as lists of coordinates, storing the edge’s endpoints.
- **Methods**:
  - `_compute_cost`: Calculates the Euclidean distance as the traversal cost.
  - `get_cost`: Returns the traversal cost.
  - `get_config_at`: Returns a configuration at a given `t` value between `start_config` and `end_config`.
- **Properties**:
  - `start_config`: The starting configuration.
  - `end_config`: The ending configuration.

### TreeGraph
The `TreeGraph` class builds on `AbstractTreeGraph` and represents a tree data structure with a single root node and parent-child relationships:
- **Initialization**: Creates empty dictionaries for vertices, edges, and parent relationships.
- **Methods**:
  - `add_vertex`: Adds a vertex and returns its ID.
  - `add_vertex_with_parent`: Adds a vertex with a parent, creating an edge to its parent.
  - `add_edge`: Adds an undirected edge between two vertices.
  - `remove_edge`: Removes the edge between two vertices, updating parent relationships as needed.
  - `get_path_to_root`: Retrieves the path from a vertex to the root.
- **Properties**:
  - `vertices`: Returns the vertices in the tree.
  - `edges`: Returns edges of each vertex.
  - `root_id`: Returns the root vertex ID.
  - `parent`: Returns the parent relationships of vertices.

### RoadmapGraph
The `RoadmapGraph` class implements a general graph structure suitable for PRM-like applications:
- **Initialization**: Creates empty dictionaries for vertices and edges.
- **Methods**:
  - `add_vertex`: Adds a vertex and returns its ID.
  - `add_edge`: Adds an undirected edge between two vertices.
  - `connect_to_neighbors`: Connects a vertex to its `k` nearest neighbors if a valid path exists.
  - `find_path`: Uses Breadth-First Search (BFS) to find a path between two vertices.
- **Properties**:
  - `vertices`: Returns the dictionary of vertices.
  - `edges`: Returns the dictionary of edges for each vertex.
  
- `planners.py`: Contains the implementation of planning algorithms (RRT and PRM) based on the refrence

- `input.json`: Configuration file containing environment setup

### File Details

#### input.json
Contains the configuration space parameters

#### Planner Classes
- `RRT`: Implements RRT algorithm with two main methods:
  - `explore()`: Basic RRT exploration
  - `plan_to_goal()`: RRT with goal biasing
- `PRM`: Implements PRM algorithm for path planning

## Setup
1. Place all files in the same directory

## Usage
The program can be run with different algorithms and parameters using command-line arguments.

### Command Line Arguments
- `-dt`: Time step parameter (default: 0.02)
- `-algorithm`: Algorithm choice (required)
- `-step_size`: Step size for extension (default: 0.1)
- `-n_iterations`: Number of iterations for RRT (default: 1000)
- `-p`: Goal sampling probability for RRT (default: 0.1)
- `-N`: Number of nodes for PRM (default: 1000)
- `-K`: Number of nearest neighbors for PRM (default: 15)

### Running Different Algorithms

1. Problem 1a: RRT Exploration without Obstacles
```bash
python main.py -algorithm simple_RRT -dt 0.02 -step_size 0.1 -n_iterations 1000
```

2. Problem 1b: RRT Exploration with Obstacles
```bash
python main.py -algorithm RRT_with_obstacles -dt 0.02 -step_size 0.1 -n_iterations 1000
```

3. Problem 2: Single-Tree Search RRT
```bash
python main.py -algorithm single_tree_search_RRT -dt 0.02 -step_size 0.1 -n_iterations 1000 -p 0.1
```

4. Problem 3: PRM
```bash
python main.py -algorithm PRM -dt 0.02 -step_size 0.1 -N 1000 -K 15
```

### Examples with Different Parameters

Running RRT with more iterations:
```bash
python main.py -algorithm simple_RRT -dt 0.02 -step_size 0.1 -n_iterations 2000
```

Running PRM with more nodes:
```bash
python main.py -algorithm PRM -dt 0.02 -step_size 0.1 -N 2000 -K 20
```
