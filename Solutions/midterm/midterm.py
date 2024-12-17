from planning import (
    rrt,
    StraightEdgeCreator,
    EuclideanDistanceComputator,
    ObstacleCollisionChecker,
)
from obstacle import WorkspaceObstacle, WorkspaceBoundary, CSpaceObstacle


class CircularMobileRobotPathPlanning:
    def __init__(
        self, robot_radius: float, Xmax: float, Ymax: float, circular_obstacles: list
    ):
        # Define CSpace
        self.cspace = [(0, Xmax), (0, Ymax)]

        # Construct the obstacles and collision checker
        boundary = WorkspaceBoundary(0, Xmax, 0, Ymax)
        obstacles = [
            CSpaceObstacle(WorkspaceObstacle(obs), robot_radius)
            for obs in circular_obstacles
        ]
        obstacles.append(CSpaceObstacle(boundary, robot_radius))
        self.collision_checker = ObstacleCollisionChecker(obstacles)

        # Define edge creator and distance computator
        self.edge_creator = StraightEdgeCreator(0.1)
        self.distance_computator = EuclideanDistanceComputator()

    def plan(self, qI: tuple, qG: tuple):
        # Run RRT and compute the path from root to goal
        qIp = (qI[0], qI[1])
        qGp = (qG[0], qG[1])

        (G, root, goal) = rrt(
            cspace=self.cspace,
            qI=qIp,
            qG=qGp,
            edge_creator=self.edge_creator,
            distance_computator=self.distance_computator,
            collision_checker=self.collision_checker,
            pG=0.1,
            numIt=10000,
        )
        path = []
        if goal is not None:
            path = G.get_path(root, goal)

        return (G, path)
