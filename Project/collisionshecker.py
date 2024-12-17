class Obstacle:
    def is_in_collision(self, robot_x, robot_y, robot_r):
        """Return whether the robot at the given configuration is in collision with the obstacle"""
        raise NotImplementedError


class WorkspaceObstacle(Obstacle):
    """A class for representing a circular obstacle"""

    def __init__(self, circular_obstacle):
        self.x = circular_obstacle.x
        self.y = circular_obstacle.y
        self.r = circular_obstacle.r

    def is_in_collision(self, robot_x, robot_y, robot_r):
        return (robot_x - self.x) ** 2 + (robot_y - self.y) ** 2 <= (
            robot_r + self.r
        ) ** 2


class WorkspaceBoundary(Obstacle):
    """A class for representing the workspace boundary"""

    def __init__(self, Xmin, Xmax, Ymin, Ymax):
        self.Xmin = Xmin
        self.Xmax = Xmax
        self.Ymin = Ymin
        self.Ymax = Ymax

    def is_in_collision(self, robot_x, robot_y, robot_r):
        return (
            robot_x < self.Xmin + robot_r
            or robot_x > self.Xmax - robot_r
            or robot_y < self.Ymin + robot_r
            or robot_y > self.Ymax - robot_r
        )


class CSpaceObstacle:
    def __init__(self, obs, robot_radius):
        self.obs = obs
        self.robot_radius = robot_radius

    def contain(self, config):
        """Return whether a configuration config = (x, y) of the robot hits the obstacle"""
        return self.obs.is_in_collision(config[0], config[1], self.robot_radius)
