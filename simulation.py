class Robot:
    """
    This class implements the robot within the simulation
    """
    def __init__(self, param, env):
        self.one = param["one"]
        self.two = param["two"]
        self.three = param["three"]
        self.four = param["four"]
        self.env = env

    def set_param(self, param):
        self.one = param["one"]
        self.two = param["two"]
        self.three = param["three"]
        self.four = param["four"]

    # Loops

    def loop_one(self):
        max_val = (0, None)
        for trophy in self.env.trophy_list:
            pos = trophy["pos"]
            deploy_time = self.deployment_time(pos)
            n_density = self.neighborhood_density(pos)
            info_gain = self.information_gain(pos)
            score = self.score_one(deploy_time, n_density, info_gain)
            if score > max_val[0]:
                max_val = (score, trophy)

        return max_val[1]

    def loop_two(self):
        max_val = (0, None)
        for trophy in self.env.trophy_list:
            pos = trophy["pos"]
            deploy_time = self.deployment_time(pos)
            n_density = self.neighborhood_density(pos)
            diff = self.difficulty(trophy["coord"])
            score = self.score_one(deploy_time, n_density, diff)
            if score > max_val[0]:
                max_val = (score, trophy)

        return max_val[1]

    # Scoring Functions

    def score_one(self, deploy_time, n_density, info_gain):
        """
        This function calculates the score for a single trophy during the first trip of the robot.
        :param deploy_time: The time it takes the robot to get to the robot
        :param n_density: The density of trophies around the goal trophy
        :param info_gain: The gain in information on the trophies in general during the trip
        :return: The trophy score
        """
        return self.one[0] * deploy_time + self.one[1] * n_density + self.one[2] * info_gain

    def score_two(self, deploy_time, n_density, diff):
        """
        This function calculates the score for a single trophy during the trips after the initial exploratory trip.
        :param deploy_time: The time it takes the robot to get to the robot
        :param n_density: The density of trophies around the goal trophy
        :param diff: The difficulty to grip the trophy
        :return: The trophy score
        """
        return self.two[0] * deploy_time + self.two[1] * n_density + self.two[2] * diff

    def score_three(self):
        pass

    def score_four(self):
        pass

    # Utilities

    def neighborhood_density(self, pos):
        return 0

    def deployment_time(self, pos):
        return self.env.get_time(pos)

    def information_gain(self, pos):
        return 0

    def difficulty(self, coord):
        diff_score = (coord[0]/self.env.size["width"]) ^ 2 + (coord[1]/self.env.size["width"]) ^ 2
        return diff_score


class Environment:
    """
    This class implements the environment within the simulation.
    """
    def __init__(self, times):
        self.times = times
        self.map = None
        self.size = {}
        self.trophy_list = None

    def get_time(self, goal):
        return self.times[goal]

    def delete_trophy(self):
        pass

    def add_trophy(self):
        pass
