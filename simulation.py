class Robot:
    """
    This class implements the robot within the simulation
    """
    def __init__(self, param, env):
        self.one = param["one"]
        self.two = param["two"]
        self.env = env
        self.pos = [0, 0]
        self.runtime = 0
        self.count = 0
        self.trophy_list = None
        self.trophy_truth = None

    def set_param(self, param):
        self.one = param["one"]
        self.two = param["two"]

    def set_trophies(self, trophy_list, trophy_truth):
        self.trophy_list = trophy_list
        self.trophy_truth = trophy_truth

    def reset(self):
        self.pos = [0, 0]
        self.runtime = 0
        self.count = 0

    def update_trophies(self, add=None, remove=None, change=None):
        if add is not None:
            self.trophy_list.append(add)
        if remove is not None:
            pass
        if change is not None:
            pass

    # Loops

    def loop_one(self):
        max_val = (0, None)
        for trophy in self.trophy_list:
            pos = trophy["pos"]
            deploy_time = self.deployment_time(pos)
            n_density = self.neighborhood_density(trophy)
            info_gain = self.information_gain(pos)
            score = self.score_one(deploy_time, n_density, info_gain)
            if score > max_val[0]:
                max_val = (score, trophy)

        return max_val[1]

    def loop_two(self):
        max_val = (0, None)
        for trophy in self.trophy_list:
            pos = trophy["pos"]
            deploy_time = self.deployment_time(pos)
            n_density = self.neighborhood_density(trophy)
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

    # Utilities

    def neighborhood_density(self, goal_trophy):
        scoring = self.env.get_neigh_score(goal_trophy[1])
        trophies = self.trophy_list
        occupancy = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ]
        for trophy in trophies:
            pos_diff = trophy["pos"][0] - goal_trophy["pos"][0]
            if -1 <= pos_diff >= 1:
                occupancy[trophy["pos"][1]][pos_diff + 1] = 1
        occupancy[goal_trophy[1]][goal_trophy[0]] = 0
        density = 0
        for score_l, occ_l in zip(scoring, occupancy):
            for score_p, occ_p in zip(score_l, occ_l):
                density += score_p * occ_p
        return density

    def deployment_time(self, pos):
        return self.env.get_time(pos)

    def information_gain(self, pos):
        return self.env.get_time(pos)

    def difficulty(self, coord):
        diff_score = (coord[0]/self.env.size["width"]) ^ 2 + (coord[1]/self.env.size["width"]) ^ 2
        return diff_score

    # Robot Actions

    def run(self):
        self.loop_one()
        return self.count

    def move(self, goal, time):
        self.pos = goal
        self.runtime += time


class Environment:
    """
    This class implements the environment within the simulation.
    """
    def __init__(self, times, info_gain, neigh_scores):
        self.times = times
        self.info_gain = info_gain
        self.neigh_scores = neigh_scores
        self.travel_time = 10
        self.turning_time = 40
        self.map = None
        self.size = {}

    def get_time(self, goal):
        return self.times[str(goal)]

    def get_info_gain(self, goal):
        return self.info_gain[str(goal)]

    def get_neigh_scores(self, level):
        return self.neigh_scores[str(level)]

    def travel_time_estimate(self, pos, goal):
        travel = self.travel_time * (goal - pos)
        turning = self.turning_time * (goal - pos)
        return max(travel, turning)
