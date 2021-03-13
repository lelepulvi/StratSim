from simulation import Robot, Environment
import json


def run():
    with open("setup.json", "r") as file:
        setup = json.load(file)
    env = Environment(setup["environment_parameters"]["times"])
    initial_param = {
        "one": [0.333, 0.333, 0.333],
        "two": [0.333, 0.333, 0.333],
        "three": [0.333, 0.333, 0.333],
        "four": [0.333, 0.333, 0.333]
    }
    robot = Robot(initial_param, env)
