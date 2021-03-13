This code can be used to determine the optimal set of 
parameters for given scoring functions used in the Amazon Picking
Challenge to determine the optimal strategy, through the use of an
abstract version of the original simulation and a genetic algorithm.

The program can can run through `main.py` which take the 
parameters ...

The functions as they will be used in the ROS simulation
are implemented in `simulation.py`, which closely
emulates the real simulation, but focuses only on time and completion
of tasks to increase the run-time speed.

The genetic algorithm is implemented in `genetic_algorithm.py`.

The parameters for the algorithm and the simulation are loaded
from the `setup.json` file.