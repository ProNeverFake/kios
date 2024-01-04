# Overview

Framework for learning Behavior Trees (BTs) based on Genetic Programming (refer to [this paper](https://ieeexplore.ieee.org/abstract/document/9562088/)) and Learning from Demonstration (refer to [this paper](https://ieeexplore.ieee.org/abstract/document/9900603)).  
The target tasks are robotic manipulation ones and are simulated in the AGX Dynamics from [Algoryx](file:///opt/Algoryx/AGX-2.34.0.1/doc/main/index.html).

## Experiments

* The [GUI](./simulation/simulation/algoryx/combined/gui.py) allows user to add demonstrations and start/stop/resume the evolution of BTs through the genetic programming.
* The experiments are defined by uncommenting the corresopnding lines in the [config file](./simulation/simulation/algoryx/config/gp_targets.yaml). Morover, adjust the [parameters](./simulation/simulation/algoryx/config/sim_data.yaml) as explained in the paper.
* For all experiments but the last, set [`random_bringup`](./simulation/simulation/algoryx/config/sim_data.yaml#L32) to `False`.
* The data with which the plots for the paper have been obtained are provided in the [log folder](./simulation/simulation/algoryx/logs).


## Disclaimer
To run the experiment is necessary to have a valid license and installation of the simulator from Algoryx. Please contact the authors of the paper for further instructions.


