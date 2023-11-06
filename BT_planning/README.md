# Combining planning and learning of behavior trees for robotic assembly

This repository contains an implementation of a Genetic Programming (GP) algorithm that evolves Behavior Trees (BTs) to solve different assembly tasks. The tasks consist in picking LEGO DUPLO bricks and placing them in different configurations. The BTs are executed in the AGX Dynamics environment, but they can also be simulated in a high-level state machine without involving physics. The GP is bootstrapped by a BT created by a planner and partially solving the task (depending on the initial configurations).

## Set-up
Put BT_SETTINGS.yaml file(s) with whatever name(s) the scripts use in your ros2 workspace root folder in order for the scripts to find them properly with relative paths.

Build with
"colcon build --packages-select behavior_tree_learning --symlink-install"

## Code coverage
To run code coverage, in consol run
"pytest --cov"

## How to run the simulation
To run duplo simulation start an agx environment console and run
python agx_<script_name>.py
And then in a py trees console run
ros2 run behavior_tree_learning <script>
where <script> is whatever script listed in setup.py that you want to run




