# TMOLI
Group 42 Planning and Decision Making project repository. 

# Prerequisites
We recommend that you run this project in a Python virtual environment
 - Python 3
 - Rust
 - numpy
 - pygame
 - MatPlotLib
 - OpEn (`pip install opengen`) 

# Instructions
To compile the solver, run the file `opengen/solver.py`. This will create the solver in a my_optimizers directory. To run the file, either run a individual sample run be running the opengen/interface.py file or one of the test cases beginning with test*.py. Running the test cases will also recompile the solver, though this is not necessary.

To enable or disable the "in-front" penalty, uncomment lines 24 and 25 in `solver.py`

To create animations, first run `interface.py` with the obstacles defined in the function `get_obstacle_definition()`. When this has finished, close all graphs (dont quit using Ctrl+C) and the run the file `import pygame.py` Note that the simulation only shows x > 70. 

# Git repo
https://github.com/alexoelkers/TMOLI
