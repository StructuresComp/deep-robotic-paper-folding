import numpy as np
from nfm import astar, FoldingPlanner
from nfm.utils import get_penalty_outline, float_tensor, get_numpy, ON_GPU

"""
This is the Lgb value of the original training set. 
We use this to non-dimensionalize the NFM. Should not be changed!
"""
Lgb_o = 0.0365

""" 
This is the Lgb value for the paper we are trying to generate a trajectory for.
We use this to re-dimensionalize the trajectory for the real world.

Lgb values used in our experiments
----------------------------------------
Origami   :  Lgb = 0.043
A4        :  Lgb = 0.048
US letter :  Lgb = 0.060
Cardboard :  Lgb = 0.132
"""
Lgb_c = 0.0480
Lgb_n = 0.0380

fp = FoldingPlanner(Lgb_o, Lgb_n)

# Discretization
delta_r = 0.002
delta = delta_r / Lgb_o

# Suspended length penalty region (we used 0.041 for origami paper)
ls_penalty_r = 0.035

# Penalty to disallow gripper going below a certain z-point. Can be None
height_penalty = 0.0125 / Lgb_o
# height_penalty = None

# Change your start and goal positions
start = (0.01 / Lgb_n, 0.03 / Lgb_n)
goal = (0.287 / Lgb_n, 0.03 / Lgb_n)

start_grid = (round(start[0] / delta), round(start[1] / delta))
goal_grid = (round(goal[0] / delta), round(goal[1] / delta))

manifold, manifold_with_penalty, penalty_boundary = fp.construct_discretized_manifold(discretization=delta_r,
                                                                                      ls_threshold=ls_penalty_r)

# Generate global optimal trajectory.
# Not using a heuristic. A* then degenerates to uniform cost search (UCS).
print("Starting path...")
path = astar(manifold_with_penalty, start_grid, goal_grid, delta, height_penalty,
             boundary=fp.get_folding_workspace(), len_penalty=0)

if path is None:
    print("Failed to generate a trajectory")
    exit(1)

path = np.array(path).reshape((-1, 2)) * delta
print("Path length: ", len(path))

# Perform path smoothing
path = fp.smooth_trajectory(path, num_points=240, weight_data=0.02, weight_smooth=0.98)

error_path = path[:, :2] * Lgb_n / Lgb_c
# error_path = path[:, :2] / Lgb_o

inp = float_tensor(fp._nn.preprocess_input(error_path * Lgb_o))[0]
all_outputs = get_numpy(fp._nn(inp))
ft_fn = np.abs(all_outputs[:, 1] / all_outputs[:, 2])
print(ft_fn.max())

# Plot trajectory along the non-dimensionalized NFM
fp.plot_trajectory(start, goal, error_path, penalty_boundary=penalty_boundary, discretization=0.00033)
