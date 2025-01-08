#For this solver we don't add a constraint for the obstacle, we add the obstacles directly to the cost function. IT WORKS (2 Static obstacles)
#So far i've only considered avoiding collision with the centre of the car, havent considered the entire car


import opengen as og
import casadi.casadi as cs
import numpy as np

from utils import *

# cost parameters
Q = cs.diag(cs.SX([10., 10., 10, 10., 0., 10.]))
R = cs.diag(cs.SX([10., 10.]))
V = cs.diag(cs.SX([10., 10., 10., 10., 0., 10.]))

# Circular obstacle parameters
x_obs1 = 50
y_obs1 = 0
r_obs1 = 0.5

x_obs2 = 15
y_obs2 = 0
r_obs2 = 4.0

def calc_cost(state, reference, u_i):
    """the cost function"""
    cost = cs.bilin(Q, (state - reference)) + cs.bilin(R, u_i)
    return cost

def main():
    u = cs.SX.sym('u', NU*N)
    z0 = cs.SX.sym('z0', (N+1)*NX)
    state, reference = z0[:NX], z0[NX:]
    
    cost = 0
    v_i = []
    phi_i = []
    acc_i = []
    alat_i = []
    obstacle_penalty = []

    for i in range(0, N):
        # Extract controls and reference
        u_i = u[i*NU:(i+1)*NU]
        ref_i = reference[i*NX:(i+1)*NX]
        
        # Update cost
        cost += calc_cost(state, ref_i, u_i)
        
        # State evolution (the kinematic model)
        state[0] += state[3] * cs.cos(state[2]) * DT
        state[1] += state[3] * cs.sin(state[2]) * DT
        state[2] += state[3] * cs.sin(state[4]) * DT
        state[3] += state[5] * DT
        state[4] += u_i[0] * DT
        state[5] += u_i[1] * DT

        v_i.append(state[3])
        phi_i.append(state[4])
        acc_i.append(state[5])
        alat_i.append(state[3] ** 2 / (L / (cs.sin(state[3]))))  # lateral acceleration

        # Obstacle avoidance constraints (penalties)
        dist_to_obstacle1 = (state[0] - x_obs1)**2 + (state[1] - y_obs1)**2  # squared distance to first obstacle
        penalty1 = cs.fmax(0, r_obs1**2 - dist_to_obstacle1)  # Penalize if inside the first obstacle
        dist_to_obstacle2 = (state[0] - x_obs2)**2 + (state[1] - y_obs2)**2  # squared distance to second obstacle
        penalty2 = cs.fmax(0, r_obs2**2 - dist_to_obstacle2)  # Penalize if inside the second obstacle

        # Combine the two penalties
        obstacle_penalty.append(penalty1 + penalty2)

    # Flatten lists to concatenate
    v_i = cs.vertcat(*v_i)
    phi_i = cs.vertcat(*phi_i)
    acc_i = cs.vertcat(*acc_i)
    alat_i = cs.vertcat(*alat_i)
    obstacle_penalty = cs.vertcat(*obstacle_penalty)

    # Add obstacle penalty to the cost
    cost += 1000 * cs.sum1(obstacle_penalty)  # Large weight to penalize violation of obstacle avoidance

    # Define limits and bounds for the problem
    v_lim = og.constraints.BallInf([5.]*N, 5.)      # velocity limits
    phi_lim = og.constraints.BallInf(None, 0.7)    # steering limit 
    acc_lim = og.constraints.BallInf(None, 4)       # acceleration limit
    alat_lim = og.constraints.BallInf(None, 4.)     # lateral acceleration limits

    bounds = og.constraints.Rectangle(UMIN, UMAX)

    # Define the optimization problem
    problem = og.builder.Problem(u, z0, cost) \
        .with_aug_lagrangian_constraints(phi_i, phi_lim) \
        .with_aug_lagrangian_constraints(acc_i, acc_lim) \
        .with_aug_lagrangian_constraints(alat_i, alat_lim) \
        .with_aug_lagrangian_constraints(v_i, v_lim) \
        .with_constraints(bounds)
    
    # Define configuration options for building and solving the problem
    build_config = og.config.BuildConfiguration()\
        .with_build_directory("my_optimizers")\
        .with_build_mode("debug")\
        .with_tcp_interface_config()
    
    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("navigation")
    
    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(1e-5)

    # Build the optimizer
    builder = og.builder.OpEnOptimizerBuilder(problem,
                                            meta,
                                            build_config,
                                            solver_config)
    builder.build()

if __name__ == "__main__":
    main()
