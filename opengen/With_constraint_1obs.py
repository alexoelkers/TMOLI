#For this solver we add a constraint for the obstacle, and we don't add the obstacle directly to the cost function. DOESN'T WORK (1 Static obstacle)

import opengen as og
import casadi.casadi as cs
import numpy as np

from utils import *

# cost parameters
Q = cs.diag(cs.SX([10., 10., 10, 10., 0., 10.]))
R = cs.diag(cs.SX([10., 10.]))
V = cs.diag(cs.SX([10., 10., 10., 10., 0., 10.]))

# Obstacle parameters
x_obstacle = 50
y_obstacle = 0
obstacle_radius = 0.5

def calc_cost(state, reference, u_i):
    """the cost function"""
    cost = cs.bilin(Q, (state - reference)) + cs.bilin(R, u_i)
    return cost

def shape_obstacles(obstacles_unshaped):
    return cs.reshape(obstacles_unshaped, (N, OBS_N*2))


def main():
    u = cs.SX.sym('u', NU*N)
    z0 = cs.SX.sym('z0', (N+1)*NX + OBS_N*N*2)
    state, reference, obstacles_unshaped = z0[:NX], z0[NX:(N+1)*NX], z0[(N+1)*NX:]
    obstacles = shape_obstacles(obstacles_unshaped)
    
    cost = 0
    v_i = []
    phi_i = []
    acc_i = []
    alat_i = []
    obstacle_constraints = []

    for i in range(0, N):
        # v_i[i] = v
        u_i = u[i*NU:(i+1)*NU]
        ref_i = reference[i*NX:(i+1)*NX]
        cost += calc_cost(state, ref_i, u_i)
        state[0] += state[3] * cs.cos(state[2]) * DT
        state[1] += state[3] * cs.sin(state[2]) * DT
        state[2] += state[3] * cs.sin(state[4]) * DT
        state[3] += state[5] * DT
        state[4] += u_i[0] * DT
        state[5] += u_i[1] * DT

        v_i.append(state[3])
        phi_i.append(state[4])
        acc_i.append(state[5])
        alat_i.append(state[3] ** 2 / (L / (cs.sin(state[3])))) # lateral acceleration

        distance_squared = (state[0] - x_obstacle) ** 2 + (state[1] - y_obstacle) ** 2
        obstacle_constraints.append(cs.fmax(0.0, - distance_squared + 2* obstacle_radius))


        """ for obstacle_i in range(OBS_N):
            obstacle_x = obstacles[i, obstacle_i*2]
            obstacle_y = obstacles[i, obstacle_i*2 + 1]
            # Obstacle avoidance constraint for each time step
            distance_squared = (state[0] - obstacle_x) ** 2 + (state[1] - obstacle_y) ** 2
            # Negative if no collition, positive if collision. This means penalty approach can be used instead
            obstacle_constraints.append(cs.fmax(0.0, - distance_squared + 2* obstacle_radius))  # Must be < 0 """

    # Convert obstacle constraints to symbolic vector
    obstacle_constraints = cs.vertcat(*obstacle_constraints)

    v_i = cs.vertcat(*v_i)
    phi_i = cs.vertcat(*phi_i)
    acc_i = cs.vertcat(*acc_i)
    alat_i = cs.vertcat(*alat_i)

    # Constraints
    v_lim = og.constraints.BallInf([5.]*N, 5.)      # velocity limits
    phi_lim = og.constraints.BallInf(None, 0.7)    # steering limit 
    acc_lim = og.constraints.BallInf(None, 4)       # acceleration limit
    alat_lim = og.constraints.BallInf(None, 4.)     # lateral acceleration limits
    bounds = og.constraints.Rectangle(UMIN, UMAX)

    # Build the optimization problem with obstacle constraints
    problem = og.builder.Problem(u, z0, cost) \
        .with_aug_lagrangian_constraints(phi_i, phi_lim) \
        .with_aug_lagrangian_constraints(acc_i, acc_lim) \
        .with_aug_lagrangian_constraints(alat_i, alat_lim) \
        .with_aug_lagrangian_constraints(v_i, v_lim) \
        .with_penalty_constraints(obstacle_constraints) \
        .with_constraints(bounds)

    # Build configuration
    build_config = og.config.BuildConfiguration()\
        .with_build_directory("my_optimizers")\
        .with_build_mode("debug")\
        .with_tcp_interface_config()
    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("navigation_obstacle")
    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(1e-5)

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                            meta,
                                            build_config,
                                            solver_config)
    builder.build()

if __name__ == "__main__":
    main()
