#I'M NOT SUPER SURE IF THIS ONE AVOIDS THE COLLISION
#In this example the obstacle is moving in the negative X direction. The obstacle is in y=0, so the car avoids the collision by changing it's trajectory
#So far i've only considered avoiding collision with the centre of the car, havent considered the entire car
import opengen as og
import casadi.casadi as cs
import numpy as np

from utils import *

# Cost parameters
Q = cs.diag(cs.SX([10., 10., 10, 10., 0., 10.]))
R = cs.diag(cs.SX([10., 10.]))
V = cs.diag(cs.SX([10., 10., 10., 10., 0., 10.]))

# Obstacle parameters (moving obstacle)
x_obs_initial = 80  # Initial x position of the obstacle
y_obs_initial = 0   # Initial y position of the obstacle
r_obs = 1        # Radius of the obstacle
obstacle_speed = -30  # Speed of the obstacle in the x-direction

def calc_cost(state, reference, u_i):
    """the cost function"""
    cost = cs.bilin(Q, (state - reference)) + cs.bilin(R, u_i)
    return cost

def moving_obstacle(t):
    """Define the position of the moving obstacle as a function of time."""
    # Simple linear motion in x-direction; constant speed along x
    x_obs = x_obs_initial + obstacle_speed * t
    y_obs = y_obs_initial  # Y position doesn't change, for simplicity
    return x_obs, y_obs

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

        # Get the moving obstacle position at the current time step
        t = i * DT  # Time at current step
        x_obs, y_obs = moving_obstacle(t)
        
        # Calculate penalty for collision with the moving obstacle
        dist_to_obstacle = (state[0] - x_obs)**2 + (state[1] - y_obs)**2  # squared distance to obstacle
        penalty = cs.fmax(0, r_obs**2 - dist_to_obstacle)  # Penalize if inside the obstacle
        obstacle_penalty.append(penalty)

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
