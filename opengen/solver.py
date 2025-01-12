import opengen as og
import casadi.casadi as cs
import numpy as np

from constants import *

# cost parameters
# [x-pos, y-pos, psi, v, angular velocity]
Q = 0.25*cs.diag(cs.SX([5., 5., 5., 0.25, 10.]))
R = 0.5*cs.diag(cs.SX([5., 5.])) #steering_angle, acceleration

# Obstacle parameters
x_obstacle = 50
y_obstacle = 0
obstacle_radius = 0.5

def calc_cost(state, reference, u_i, obstacles_unshaped):
    """the cost function"""
    cost = cs.bilin(Q, (state - reference)) + cs.bilin(R, u_i)
    #Add the "no-infront cost" and a repulsive field around the obstacles
    for i in range(OBS_N):
        obstacle_x = obstacles_unshaped[i + (i * 2 * OBS_N)]
        obstacle_y = obstacles_unshaped[1 + i + (i * 2 * OBS_N)]
        infront_cost = INFRONT_MAXCOST * cs.fmax(0, INFRONT_DISTANCE - cs.fmax(obstacle_x - state[0],0))
        cost += cs.sign(cs.fmax(0, 2*obstacle_radius - cs.fabs(state[1] - obstacle_y))) * infront_cost

        #Repulsive field
        cost += cs.fmax(MAX_COST - cs.sqrt((state[1] - obstacle_y)**2 + (obstacle_x - state[0])**2), 0)
    return cost

def shape_obstacles(obstacles_unshaped):
    return cs.reshape(obstacles_unshaped, (N, OBS_N*2))


def main():
    u = cs.SX.sym('u', NU*N)
    z0 = cs.SX.sym('z0', (N+1)*NX + OBS_N*N*2)
    state, reference, obstacles_unshaped = z0[:NX], z0[NX:(N+1)*NX], z0[(N+1)*NX:]
    cost = 0
    constraints = []

    for i in range(0, N):
        #Calculate the cost and update the states
        u_i = u[i*NU:(i+1)*NU]
        ref_i = reference[i*NX:(i+1)*NX]
        cost += calc_cost(state, ref_i, u_i, obstacles_unshaped)
        state[0] += state[3] * cs.cos(state[2]) * DT
        state[1] += state[3] * cs.sin(state[2]) * DT
        state[2] += state[3] * cs.sin(state[4]) * DT
        state[3] += u_i[1] * DT
        state[4] += u_i[0] * DT
        
        #Constrains the absolute values of the physical properties
        constraints.append(cs.fmax(state[3] - V_MAX, 0.0))
        constraints.append(cs.fmax(cs.fabs(state[4]) - STEERING_MAX, 0.0))
        lateral_acc = cs.tan(state[4]) * state[3] ** 2 / L
        constraints.append(cs.fmax(cs.fabs(lateral_acc) - LATERAL_ACC_MAX, 0.0))
        
        #Obstacle constraints, loop through each obstacle
        for obstacle_i in range(OBS_N):
            obstacle_x = obstacles_unshaped[obstacle_i * 2 + (i * 2 * OBS_N)] # obstacles[i, obstacle_i*2]
            obstacle_y = obstacles_unshaped[1 + obstacle_i * 2 + (i * 2 * OBS_N)] # obstacles[i, obstacle_i*2 + 1]
            # Obstacle avoidance constraint for each time step
            distance_squared = (state[0] - obstacle_x) ** 2 + (state[1] - obstacle_y) ** 2
            # Negative if no collition, positive if collision. This means penalty approach can be used instead
            constraints.append(cs.fmax(0.0, - cs.sqrt(distance_squared) + 2 * obstacle_radius))  # Must be < 0

    # Convert obstacle constraints to symbolic vector
    constraints = cs.vertcat(*constraints)
    bounds = og.constraints.Rectangle(UMIN, UMAX)
    
    # Build the optimization problem with obstacle constraints
    problem = og.builder.Problem(u, z0, cost) \
        .with_constraints(bounds) \
        .with_penalty_constraints(constraints) \

    # Build configuration
    build_config = og.config.BuildConfiguration()\
        .with_build_directory("my_optimizers")\
        .with_build_mode("debug")\
        .with_tcp_interface_config()
    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("navigation_obstacle")
    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(5e-5) \
        .with_delta_tolerance(1e-6) \
        .with_max_outer_iterations(100) \
        .with_initial_penalty(1)

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                            meta,
                                            build_config,
                                            solver_config)
    builder.build()

if __name__ == "__main__":
    main()
