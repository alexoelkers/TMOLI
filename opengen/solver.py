import opengen as og
import casadi.casadi as cs
import numpy as np

from utils import *

# cost parameters
Q = cs.diag(cs.SX([10, 10, 0.1, 10])) # Q = 10
R = cs.diag(cs.SX([1, 10]))
V = cs.diag(cs.SX([200, 200, 2, 200]))

def calc_cost(state, reference, u_i):
    """the cost function"""
    cost = cs.bilin(Q, (state - reference)) + cs.bilin(R, u_i)
    return cost


def main():
    u = cs.SX.sym('u', NU*N)
    z0 = cs.SX.sym('z0', 2*NX)
    state, reference = z0[:NX], z0[NX:]
    # x, y, theta, v = state[0], state[1], state[2], state[3]
    # xref, yref, thetaref, vref = reference[0], reference[1], reference[2], reference[3]

    cost = 0
    v_i = []

    for i in range(0, NU*N, NU):
        # v_i[i] = v
        u_i = u[i:i+NU]
        cost += calc_cost(state, reference, u_i)
        state[0] += state[3] * cs.cos(state[2]) * DT
        state[1] += state[3] * cs.sin(state[2]) * DT
        state[2] += state[3] * cs.sin(u_i[0]) * DT
        # state[2] = (state[2] + cs.pi) % (2 * cs.pi) - cs.pi
        state[3] += u_i[1] * DT
        v_i.append(state[3])

    # v_i[N] = v # terminal velocity

    v_i = cs.vertcat(*v_i)
    set_c = og.constraints.BallInf([5]*(N), 5)
    cost += cs.bilin(V, (state - reference))

    bounds = og.constraints.Rectangle(UMIN, UMAX)

    problem = og.builder.Problem(u, z0, cost)\
        .with_aug_lagrangian_constraints(v_i, set_c)\
        .with_constraints(bounds)
    build_config = og.config.BuildConfiguration()\
        .with_build_directory("my_optimizers")\
        .with_build_mode("debug")\
        .with_tcp_interface_config()
    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("navigation")
    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(1e-5)

    cost

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                            meta,
                                            build_config,
                                            solver_config)
    builder.build()

if __name__ == "__main__":
    main()