import opengen as og
import casadi.casadi as cs
import numpy as np

from utils import *

# cost parameters
Q = 10
R = 0.01
V = 0.

def cost():
    """the cost function"""
    pass


def main():
    u = cs.SX.sym('u', NU*N)
    z0 = cs.SX.sym('z0', 2*NX)
    x, v, xref, vref = z0[0], z0[1], z0[2], z0[3]

    cost = 0
    v_i = []

    for i in range(0, NU*N, NU):
        # v_i[i] = v
        u_i = u[i:i+NU] # this was originally a magic number, so if shit breaks, check here first
        # cost += quadform(Q, (z0 - xref)) + quadform(R, u_t)
        cost += Q * (x - xref) ** 2 + Q * (v - vref) ** 2 + R * cs.dot(u_i, u_i)
        x += v * DT
        v += u_i
        v_i.append(v)

    # v_i[N] = v # terminal velocity

    v_i = cs.vertcat(*v_i)
    set_c = og.constraints.BallInf([5]*(N), 5)
    cost += V * (x - xref) ** 2 + V * (v - vref) ** 2

    umin = [-4.0] * (NU*N)
    umax = [4.0] * (NU*N)
    bounds = og.constraints.Rectangle(umin, umax)

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