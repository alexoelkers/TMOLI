import opengen as og
import casadi.casadi as cs
import numpy as np

from utils import *

# cost parameters
Q = cs.diag(cs.SX([10., 10., 10, 10., 0., 10.]))
R = cs.diag(cs.SX([10., 10.]))
V = cs.diag(cs.SX([10., 10., 10., 10., 0., 10.]))

def calc_cost(state, reference, u_i):
    """the cost function for the MPC, the value of this funciton is the 
    minimisation objective"""
    cost = cs.bilin(Q, (state - reference)) + cs.bilin(R, u_i)
    return cost


def main():
    """MPC control loop
    
    This function takes no inputs, instead it compiles an LP problem, which can be accessed by 
    calling a rust TCP server. Per conventional MPC theory, the solver will try to minimise
    the cost of an objective function, while staying within all constraints.

    Key call info:
    --------------
    u: a vector of all inputs to the system over the solution horizon (optimisation 
       variable)
    z0: the input vector of the MPC contains:
        current state of the vehicle (size NX)
        target trajectory of the vehicle over the solution horizon (size N*NX)
    """
    u = cs.SX.sym('u', NU*N)
    z0 = cs.SX.sym('z0', (N+1)*NX)
    state, reference = z0[:NX], z0[NX:]
    
    cost = 0
    v_i = []
    phi_i = []
    acc_i = []
    alat_i = []

    for i in range(0, N):
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
        alat_i.append(state[3] ** 2 / (L / (cs.sin(state[4])))) # lateral acceleration

    # cost += cs.bilin(V, (state - reference[])) # terminal cost

    v_i = cs.vertcat(*v_i)
    phi_i = cs.vertcat(*phi_i)
    acc_i = cs.vertcat(*acc_i)
    alat_i = cs.vertcat(*alat_i)

    v_lim = og.constraints.BallInf([5.]*N, 5.)      # velocity limits
    phi_lim = og.constraints.BallInf(None, 0.7)    # steering limit 
    acc_lim = og.constraints.BallInf(None, 4)       # acceleration limit
    alat_lim = og.constraints.BallInf(None, 4.)     # latteral acceleration limits

    bounds = og.constraints.Rectangle(UMIN, UMAX)

    problem = og.builder.Problem(u, z0, cost) \
        .with_aug_lagrangian_constraints(phi_i, phi_lim) \
        .with_aug_lagrangian_constraints(acc_i, acc_lim) \
        .with_aug_lagrangian_constraints(alat_i, alat_lim) \
        .with_aug_lagrangian_constraints(v_i, v_lim) \
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