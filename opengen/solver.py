import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

(nu, nx, N, L, dt) = (1, 2, 20, 0.5, 0.25)
# xref = np.array([150, 0])
# (xref, vref) = (150, 0)
# (Q, R, V) = 1, 0.2, 2
#Q = np.eye(nx)
#R = np.eye(nu)
#V = np.eye(nx)
# (q, qtheta, r, qN, qthetaN) = (10, 0.1, 1, 200, 2)

# def quadform(A, x):
    # """implements x.T @ A @ x"""
    # return cs.dot(x.T, cs.dot(A, x))

# A = np.eye(2)
# b = np.array([1, -1])
# print(quadform(A, b))

def main():
    (nu, nx, N, L, dt) = (1, 2, 20, 0.5, 0.25)
    # xref = np.array([150, 0])
    (xref, vref) = (150, 0)
    (Q, R, V) = 10, 0.01, 0.
    #Q = np.eye(nx)
    #R = np.eye(nu)
    #V = np.eye(nx)
    (q, qtheta, r, qN, qthetaN) = (10, 0.1, 1, 200, 2)

    u = cs.SX.sym('u', nu*N)
    z0 = cs.SX.sym('z0', nx)
    (x, v) = (z0[0], z0[1])

    cost = 0
    v_hist = []
    for t in range(0, nu*N, nu):
        u_t = u[t:t+nu] # this was originally a magic number, so if shit breaks, check here first
        # cost += quadform(Q, (z0 - xref)) + quadform(R, u_t)
        cost += Q * (x - xref) ** 2 + Q * (v - vref) ** 2 + R * cs.dot(u_t, u_t)
        x += v * dt
        v += u_t[0]
        v_hist.append(v)

    v_hist = cs.vertcat(*v_hist)
    set_c = og.constraints.BallInf([5]*(nu*N), 5)
    cost += V * (x - xref) ** 2 + V * (v - vref) ** 2

    umin = [-4.0] * (nu*N)
    umax = [4.0] * (nu*N)
    bounds = og.constraints.Rectangle(umin, umax)

    problem = og.builder.Problem(u, z0, cost)\
        .with_aug_lagrangian_constraints(v_hist, set_c)\
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