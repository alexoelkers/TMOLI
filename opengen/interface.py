import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

from utils import *
import splinterp as sp


SIMTIME = 30    # maximum allowable simtime
T = int(SIMTIME / DT) # simulation period [n]

def car_ode(x, u):
    x, y, theta, v, phi, a = x
    x += v * np.cos(theta) * DT
    y += v * np.sin(theta) * DT
    theta += v * np.sin(phi) * DT
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    v += a * DT
    phi += u[0] * DT
    a += u[1] * DT
    return np.array([x, y, theta, v, phi, a])


def main():
    # Use TCP server
    # ------------------------------------
    mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation', port=12345)
    mng.start()

    mng.ping()

    x = np.array([0, 0, 0, 4, 0, 0])
    u_history = []
    x_history = []

    for t in np.arange(0, T*DT, DT):
        goal = sp.generate_guide_trajectory(x)
        solution = mng.call([*x, *goal], initial_guess=[0.0] * (NU*N))
        # print(f"connection success")
        if solution.is_ok():
            u = solution.get().solution[:NU]
        else:
            err = solution.get()
            raise(ValueError(err.message))
        x_history.append(x)
        u_history.append(u)
        x = car_ode(x, u)    

    # close TCP connection
    mng.kill()

    x_history = np.array(x_history)
    u_history = np.array(u_history)

    fig1, ax1 = plt.subplots()
    ax1.plot(np.arange(0, T*DT, DT), x_history[:, 1:])
    # ax1.hlines(goal, 0, T*DT)

    fig2, ax2 = plt.subplots()
    ax2.plot(x_history[:,0], x_history[:,1])
    ax2.set(aspect="equal")

    fig3, ax3 = plt.subplots()
    ax3.plot(np.arange(0, T*DT, DT), x_history[:, 3] ** 2 / (L / (np.sin(x_history[:, 4]))))

    plt.show()

if __name__ == "__main__":
    main()
