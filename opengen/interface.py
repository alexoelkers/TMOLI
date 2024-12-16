import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

from utils import *

# position reference parameters
XREF = 150
YREF = 0
THETAREF = 0
VREF = 10

GOAL_QUEUE = [[150, 0, 0, 10], [100, 150, np.pi/2, 10]]

T = 300 # simulation period [n]

def car_ode(x, u):
    x, y, theta, v = x
    x += v * np.cos(theta) * DT
    y += v * np.sin(theta) * DT
    theta += v * np.sin(u[0]) * DT
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    v += u[1] * DT
    return np.array([x, y, theta, v])


def main():
    # Use TCP server
    # ------------------------------------
    mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation', port=12345)
    mng.start()

    mng.ping()

    x = np.array([0, 0, 0, 0])
    u_history = []
    x_history = []

    goal = GOAL_QUEUE.pop(0)

    for t in np.arange(0, T*DT, DT):
        if t == 15:
            goal = GOAL_QUEUE.pop(0)
            print(f"goal updated")
        solution = mng.call([*x, *goal], initial_guess=[1.0] * (NU*N))
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

    fig, ax = plt.subplots()

    # ax[0].plot(np.arange(0, T*DT, DT), u_history)
    # ax.plot(np.arange(0, T*DT, DT), x_history)
    ax.plot(x_history[:,0], x_history[:,1])
    # ax.hlines(goal, 0, T*DT)
    ax.set(aspect="equal")

    plt.show()

if __name__ == "__main__":
    main()
