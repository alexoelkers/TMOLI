import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

from utils import *

# position reference parameters
XREF = 150
VREF = 0

T = 300 # simulation period [s]

def car_ode(x, u, dt):
    return x + np.array([x[1], u]) * dt


def main():
    # Use TCP server
    # ------------------------------------
    mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation', port=12345)
    mng.start()

    mng.ping()

    x = np.array([0, 0])
    u_history = []
    x_history = []

    for t in range(T):
        solution = mng.call([*x, XREF, VREF], initial_guess=[1.0] * (NU*N))
        # print(f"connection success")
        if solution.is_ok():
            u = solution.get().solution[0]
        else:
            err = solution.get()
            raise(ValueError(err.message))
        x_history.append(x)
        u_history.append(u)
        x = car_ode(x, u, DT)        

    # close TCP connection
    mng.kill()

    fig, ax = plt.subplots(2)

    ax[0].plot(np.arange(0, T*DT, DT), u_history)
    ax[1].plot(np.arange(0, T*DT, DT), x_history)

    plt.show()

if __name__ == "__main__":
    main()
