import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

from utils import *

# position reference parameters
XREF = 20
YREF = 20
THETAREF = 0
VREF = 0

REF = [XREF, YREF, THETAREF, VREF]

T = 300 # simulation period [n]

def car_ode(x, u):
    x, y, theta, v = x
    x += v * np.cos(theta) * DT
    y += v * np.sin(theta) * DT
    theta += v * np.sin(u[0]) * DT
    v += u[1] * DT
    return np.array([x, y, theta, v])


def main():
    # Use TCP server
    # ------------------------------------
    mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation', port=12345)
    mng.start()

    mng.ping()

    x = np.array([0, 0, 0, 5])
    u_history = []
    x_history = []

    for t in range(T):
        solution = mng.call([*x, *REF], initial_guess=[1.0] * (NU*N))
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

    fig, ax = plt.subplots(2)

    ax[0].plot(np.arange(0, T*DT, DT), u_history)
    ax[1].plot(x_history[:,0], x_history[:,1])

    plt.show()

if __name__ == "__main__":
    main()
