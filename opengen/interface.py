import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import time

from opengen_a3 import nu, N, dt

T = 300

def car_ode(x, u, dt):
    return x + np.array([x[1], u]) * dt

# Use TCP server
# ------------------------------------
mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation', port=12345)
mng.start()

mng.ping()

x = np.array([0, 0])
u_history = []
x_history = []

start = time.process_time()
for t in range(T):
    solution = mng.call(x, initial_guess=[1.0] * (nu*N))
    # print(f"connection success")
    if solution.is_ok():
        u = solution.get().solution[0]
    else:
        err = solution.get()
        raise(ValueError(err.message))
    x_history.append(x)
    u_history.append(u)
    x = car_ode(x, u, dt)

stop = time.process_time()
print(stop - start)
    

mng.kill()

fig, ax = plt.subplots(2)

ax[0].plot(np.arange(0, T*dt, dt), u_history)
ax[1].plot(np.arange(0, T*dt, dt), x_history)

plt.show()
