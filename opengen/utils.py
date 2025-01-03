# problem parameters
NU  = 2     # nu = number of inputs
NX  = 6     # nx = number of state variables
N   = 20    # N = MPC horizon length
DT  = 0.05  # dt = discrete time step size
UMIN = [-100, -0.5] * N
UMAX = [100, 0.5] * N

# car parameters
L = 1   # car length [m]