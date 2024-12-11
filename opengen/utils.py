# problem parameters
NU  = 2     # nu = number of inputs
NX  = 4     # nx = number of state variables
N   = 20    # N = MPC horizon length
DT  = 0.1  # dt = discrete time step size
UMIN = [-0.8, -0.8] * N
UMAX = [0.8, 0.8] * N