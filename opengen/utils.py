# problem parameters
NU  = 2     # nu = number of inputs
NX  = 6     # nx = number of state variables
N   = 20    # N = MPC horizon length
DT  = 0.1  # dt = discrete time step size
UMIN = [-100, -0.5] * N
UMAX = [100, 0.5] * N

# car parameters
L = 1   # car length [m]

K = 10  # turn radius
VG = 4  # vehicle goal velocity

XG = 100    # goal x position
OMEGAG = VG / K # goal angular velocity
Y0 = 0  # starting y coordinate