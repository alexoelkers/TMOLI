# problem parameters
NU  = 2     # nu = number of inputs
NX  = 5     # nx = number of state variables
N   = 25    # N = MPC horizon length
DT  = 0.05  # dt = discrete time step size
UMIN = [-100, -2] * N
UMAX = [100, 2] * N

# Simulation parameters
SIMTIME = 30  # Maximum allowable simulation time
T = int(SIMTIME / DT)  # Total simulation steps

# car parameters
L = 1   # car length [m]

K = 15  # turn radius
VG = 4  # vehicle goal velocity

XG = 100    # goal x position
OMEGAG = VG / K # goal angular velocity
Y0 = 0  # starting y coordinate

#obstacle parameters
OBS_Y = Y0  + 1.1 * L
OBS_N = 5 #Changing this value requires recreating the solver

INFRONT_DISTANCE = 15 #Changing this value requires recreating the solver
INFRONT_MAXCOST = 5 #Changing this value requires recreating the solver
MAX_COST = 10

#Constraints #Changing these value requires recreating the solver
V_MAX = 6
STEERING_MAX = 0.2
#ACC_MAX = 2
LATERAL_ACC_MAX = 4