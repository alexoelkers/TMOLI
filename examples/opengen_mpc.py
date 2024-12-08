import casadi.casadi as cs
import opengen
import opengen.builder
import opengen.config
import opengen.constraints
import opengen.functions

NUM_OVPS = 5
NUM_STATES = 3
NUM_INPUTS = 2
DT = 0.1
NUM_STEPS = 20
NUM_VARIABLES = NUM_STEPS * NUM_OVPS
weight_tracking = 0.5
weight_input = 0.5

x = cs.SX.sym("x", NUM_VARIABLES) #Optimization variables
p = cs.SX.sym("p", 6)

def cost(x, p):
    target_state = p[3:]

    tracking_cost = 0.5
    input_cost = 0.5

    cost = 0

    for k in range(NUM_STEPS):
        #Tracking cost
        cost += tracking_cost*((x[k*NUM_OVPS] - target_state[0])**2 + (x[k*NUM_OVPS + 1] - target_state[+ 2])**2 + (x[k*NUM_OVPS + 2] - target_state[2])**2 )
        #Input cost
        cost += input_cost*((x[k*NUM_OVPS+3])**2 + (x[k*NUM_OVPS+4])**2)

    return cost

def equality_constraint_func():
    #All constraints within must be 0
    #Enforece that x_0 is the intial state
    const_list = [x[0] - p[0], x[1] - p[1], x[2] - p[2]]

    #Enforce the model
    for k in range(NUM_STEPS - 1):
        const_list.append(x[(k+1)*NUM_OVPS] - (x[k*NUM_OVPS] + DT * x[(k)*NUM_OVPS + 3] * cs.cos(x[k*NUM_OVPS + 2])))
        const_list.append(x[(k+1)*NUM_OVPS+1] - (x[k*NUM_OVPS+1] + DT * x[(k)*NUM_OVPS + 3] * cs.sin(x[k*NUM_OVPS + 2])))
        const_list.append(x[(k+1)*NUM_OVPS+2] - (x[k*NUM_OVPS+2] + DT * x[(k)*NUM_OVPS + 3] * cs.sin(x[k*NUM_OVPS + 4])))

    concatenated = cs.vertcat(*const_list)
    return concatenated

def build():

    f1 = equality_constraint_func()
    set_c = opengen.constraints.Zero()

    problem = opengen.builder.Problem(x, p, cost(x,p)).with_aug_lagrangian_constraints(f1, set_c)

    meta = opengen.config.OptimizerMeta('mpc_demo', '0.1.0')
    build_config = opengen.config.BuildConfiguration().with_build_directory("opengen_build").with_build_mode("debug").with_tcp_interface_config()
    solver_config = opengen.config.SolverConfiguration().with_lbfgs_memory(15)

    builder = opengen.builder.OpEnOptimizerBuilder(problem, meta, build_config, solver_config)

    builder.build()

if __name__ == "__main__":
    build()



