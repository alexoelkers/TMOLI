import scipy.optimize as sco
import numpy as np
from scipy.linalg import block_diag

class Scipy_MPC():

    def __init__(self, target_state = None, steps = 20) -> None:
        self.NUM_OPTIMIZATION_VARIABLES_PER_STEP = 5
        self.NUM_STATES = 3
        self.NUM_INPUTS = 2
        self.DT = 0.1
        self.weight_tracking = 0.5*np.eye(3)
        self.weight_input = 0.5*np.eye(2)
        self._steps = steps
        self.NUM_TOTAL_OPTIM_VAR = self.NUM_OPTIMIZATION_VARIABLES_PER_STEP * self._steps
        if target_state is None:
            self._target_state = np.array([0, 0, 0])
        else:
            self._target_state = target_state

    def get_steps(self) -> int:
        return self._steps

    def get_target_state(self) -> np.ndarray:
        return self._target_state

    def cost_fun(self, optim_variables: np.ndarray) -> float:
        if len(optim_variables) % self.NUM_OPTIMIZATION_VARIABLES_PER_STEP == 0:
            steps = len(optim_variables) // self.NUM_OPTIMIZATION_VARIABLES_PER_STEP
        else:
            raise ValueError(f"Incorrect length of optim_variables. Expected number divisible by {self.NUM_OPTIMIZATION_VARIABLES_PER_STEP}")
        
        cost = 0.

        #reshape the optimization variables
        x, u = self._get_states(optim_variables)

        for k in range(steps):
            state_delta = x[:, k] - self._target_state
            cost += state_delta.transpose() @ self.weight_tracking @ state_delta + u[:, k] @ self.weight_input @ u[:, k]

        return cost


    def discrete_bicycle_constraint(self, optim_variables):
        x, u = self._get_states(optim_variables)

        #Dont need to constrin the input here
        x_k_diff = np.zeros(self._steps*self.NUM_STATES)

        for k in range(self._steps - 1):
            x_k_diff[k*self.NUM_STATES] = (x[0,k] + self.DT*u[0,k]*np.cos(x[2,k])) - x[0,k+1]
            x_k_diff[k*self.NUM_STATES + 1] = (x[1, k] + self.DT*u[0,k]*np.sin(x[2,k])) - x[1, k+1]
            x_k_diff[k*self.NUM_STATES + 2] = (x[2, k] + self.DT*u[0,k]*np.sin(u[1,k])) - x[2, k+1]

        return x_k_diff
    
    def optimize(self, current_state = np.array([0, 0, 0])):
        #TODO: Maybe make these more editable, I don't know...
        bounds = sco.Bounds([-np.inf, -np.inf, -np.pi/2, -1, -1]*self._steps, [np.inf, np.inf, np.pi/2, 1, 1]*self._steps)
        
        untouched_vars = self.NUM_TOTAL_OPTIM_VAR - self.NUM_STATES
        x0 = np.concatenate((current_state, np.zeros(untouched_vars)))

        #Initial constraint
        #Because the optimization variables have to be in a vector and not a matrix
        three_states = np.eye(self.NUM_STATES)
        untouched_vars = self.NUM_TOTAL_OPTIM_VAR - self.NUM_STATES
        zeros = np.zeros((untouched_vars, untouched_vars))
        initial_constraint_matrix = block_diag(three_states, zeros)
        #In scipy optimize, enfore equality by making lb = ub
        init_constraint = sco.LinearConstraint(initial_constraint_matrix, x0, x0)

        x_k_plus_1 = np.zeros(self._steps*self.NUM_STATES)

        state_constraint = sco.NonlinearConstraint(self.discrete_bicycle_constraint, x_k_plus_1, x_k_plus_1)

        constraints = [state_constraint, init_constraint]
        
        result = sco.minimize(self.cost_fun, x0, method='trust-constr', 
                              bounds=bounds, constraints=constraints)
        return result.success, result.x[3:5], result.message
        
    
    def _get_states(self, optim_variables: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Returns the optimization variables x and u in matrix form given the array of all optimization variables
        """
        if len(optim_variables) % self.NUM_OPTIMIZATION_VARIABLES_PER_STEP != 0:
            raise ValueError(f"Incorrect length of optim_varaibles. Expected number divisible by {self.NUM_OPTIMIZATION_VARIABLES_PER_STEP}")

        optim_var_matrix = np.reshape(optim_variables, (self.NUM_OPTIMIZATION_VARIABLES_PER_STEP, -1), order="F")
        x = optim_var_matrix[0:self.NUM_STATES, :]
        u = optim_var_matrix[self.NUM_STATES:self.NUM_OPTIMIZATION_VARIABLES_PER_STEP, :]
        return x, u
    





