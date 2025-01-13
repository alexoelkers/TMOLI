class SolverError(Exception):
    """
    The optimizer could not solve the optimization problem.
    """

    def __init__(self, *args):
        super().__init__(*args)