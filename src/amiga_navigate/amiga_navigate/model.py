import do_mpc
from casadi import *

def model():
    model_type = 'continuous'
    model = do_mpc.model.Model(model_type)

    # States
    x = model.set_variable(var_type='_x', var_name='x', shape=(1, 1))
    y = model.set_variable(var_type='_x', var_name='y', shape=(1, 1))
    theta = model.set_variable(var_type='_x', var_name='theta', shape=(1, 1))

    # Controls
    v = model.set_variable(var_type='_u', var_name='v', shape=(1, 1))
    omega = model.set_variable(var_type='_u', var_name='omega', shape=(1, 1))

    # Model equations
    model.set_rhs('x', v * cos(theta))
    model.set_rhs('y', v * sin(theta))
    model.set_rhs('theta', omega)

    model.setup()

    return model
