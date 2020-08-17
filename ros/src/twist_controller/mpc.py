###
#
#   File: mpc.py
#   
#   Description: MPC class to implement discrete model predictive control 
#   for steering_cmd, throttle_cmd, and brake_cmd in the Udacity 
#   Self-Driving Car Engineer capstone project.
#
#   Author: Nikko Sadural (nsadural)
#   
#   Sources: https://www.do-mpc.com/en/latest/ (model predictive control python toolbox)
#
###

import numpy as np
import do_mpc   # Updated 'requirements.txt' with dependencies
import rospy
from casadi import *

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class MPC(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        ### Define model type
        model_type = 'discrete'
        model = do_mpc.model.Model(model_type)

        ### Define model variables
        # States
        x0 = model.set_variable(var_type='_x', var_name='x0', shape=(1,1))
        y0 = model.set_variable(var_type='_x', var_name='y0', shape=(1,1))
        psi0 = model.set_variable(var_type='_x', var_name='psi0', shape=(1,1))
        v0 = model.set_variable(var_type='_x', var_name='v0', shape=(1,1))
        cte0 = model.set_variable(var_type='_x', var_name='cte0', shape=(1,1))
        epsi0 = model.set_variable(var_type='_x', varname='epsi0', shape=(1,1))
        # Controls
        steer0 = model.set_variable(var_type='_u', varname='steer0', shape=(1,1))
        a0 = model.set_variable(var_type='_u', varname='a0', shape=(1,1))

        ### Define model parameters
        dt = 0.02   # 1/50 Hz = 0.02 s
        L_f = 0.40*wheel_base   # TODO: Tune parameter due to uncertainty based on simulation performance
        # TODO: Need to include f(xt), f'(xt) terms for cte and epsi variables (polyfit, polyeval)

        ### Define discrete equations
        x1 = x0 + v0*np.cos(psi0)*dt
        y1 = y0 + v0*np.sin(psi0)*dt
        psi1 = psi0 + v0/L_f*steer0*dt
        v1 = v0 + a0*dt
        #cte1 = ...
        #epsi1 = ...
        model.set_rhs('x0',x1)
        model.set_rhs('y0',y1)
        model.set_rhs('psi0',psi1)
        model.set_rhs('v0',v1)
        #model.set_rhs('cte0',cte1)
        #model.set_rhs('epsi0',epsi1)
        # TODO: Need to include f(xt), f'(xt) terms for cte and epsi variables (polyfit, polyeval)

        ### Complete model setup
        model.setup()

        # TODO: Set optimizer parameters, cost function, constraints, scaling?, setup