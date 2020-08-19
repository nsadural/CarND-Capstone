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
#import do_mpc   # NOTE: Python 3.X package, does not work with Python 2.7
import rospy
from casadi import *
from lowpass import LowPassFilter
import sys

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
V_REF = 0.95 * 25 * ONE_MPH
MAX_LIMIT = sys.float_info.max

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

        self.max_throttle = 0.5*accel_limit
        self.min_throttle = 0.0

        tau = 0.5   # 1/(2pi*tau)==cutoff frequency
        ts = 0.02   # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

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

        ### Define discrete equations
        x1 = x0 + v0*np.cos(psi0)*dt
        y1 = y0 + v0*np.sin(psi0)*dt
        psi1 = psi0 + v0/L_f*steer0*dt
        v1 = v0 + a0*dt
        cte1 = cte0 + v0*np.sin(epsi0)*dt
        epsi1 = epsi0 + v0/L_f*steer0*dt
        model.set_rhs('x0',x1)
        model.set_rhs('y0',y1)
        model.set_rhs('psi0',psi1)
        model.set_rhs('v0',v1)
        model.set_rhs('cte0',cte1)
        model.set_rhs('epsi0',epsi1)

        ### Complete model setup
        model.setup()

        ### Create controller object
        mpc = do_mpc.controller.MPC(model)

        ### Set optimizer parameters
        setup_mpc = {
            'n_horizon': 20,
            't_step': 0.02,
            'store_full_resolution': True,
            }
        mpc.set_param(**setup_mpc)

        ### Define cost function for optimization
        mterm = cte0**2 + epsi0**2 + (v0 - V_REF)**2
        lterm = cte0**2 + epsi0**2 + (v0 - V_REF)**2
        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(
            steer0 = 1e-6,
            a0 = 1e-6
            )

        ### Define input and state constraints
        # Upper bounds on states
        mpc.bounds['upper','_x','x0'] = MAX_LIMIT
        mpc.bounds['upper','_x','y0'] = MAX_LIMIT
        mpc.bounds['upper','_x','psi0'] = MAX_LIMIT
        mpc.bounds['upper','_x','v0'] = 25 * ONE_MPH  # m/s (25 mph)
        mpc.bounds['upper','_x','cte0'] = MAX_LIMIT
        mpc.bounds['upper','_x','epsi0'] = MAX_LIMIT
        # Lower bounds on states
        mpc.bounds['lower','_x','x0'] = -MAX_LIMIT
        mpc.bounds['lower','_x','y0'] = -MAX_LIMIT
        mpc.bounds['lower','_x','psi0'] = -MAX_LIMIT
        mpc.bounds['lower','_x','v0'] = 0.0
        mpc.bounds['lower','_x','cte0'] = -MAX_LIMIT
        mpc.bounds['lower','_x','epsi0'] = -MAX_LIMIT

        # Upper bounds on inputs
        mpc.bounds['upper','_u','steer0'] = max_steer_angle
        mpc.bounds['upper','_u','a0'] = accel_limit
        # Lower bounds on inputs
        mpc.bounds['lower','_u','steer0'] = -max_steer_angle
        mpc.bounds['lower','_u','a0'] = decel_limit

        ### Finalize optimization problem to obtain input
        mpc.setup()

    def control(current_x, current_y, current_psi, current_vel, waypoint_x, waypoint_y, waypoint_psi, dbw_enabled):
        # Check if dbw is enabled; if not, return zero control
        if not dbw_enabled:
            return 0.0, 0.0, 0.0

        # Use lowpass filter to attenuate high-frequency noise in velocity signal
        current_vel = self.vel_lpf(current_vel)

        # Initialize vehicle states
        x_init = current_x
        y_init = current_y
        psi_init = current_psi
        v_init = current_vel
        cte_init = np.sqrt((waypoint_y-current_y)**2 + (waypoint_x-current_x)**2)
        epsi_init = current_psi - waypoint_psi
        state_init = np.array([x_init, y_init, psi_init, v_init, cte_init, epsi_init]).reshape(-1,1)
        mpc.x0 = state_init

        # Initial guess for optimization problem
        mpc.set_initial_guess()

        # Obtain optimal steering and acceleration control input with the current state
        u = mpc.make_step(state_init)

        # Retrieve steering input from optimal control
        steering = u[0]

        # Retrieve acceleration input from optimal control
        accel = u[1]
        brake = 0

        # Determine throttle within limits
        if accel > self.max_throttle:
            throttle = self.max_throttle
        elif accel < self.min_throttle:
            throttle = self.min_throttle
        else:
            throttle = accel
        
        # Determine braking torque
        if current_vel < 0.1:
            throttle = 0
            brake = 700
        elif throttle < 0.1:
            throttle = 0
            decel = max(accel, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        rospy.loginfo('Throttle: %s, Brake: %s, Steering: %s', 
               throttle, brake, steering)

        return throttle, brake, steering