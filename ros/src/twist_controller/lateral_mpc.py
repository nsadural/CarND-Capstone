# TODO:
#   - Check ros dbw node to make sure all vehicle states are available (pose, speed, yaw rate)

from gekko import GEKKO
import numpy as np
from scipy import interpolate
from math import pi
import rospy

class LateralMPC(object):
    def __init__(self, vehicle_mass, wheel_base, max_steer_angle, steer_ratio):
        self.vehicle_mass = vehicle_mass
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.front_to_cg = 0.35*wheel_base
        self.rear_to_cg = wheel_base - self.front_to_cg
        self.yaw_inertial_moment = 2.86*vehicle_mass - 1315
        self.max_steer = max_steer_angle
        self.min_steer = -max_steer_angle
        self.front_cornering_stiffness = 867*180/pi
        self.rear_cornering_stiffness = 867*180/pi
        self.pred_horizon = 20
        self.pred_time = 0.1
        self.ctrl_horizon = 1        
        
    def get_steering(self, current_steer, current_x, current_y, current_psi, current_velocity, current_lateral_velocity, current_yaw_rate, trajectory_x, trajectory_y, trajectory_psi):
        # Translate vehicle and trajectory points to trajectory frame
        x_t = trajectory_x[0]
        y_t = trajectory_y[0]
        current_x -= x_t
        current_y -= y_t
        for i in range(len(trajectory_x)):
            trajectory_x[i] -= x_t
            trajectory_y[i] -= y_t

        # Rotate vehicle and trajectory points clockwise to trajectory frame
        theta = -np.arctan2(trajectory_y[1], trajectory_x[1])
        x0 = current_x*np.cos(theta) - current_y*np.sin(theta)
        y0 = current_x*np.sin(theta) + current_y*np.cos(theta)
        psi0 = current_psi + theta
        for i in range(len(trajectory_x)):
            trajectory_x[i] = trajectory_x[i]*np.cos(theta) - trajectory_y[i]*np.sin(theta)
            trajectory_y[i] = trajectory_x[i]*np.sin(theta) + trajectory_y[i]*np.cos(theta)
            trajectory_psi[i] += theta
            
        ### DEBUG ###
        #rospy.logwarn("x_t : {0}".format(x_t))
        #rospy.logwarn("y_t : {0}".format(y_t))
        #rospy.logwarn("Transformed trajectory_x : {0}".format(trajectory_x))

        # Polynomial fit of trajectory cubic spline and derivatives
        cs = interpolate.CubicSpline(trajectory_x, trajectory_y)
        xs = np.arange(0, trajectory_x[-1], 1)
        dys = cs(xs, nu=1)
        ddys = cs(xs, nu=2)
        coeffs_ys = np.polyfit(xs, cs(xs), 3)
        coeffs_dys = np.polyfit(xs, dys, 2)
        coeffs_ddys = np.polyfit(xs, ddys, 1)

        # Initial conditions
        cte0 = (cs(0, nu=1)*x0 - y0 + trajectory_y[0]) / (np.sqrt(pow(cs(0, nu=1), 2) + 1))
        epsi0 = np.arctan(cs(0, nu=1)) - psi0
        delta0 = current_steer
        beta0 = np.arctan((self.rear_to_cg/self.wheel_base)*np.tan(delta0*pi/180))
        yd0 = current_lateral_velocity
        psid0 = current_yaw_rate

        # Setup GEKKO model
        m = GEKKO(remote=True)
        m.time = np.linspace(0, self.pred_horizon*self.pred_time, self.pred_horizon + 1)
        
        # Setup model control variable
        delta = m.MV(value=delta0, lb=self.min_steer/self.steer_ratio*180/pi, ub=self.max_steer/self.steer_ratio*180/pi, name='delta')
        delta.STATUS = 1
        delta.COST = 0
        delta.DCOST = 1000

        # Setup model controlled state variables
        cte = m.CV(value=cte0, name='cte')
        cte.STATUS = 1
        cte.SP = 0

        epsi = m.CV(value=epsi0, name='epsi')
        epsi.STATUS = 1
        epsi.SP = 0

        m.options.CV_TYPE = 2

        # Setup model uncontrolled state variables
        x = m.Var(value=x0, name='x')
        y = m.Var(value=y0, name='y')
        psi = m.Var(value=psi0, name='psi')
        beta = m.Var(value=beta0, name='beta')
        yd = m.Var(value=yd0, name='yd')
        psid = m.Var(value=psid0, name='psid')

        # Setup model intermediates for interim calculations(
        y_des = m.Intermediate(coeffs_ys[0]*x**3 + coeffs_ys[1]*x**2 + coeffs_ys[2]*x + coeffs_ys[3])
        psi_des = m.Intermediate(m.atan(coeffs_dys[0]*x**2 + coeffs_dys[1]*x + coeffs_dys[2]))
        A = m.Intermediate(coeffs_dys[0]*x**2 + coeffs_dys[1]*x + coeffs_dys[2])
        B = m.Const(-1)
        C = m.Intermediate(-B*y_des - A*x)

        # Select vehicle motion model
        thresh_velocity = float('inf')  # Since vehicle does not return twist.linear.y velocity...
        if (current_velocity < thresh_velocity):
            # Kinematic bicycle model
            m.Equations([x.dt() == current_velocity*m.cos(psi + beta),
                         y.dt() == current_velocity*m.sin(psi + beta),
                         psi.dt() == current_velocity*m.cos(beta)/self.wheel_base*m.tan(delta*pi/180),
                         cte == (A*x + B*y + C)/m.sqrt(A*A + B*B),
                         epsi == psi_des - (psi + beta),
                         beta == m.atan((self.rear_to_cg/self.wheel_base)*m.tan(delta*pi/180))])

        else:
            # Linear tire model
            alpha_f = m.Intermediate(-delta*pi/180 + m.atan((yd + psid*self.front_to_cg)/current_velocity))
            alpha_r = m.Intermediate(m.atan((yd - psid*self.rear_to_cg)/current_velocity))
            F_f = m.Intermediate(-self.front_cornering_stiffness*alpha_f)
            F_r = m.Intermediate(-self.rear_cornering_stiffness*alpha_r)
            
            # Dynamic bicycle model
            m.Equations([yd.dt() == -current_velocity*psid + (2/self.vehicle_mass)*(F_f*m.cos(delta*pi/180) + F_r),
                         self.yaw_inertial_moment*psid.dt() == 2*(self.front_to_cg*F_f - self.rear_to_cg*F_r),
                         x.dt() == current_velocity*m.cos(psi) - yd*m.sin(psi),
                         y.dt() == current_velocity*m.sin(psi) + yd*m.cos(psi),
                         psi.dt() == psid,
                         cte == (A*x + B*y + C)/m.sqrt(A*A + B*B),
                         epsi == psi_des - psi])

        # Solve control optimization problem
        m.options.IMODE = 6
        m.options.solver = 3
        m.solve(disp=False, GUI=False)

        # Return steering commands for predefined control horizon
        steering = []
        for i in range(self.ctrl_horizon):
            steering.append(delta.value[i+1]*self.steer_ratio*pi/180)
        
        ### DEBUG ###
        rospy.logwarn("MPC steering : {0}\n".format(steering))

        return steering







        
