import numpy as np
from scipy import interpolate
from math import pi
import rospy

class LateralLQR(object):
    def __init__(self, wheel_base, max_steer_angle, steer_ratio):
        self.wheel_base = wheel_base
        self.max_steer = max_steer_angle
        self.min_steer = -max_steer_angle
        self.steer_ratio = steer_ratio
        
        self.sample_time = 0.020
        self.control_horizon = 9

    def get_steering(self, current_x, current_y, current_psi, current_velocity, current_yaw_rate, trajectory_x, trajectory_y, trajectory_psi, des_velocity, des_yaw_rate):
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
        x = current_x*np.cos(theta) - current_y*np.sin(theta)
        y = current_x*np.sin(theta) + current_y*np.cos(theta)
        psi = current_psi + theta
        for i in range(len(trajectory_x)):
            trajectory_x[i] = trajectory_x[i]*np.cos(theta) - trajectory_y[i]*np.sin(theta)
            trajectory_y[i] = trajectory_x[i]*np.sin(theta) + trajectory_y[i]*np.cos(theta)
            trajectory_psi[i] = trajectory_psi[i] + theta
            

        # Cubic spline fit for transformed trajectory
        cs = interpolate.CubicSpline(trajectory_x, trajectory_y)

        # Current vehicle error state
        cte = (cs(0, nu=1)*x - y + trajectory_y[0]) / (np.sqrt(pow(cs(0, nu=1), 2) + 1))
        #epsi = (psi - np.arctan(cs(5, nu=1)))
        epsi = psi - trajectory_psi[0]

        ### LQR gain schedule
        k1 = -0.000077*current_velocity + 0.044710
        k2 = -0.000441*current_velocity + 0.504839

        # Feedforward compensation
        steering_ff = np.arctan(des_yaw_rate*self.wheel_base/des_velocity)
        k_ff = 1.5
        
        # LQR with feedforward compensation
        steering = (-k1*cte - k2*epsi + k_ff*steering_ff)*self.steer_ratio

        # Verify control within limits
        if steering > self.max_steer:
            steering = self.max_steer
        elif steering < self.min_steer:
            steering = self.min_steer
            
        rospy.logwarn("cte [m]: {0}".format(cte))
        rospy.logwarn("epsi [rad]: {0}".format(epsi))
        #rospy.logwarn("x [m]: {0}".format(x))
        #rospy.logwarn("y [m]: {0}".format(y))
        #rospy.logwarn("trajectory_x[0]: {0}".format(trajectory_x[0]))
        #rospy.logwarn("trajectory_y[0]: {0}".format(trajectory_y[0]))
        #rospy.logwarn("trajectory_x[1]: {0}".format(trajectory_x[1]))
        #rospy.logwarn("trajectory_y[1]: {0}".format(trajectory_y[1]))
        #rospy.logwarn("steering_ff [rad]: {0}".format(steering_ff))
        rospy.logwarn("delta [rad]: {0}\n".format(steering))
        
        steering_list = []
        steering_list.append(steering)
        for j in range(self.control_horizon-1):
            steering, cte, epsi = self.propogate_error(cte, epsi, current_velocity, steering, steering_ff, k1, k2, k_ff)
            steering_list.append(steering)

        return steering_list
    
    def propogate_error(self, cte, epsi, current_velocity, steering, steering_ff, k1, k2, k_ff):
        cte_next = epsi*current_velocity*self.sample_time
        epsi_next = ((current_velocity/self.wheel_base)*(steering)*self.sample_time)
        
        steering_next = (-k1*cte_next - k2*epsi_next + k_ff*steering_ff)*self.steer_ratio
        
        if steering_next > self.max_steer:
            steering_next = self.max_steer
        elif steering_next < self.min_steer:
            steering_next = self.min_steer
            
        return steering_next, cte_next, epsi_next
    
