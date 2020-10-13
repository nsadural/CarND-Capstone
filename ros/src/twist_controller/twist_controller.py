from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
from lateral_mpc import LateralMPC
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        #self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        self.yaw_controller = LateralMPC(vehicle_mass, wheel_base, max_steer_angle, steer_ratio)
        
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. # Minimum throttle value
        mx = 0.5 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts) # Filter out the high-frequency noise in velocity
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.future_steering = []
        self.previous_steering = 0
        self.iteration = 0
        
        self.last_time = rospy.get_time()
        

    def control(self, current_vel, curr_ang_vel, linear_vel, angular_vel, dbw_enabled, current_x, current_y, current_psi, current_latvel, trajectory_x, trajectory_y, trajectory_psi):
        # Return throttle, brake, steer
        
        # If the dbw is turned off, we will reset our PID controller and return nothing
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        # Filter out the high-frequency noise in velocity
        curren_vel = self.vel_lpf.filt(current_vel)
        curren_latvel = self.vel_lpf.filt(current_latvel)
        #steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        if not self.future_steering:
            self.future_steering = self.yaw_controller.get_steering(self.previous_steering, 
                                                                    current_x, 
                                                                    current_y, 
                                                                    current_psi, 
                                                                    curren_vel, 
                                                                    curren_latvel, 
                                                                    curr_ang_vel, 
                                                                    trajectory_x, 
                                                                    trajectory_y, 
                                                                    trajectory_psi) 

        self.previous_steering = self.future_steering.pop(0)
        steering = self.previous_steering
        
        rospy.logwarn("ITERATION : {0}".format(self.iteration))
        self.iteration += 1
        #rospy.logwarn("Current velocity : {0}".format(current_vel))
        #rospy.logwarn("Current steering : {0}".format(steering))
        #rospy.logwarn("Current x : {0}".format(current_x))
        #rospy.logwarn("Current y : {0}".format(current_y))
        #rospy.logwarn("Current heading : {0}".format(current_psi))
        #rospy.logwarn("Trajectory x[0] : {0}".format(trajectory_x[0]))
        #rospy.logwarn("Trajectory y[0] : {0}\n".format(trajectory_y[0]))
        #rospy.logwarn("Trajectory heading[0] : {0}".format(trajectory_psi[0]))
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        current_time = rospy.get_time() 
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0
        
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 700 # N*m - to hold the car in place if we are stopped at a light. Acceleration - 1m/s^2
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Brake torque( N*m) = deceleration * mass * wheel radius
        
        #rospy.loginfo('Throttle: %s, Brake: %s, Steering: %s', throttle, brake, steering)
        
        return throttle, brake, steering
