import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel,
                max_steer_angle):
        # TODO: Implement
        
        # Init steering/yaw controller
        self.yaw_controller = YawController(wheel_base=wheel_base, 
                                            steer_ratio=steer_ratio, 
                                            min_speed=0.1, 
                                            max_lat_accel=max_lat_accel,
                                            max_steer_angle=max_steer_angle)
        
        # Initialize throttle controller
        kwp = 0.3
        kwi = 0.1
        kdd = 0.
        minn = 0. 
        maxx = 0.4 
        self.throttle_controller = PID(kwp, kwi, kdd, minn, maxx)


        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()

        tau = 0.5 
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)



    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_err = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_err, sample_time)

        brake = 0
        
        if linear_vel == 0 and current_vel < 0:
            throttle = 0.
            brake = 400 
        

        elif throttle < 0.1 and vel_err < 0:
            throttle = 0.
            decel = max(vel_err, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius 
        
        return throttle, brake, steering