from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity,brake_deadband,decel_limit,
        accel_limit,wheel_radius,wheel_base, steer_ratio,max_lat_accel,max_steer_angle,
        kp_throttle, ki_throttle, kd_throttle, kp_steer, ki_steer, kd_steer, steering_ratio):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base,steer_ratio,0.1,max_lat_accel,max_steer_angle)
   
        mn = 0.   # minimum throttle value
        mx = 0.5  # maximum throttle value
        self.throttle_controller = PID(kp_throttle,ki_throttle,kd_throttle,mn,mx)
        band = 4.   # max steering value
        self.steering_controller = PID(kp_steer, ki_steer, kd_steer, -band, band)
        self.steering_ratio = steering_ratio

        tau = 0.5 # 1/(2pi *tau) = cutoff frequency
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau,ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.max_steer_angle = max_steer_angle

        self.last_time = rospy.get_time()
        self.last_vel = 0.

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, distance):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            self.steering_controller.reset()
            return 0.,0.,0.

        current_vel = self.vel_lpf.filt(current_vel)

        # rospy.logwarn("Angular vel: {0}".format(angular_vel))
        # rospy.logwarn("Target velocity: {0}".format(linear_vel))
        # rospy.logwarn("Target angular velocity: {0}".format(angular_vel))
        # rospy.logwarn("Current velocity: {0}".format(current_vel))
        # rospy.logwarn("Filtered velocity: {0}".format(self.vel_lpf.get()))

        # pilot control
        steering = self.steering_ratio * self.yaw_controller.get_steering(linear_vel,angular_vel, current_vel)

        # error between current and expected
        vel_error = linear_vel - current_vel

        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        steering += (1. - self.steering_ratio) * self.steering_controller.step(distance, sample_time)
        if -1. * self.max_steer_angle > steering:
            steering = -1 * self.max_steer_angle
        elif self.max_steer_angle < steering:
            steering = self.max_steer_angle
        brake = 0

        if linear_vel == 0 and current_vel < 0.1:
            throttle = 0
            brake = 400 # N*.m to hold the car in place if we are stopped at light . Acceleration < 1m/s^2

        if current_vel > linear_vel:
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass * self.wheel_radius   # Torque N*m
        return throttle, brake, steering 



