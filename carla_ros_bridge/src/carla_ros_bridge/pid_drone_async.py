
import math

from simple_pid import PID


class Controller_async:
    def __init__(self,sample_time):

        self.yaw_rate_pid = PID(5.0, 0, 1,sample_time=sample_time, setpoint=0, output_limits=(-1, 1))
        self.pitch_pid = PID(1, 0, 1,sample_time=sample_time, setpoint=0, output_limits=(-1, 1))
        self.roll_pid = PID(1, 0, 1,sample_time=sample_time, setpoint=0, output_limits=(-1, 1))       
        self.vforward_pid = PID(10, 0.01, 10,sample_time=sample_time, setpoint=0, output_limits=(-35, 35))
        self.vleft_pid = PID(10, 0.01, 10,sample_time=sample_time, setpoint=0, output_limits=(-35, 35))
        self.vz_pid = PID(1, 0.4, 0,sample_time=sample_time, setpoint=0, output_limits=(0, 1))

    def handle_ros(self, vforward_desired=0, vleft_desired=0,vz_desired=0,yaw_rate_desired=0):
        self.vforward_pid.setpoint=vforward_desired
        self.vleft_desired.setpoint=vleft_desired
        self.vz_desired.setpoint=vz_desired
        self.yaw_rate_desired.setpoint=yaw_rate_desired
        
    def motor_calculate(self,vehicle_transform,vehicle_velocity,vehicle_angular_rate):
        vforward_output =self.vforward_pid(self.forward_speed(vehicle_velocity,math.radians(vehicle_transform.rotation.yaw)))
        self.pitch_pid.setpoint=-vforward_output
        pitch_output =self.pitch_pid(vehicle_transform.rotation.pitch)#1/(100*math.sin(math.radians(abs(vehicle_transform.rotation.pitch)))+0.1)*
        
        vleft_output =self.vleft_pid(self.right_speed(vehicle_velocity,math.radians(vehicle_transform.rotation.yaw)))
        self.roll_pid.setpoint=vleft_output
        roll_output =  self.roll_pid(vehicle_transform.rotation.roll)
       
        yaw_rate_output = self.yaw_rate_pid(-vehicle_angular_rate.z)
        throttle_output =self.vz_pid(vehicle_velocity.z)
        
        #print(vehicle_velocity)
        #print(vehicle_transform)
        #print(roll_output)
        #print("forward_speed",forward_speed(vehicle_velocity,math.radians(vehicle_transform.rotation.yaw)))
        #print("right_speed",right_speed(vehicle_velocity,math.radians(vehicle_transform.rotation.yaw)))
        #print("yawRate",vehicle_angular_rate.yaw_rate)
        speed_set=1
        front_left =    speed_set*throttle_output  + +roll_output + +pitch_output + -yaw_rate_output
        front_right =   speed_set*throttle_output  + -roll_output + +pitch_output + +yaw_rate_output 
        rear_left =     speed_set*throttle_output  + +roll_output + -pitch_output + +yaw_rate_output
        rear_right =    speed_set*throttle_output  + -roll_output + -pitch_output + -yaw_rate_output
        
        return front_left,front_right,rear_left,rear_right
    
    def forward_speed(cls,world_speed,world_yaw):
        return world_speed.x*math.cos(world_yaw)+world_speed.y*math.sin(world_yaw)


    def right_speed(cls,world_speed,world_yaw):
        return -world_speed.x*math.sin(world_yaw)+world_speed.y*math.cos(world_yaw)
    
class AngularRate:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

