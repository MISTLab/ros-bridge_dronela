#!/usr/bin/env python

#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""
import math
import os
import time

from carla_msgs.msg import CarlaWorldInfo


import numpy
from carla import VehicleControl

from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_ros_bridge.drone import Drone
from carla_ros_bridge.pid_drone_sync import Controller_sync, AngularRate
from carla_ros_bridge.pid_drone_async import Controller_async

from sensor_msgs.msg import Imu

from carla_msgs.msg import (
    CarlaEgoVehicleInfo,
    CarlaEgoVehicleInfoWheel,
    CarlaEgoVehicleControl,
    DronelaEgoDroneControl,
    CarlaEgoVehicleStatus,
    CarlaStatus
)
from std_msgs.msg import Bool  # pylint: disable=import-error
from std_msgs.msg import ColorRGBA  # pylint: disable=import-error
from carla_ros_bridge.drone import Drone



  

class EgoDrone(Drone):

    """
    Vehicle implementation details for the ego drone
    """

    def __init__(self, world, uid, name, parent, node, carla_actor, vehicle_control_applied_callback):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        super(EgoDrone, self).__init__(world=world,uid=uid,
                                         name=name,
                                         parent=parent,
                                         node=node,
                                         carla_actor=carla_actor
                                         
                                         )

        self.vehicle_info_published = False
        self.vehicle_control_override = False
        self._vehicle_control_applied_callback = vehicle_control_applied_callback

        self.vehicle_status_publisher = node.new_publisher(
            CarlaEgoVehicleStatus,
            self.get_topic_prefix() + "/vehicle_status",
            qos_profile=10)
        self.vehicle_info_publisher = node.new_publisher(
            CarlaEgoVehicleInfo,
            self.get_topic_prefix() +
            "/vehicle_info",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.control_subscriber = node.new_subscription(
            CarlaEgoVehicleControl,
            self.get_topic_prefix() + "/vehicle_control_cmd",
            lambda data: self.control_command_updated(data, manual_override=False),
            qos_profile=10)
        
        self.imu_subscriber = node.new_subscription(
            Imu,
            self.get_topic_prefix() + "/imu",
            lambda imu_data: self.imu_updated(imu_data),
            qos_profile=10)
        self.angular_rate = AngularRate()
        

        self.manual_control_subscriber = node.new_subscription(
            DronelaEgoDroneControl,
            self.get_topic_prefix() + "/drone_control_cmd_manual",
            lambda data: self.control_command_updated(data, manual_override=True),
            qos_profile=10)


        self.control_override_subscriber = node.new_subscription(
            Bool,
            self.get_topic_prefix() + "/vehicle_control_manual_override",
            self.control_command_override,
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.enable_autopilot_subscriber = node.new_subscription(
            Bool,
            self.get_topic_prefix() + "/enable_autopilot",
            self.enable_autopilot_updated,
            qos_profile=10)
        settings=world.get_settings()
        if settings.synchronous_mode==True:
            self.controller=Controller_sync(settings.fixed_delta_seconds)
        else:
            self.controller=Controller_async()

        #world.on_tick(lambda world_snap_shot:self.on_event_tick_world(world_snap_shot))
        self.carla_status_subscriber = self.node.new_subscription(
            CarlaStatus,
            "/status",
            self._on_new_carla_frame,
            qos_profile=10)
        
    def _on_new_carla_frame(self, data):
        """
        callback on new frame

        As CARLA only processes one vehicle control command per tick,
        send the current from within here (once per frame)
        """
        vehicle_transform = self.carla_actor.get_transform()
        vehicle_velocity= self.carla_actor.get_velocity()
        vehicle_angular_rate=self.get_angular_velocity()
        #spectator.set_transform(carla.Transform(vehicle_transform.location+carla.Location(x=0) +carla.Location(y=-0) + carla.Location(z=4), carla.Rotation(pitch=-90)))
        front_left,front_right,rear_left,rear_right=self.controller.motor_calculate(vehicle_transform,
                                                                                    vehicle_velocity,
                                                                                    vehicle_angular_rate)

        self.carla_actor.apply_motor_speed(front_left*3000,front_right*3000,rear_left*3000,rear_right*3000)
        #print(self.get_id(),"released control lock")
        self._vehicle_control_applied_callback(self.get_id())
            


    # def on_event_tick_world(self,world_snap_shot):
        # vehicle_transform = self.carla_actor.get_transform()
        # vehicle_velocity= self.carla_actor.get_velocity()
        # vehicle_angular_rate=self.get_angular_velocity()
        # #spectator.set_transform(carla.Transform(vehicle_transform.location+carla.Location(x=0) +carla.Location(y=-0) + carla.Location(z=4), carla.Rotation(pitch=-90)))
        # front_left,front_right,rear_left,rear_right=self.controller.motor_calculate(vehicle_transform,
        #                                                                             vehicle_velocity,
        #                                                                             vehicle_angular_rate)

        # self.carla_actor.apply_motor_speed(front_left*3000,front_right*3000,rear_left*3000,rear_right*3000)
        # print(self.get_id(),"released control lock")
        # self._vehicle_control_applied_callback(self.get_id())
           
    def imu_updated(self,imu_data):
        self.angular_rate = imu_data.angular_velocity

    def get_angular_velocity(self):
        return self.angular_rate
    
    def get_marker_color(self):
        """
        Function (override) to return the color for marker messages.

        The ego vehicle uses a different marker color than other vehicles.

        :return: the color used by a ego vehicle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0.0
        color.g = 255.0
        color.b = 0.0
        return color

    def send_vehicle_msgs(self, frame, timestamp):
        """
        send messages related to vehicle status

        :return:
        """
        vehicle_status = CarlaEgoVehicleStatus(
            header=self.get_msg_header("map", timestamp=timestamp))
        vehicle_status.velocity = self.get_vehicle_speed_abs(self.carla_actor)
        vehicle_status.acceleration.linear = self.get_current_ros_accel().linear
        vehicle_status.orientation = self.get_current_ros_pose().orientation
        # vehicle_status.control.throttle = self.carla_actor.get_control().throttle
        # vehicle_status.control.steer = self.carla_actor.get_control().steer
        # vehicle_status.control.brake = self.carla_actor.get_control().brake
        # vehicle_status.control.hand_brake = self.carla_actor.get_control().hand_brake
        # vehicle_status.control.reverse = self.carla_actor.get_control().reverse
        # vehicle_status.control.gear = self.carla_actor.get_control().gear
        # vehicle_status.control.manual_gear_shift = self.carla_actor.get_control().manual_gear_shift
        self.vehicle_status_publisher.publish(vehicle_status)

        # only send vehicle once (in latched-mode)
        # if not self.vehicle_info_published:
        #     self.vehicle_info_published = True
        #     vehicle_info = CarlaEgoVehicleInfo()
        #     vehicle_info.id = self.carla_actor.id
        #     vehicle_info.type = self.carla_actor.type_id
        #     vehicle_info.rolename = self.carla_actor.attributes.get('role_name')
        #     vehicle_physics = self.carla_actor.get_physics_control()

        #     for wheel in vehicle_physics.wheels:
        #         wheel_info = CarlaEgoVehicleInfoWheel()
        #         wheel_info.tire_friction = wheel.tire_friction
        #         wheel_info.damping_rate = wheel.damping_rate
        #         wheel_info.max_steer_angle = math.radians(wheel.max_steer_angle)
        #         wheel_info.radius = wheel.radius
        #         wheel_info.max_brake_torque = wheel.max_brake_torque
        #         wheel_info.max_handbrake_torque = wheel.max_handbrake_torque

        #         inv_T = numpy.array(self.carla_actor.get_transform().get_inverse_matrix(), dtype=float)
        #         wheel_pos_in_map = numpy.array([wheel.position.x/100.0,
        #                                 wheel.position.y/100.0,
        #                                 wheel.position.z/100.0,
        #                                 1.0])
        #         wheel_pos_in_ego_vehicle = numpy.matmul(inv_T, wheel_pos_in_map)
        #         wheel_info.position.x = wheel_pos_in_ego_vehicle[0]
        #         wheel_info.position.y = -wheel_pos_in_ego_vehicle[1]
        #         wheel_info.position.z = wheel_pos_in_ego_vehicle[2]
        #         vehicle_info.wheels.append(wheel_info)

        #     vehicle_info.max_rpm = vehicle_physics.max_rpm
        #     vehicle_info.max_rpm = vehicle_physics.max_rpm
        #     vehicle_info.moi = vehicle_physics.moi
        #     vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
        #     vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
        #         vehicle_physics.damping_rate_zero_throttle_clutch_engaged
        #     vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
        #         vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
        #     vehicle_info.use_gear_autobox = vehicle_physics.use_gear_autobox
        #     vehicle_info.gear_switch_time = vehicle_physics.gear_switch_time
        #     vehicle_info.clutch_strength = vehicle_physics.clutch_strength
        #     vehicle_info.mass = vehicle_physics.mass
        #     vehicle_info.drag_coefficient = vehicle_physics.drag_coefficient
        #     vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
        #     vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
        #     vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z

        #     self.vehicle_info_publisher.publish(vehicle_info)

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        self.send_vehicle_msgs(frame, timestamp)
        super(EgoDrone, self).update(frame, timestamp)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscriptions
        Finally forward call to super class.

        :return:
        """
        self.node.logdebug("Destroy Vehicle(id={})".format(self.get_id()))
        self.node.destroy_subscription(self.control_subscriber)
        self.node.destroy_subscription(self.enable_autopilot_subscriber)
        self.node.destroy_subscription(self.control_override_subscriber)
        self.node.destroy_subscription(self.manual_control_subscriber)
        self.node.destroy_publisher(self.vehicle_status_publisher)
        self.node.destroy_publisher(self.vehicle_info_publisher)
        Drone.destroy(self)

    def control_command_override(self, enable):
        """
        Set the vehicle control mode according to ros topic
        """
        self.vehicle_control_override = enable.data

    def control_command_updated(self, ros_vehicle_control, manual_override):
        """
        Receive a CarlaEgoVehicleControl msg and send to CARLA

        This function gets called whenever a ROS CarlaEgoVehicleControl is received.
        If the mode is valid (either normal or manual), the received ROS message is
        converted into carla.VehicleControl command and sent to CARLA.
        This bridge is not responsible for any restrictions on velocity or steering.
        It's just forwarding the ROS input to CARLA

        :param manual_override: manually override the vehicle control command
        :param ros_vehicle_control: current vehicle control input received via ROS
        :type ros_vehicle_control: carla_msgs.msg.CarlaEgoVehicleControl
        :return:
        """
        self.controller.vz_pid.setpoint=ros_vehicle_control.vz
        self.controller.vforward_pid.setpoint=ros_vehicle_control.vforward
        self.controller.vleft_pid.setpoint=-ros_vehicle_control.vleft
        self.controller.yaw_rate_pid.setpoint=-ros_vehicle_control.yawrate

        if manual_override == self.vehicle_control_override:
            pass

    def enable_autopilot_updated(self, enable_auto_pilot):
        """
        Enable/disable auto pilot

        :param enable_auto_pilot: should the autopilot be enabled?
        :type enable_auto_pilot: std_msgs.Bool
        :return:
        """
        self.node.logdebug("Ego vehicle: Set autopilot to {}".format(enable_auto_pilot.data))
        self.carla_actor.set_autopilot(enable_auto_pilot.data)

    @staticmethod
    def get_vector_length_squared(carla_vector):
        """
        Calculate the squared length of a carla_vector
        :param carla_vector: the carla vector
        :type carla_vector: carla.Vector3D
        :return: squared vector length
        :rtype: float64
        """
        return carla_vector.x * carla_vector.x + \
            carla_vector.y * carla_vector.y + \
            carla_vector.z * carla_vector.z

    @staticmethod
    def get_vehicle_speed_squared(carla_vehicle):
        """
        Get the squared speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: squared speed of a carla vehicle [(m/s)^2]
        :rtype: float64
        """
        return EgoDrone.get_vector_length_squared(carla_vehicle.get_velocity())

    @staticmethod
    def get_vehicle_speed_abs(carla_vehicle):
        """
        Get the absolute speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: speed of a carla vehicle [m/s >= 0]
        :rtype: float64
        """
        speed = math.sqrt(EgoDrone.get_vehicle_speed_squared(carla_vehicle))
        return speed
