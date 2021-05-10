#!/usr/bin/python3

from math import pi
import os
import sys
import logging
import asyncio
import threading
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import RvrStreamingServices
from sphero_sdk import BatteryVoltageStatesEnum as VoltageStates
from sphero_sdk import ControlSystemTypesEnum
from sphero_sdk import ControlSystemIdsEnum
from sphero_sdk import SpheroRvrTargets

import tf2_ros
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

#from scipy.spatial.transform import Rotation

class Sphero(Node):
    # =======================================================
    # Constructor
    # =======================================================
    def __init__(self):
        super().__init__('sphero')
        self.logger = self.get_logger()
        self.t_stop = threading.Event() # threading stuff

        #======== Define parameters =========#
        self.declare_parameter("port", "/dev/ttyS0",
                                    ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                    description='UART port connected to Sphero'))
        self.declare_parameter("cmd_vel", "cmd_vel",
                                    ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                    description='Topic to subscribe to for cmd_vel messages'))
        self.declare_parameter("imu_link", "imu_link",
                                    ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                    description='IMU LINK'))
        self.declare_parameter("imu", "imu",
                                    ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                    description='IMU'))
        self.declare_parameter("odom", "odom",
                                    ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                    description='ODOM'))
        self.declare_parameter("base_link", "base_link",
                                    ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                    description='BASE LINK'))
        self.declare_parameter("base_footprint", "base_footprint",
                                    ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                    description='BASE FOOTPRINT'))
        self.declare_parameter("laser", "laser",
                                    ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                    description='LASER'))

        self.received_components = set()

        self.loop = asyncio.get_event_loop()
        self.rvr = SpheroRvrAsync(
            dal=SerialAsyncDal(
                loop=self.loop,
                port_id=self.get_parameter("port").value,
            )
        )

        _ = self.create_subscription(Twist,
            self.get_parameter("cmd_vel").value, self.cmdvel_cb, 10)

        self.imu = Imu(header=Header(frame_id=self.get_parameter("imu_link").value))
        self.imu.orientation_covariance = [1e-6, .0, .0, .0, 1e-6, .0, .0, .0, 1e-6]
        self.imu.angular_velocity_covariance = [1e-6, .0, .0, .0, 1e-6, .0, .0, .0, 1e-6]
        self.imu.linear_acceleration_covariance = [1e-6, .0, .0, .0, 1e-6, .0, .0, .0, 1e-6]
        self.imu_pub = self.create_publisher(Imu, self.get_parameter("imu").value, 10)

        self.acceleration = Vector3()

        self.position = Point()
        self.orientation = Quaternion()
        self.pose = Pose(
            position=self.position,
            orientation=self.orientation,
        )

        self.pose_with_covariance = PoseWithCovariance(pose=self.pose)
        self.linear = Vector3()
        self.angular = Vector3()
        self.twist = Twist(linear=self.linear,angular=self.angular)
        self.twist_with_covariance = TwistWithCovariance(twist=self.twist)
        self.odom = Odometry(
            pose=self.pose_with_covariance,
            twist=self.twist_with_covariance,
            header=Header(
                frame_id=self.get_parameter("odom").value,
            ),
            child_frame_id=self.get_parameter("base_footprint").value
        )

        self.pub_odom = self.create_publisher(Odometry, self.get_parameter("odom").value, 10)

        self.odom_trans = tf2_ros.TransformStamped()
        self.odom_trans.header.frame_id = self.get_parameter("odom").value
        self.odom_trans.child_frame_id = self.get_parameter("base_footprint").value

        self.laser_trans = tf2_ros.TransformStamped()
        self.laser_trans.header.frame_id = self.get_parameter("base_link").value
        self.laser_trans.child_frame_id = self.get_parameter("laser").value
        self.laser_trans.transform.translation.x = 0.05
        self.laser_trans.transform.translation.y = 0.0
        self.laser_trans.transform.translation.z = 0.10
        self.laser_trans.transform.rotation.x = 0.0
        self.laser_trans.transform.rotation.y = 0.0
        self.laser_trans.transform.rotation.z = 0.0
        self.laser_trans.transform.rotation.w = 1.0

        self.footprint_trans = tf2_ros.TransformStamped()
        self.footprint_trans.header.frame_id = self.get_parameter("base_footprint").value
        self.footprint_trans.child_frame_id = self.get_parameter("base_link").value
        self.footprint_trans.transform.translation.x = 0.0
        self.footprint_trans.transform.translation.y = 0.0
        self.footprint_trans.transform.translation.z = 0.0
        self.footprint_trans.transform.rotation.x = 0.0
        self.footprint_trans.transform.rotation.y = 0.0
        self.footprint_trans.transform.rotation.z = 0.0
        self.footprint_trans.transform.rotation.w = 1.0

        self.imu_trans = tf2_ros.TransformStamped()
        self.imu_trans.header.frame_id = self.get_parameter("base_link").value
        self.imu_trans.child_frame_id = self.get_parameter("imu_link").value
        self.imu_trans.transform.translation.x = 0.0
        self.imu_trans.transform.translation.y = 0.0
        self.imu_trans.transform.translation.z = 0.0
        self.imu_trans.transform.rotation.x = 0.0
        self.imu_trans.transform.rotation.y = 0.0
        self.imu_trans.transform.rotation.z = 0.0
        self.imu_trans.transform.rotation.w = 1.0

        self.br = tf2_ros.TransformBroadcaster(self)

        self.logger.info('Starting Sphero RVR')

    # =======================================================
    # Send an echo message to Sphero to check if it's connected
    # =======================================================
    async def sphero_init(self):
        await self.rvr.wake()
        await asyncio.sleep(2) # Give RVR time to wake up
        echo_response = await asyncio.wait_for(
            self.rvr.echo(
                data=[3, 1, 4],
                target=SpheroRvrTargets.primary.value
            ), timeout=10.0)
        self.logger.info('Echo response: ' + str(echo_response))

        # reset orientation
        await self.rvr.reset_yaw()
        #reset position
        await self.rvr.reset_locator_x_and_y()
        # Set the default control system for RC and drive with slew
        control_system_type = ControlSystemTypesEnum.control_system_type_rc_drive
        controller_id = ControlSystemIdsEnum.rc_drive_slew_mode
        await self.rvr.set_default_control_system_for_type(
            control_system_type = control_system_type, controller_id = controller_id)
        # set timeout for stopping
        await self.rvr.set_custom_control_system_timeout(command_timeout=1000)

        # Display status information
        nordic_main_application_version = await self.rvr.get_main_application_version(
            target=SpheroRvrTargets.primary.value)
        self.logger.info('Nordic main application version (target 1): ' +
            str(nordic_main_application_version))
        st_main_application_version = await self.rvr.get_main_application_version(
            target=SpheroRvrTargets.secondary.value)
        self.logger.info('ST main application version (target 2): ' +
            str(st_main_application_version))
        battery_percentage = await self.rvr.get_battery_percentage()
        self.logger.info('Battery percentage: ' + str(battery_percentage))
        battery_voltage_state = await self.rvr.get_battery_voltage_state()
        self.logger.info('Voltage state: ' + str(battery_voltage_state))
        state_info = '[{}, {}, {}, {}]'.format(
            '{}: {}'.format(VoltageStates.unknown.name, VoltageStates.unknown.value),
            '{}: {}'.format(VoltageStates.ok.name, VoltageStates.ok.value),
            '{}: {}'.format(VoltageStates.low.name, VoltageStates.low.value),
            '{}: {}'.format(VoltageStates.critical.name, VoltageStates.critical.value)
        )
        self.logger.info('Voltage states: ' + state_info)
        response = await self.rvr.get_active_control_system_id()
        controller_id = ControlSystemIdsEnum(response['controller_id'])
        self.logger.info('Active controller: {}'.format(controller_id.name))

    # =======================================================
    # Handler for received cmd_vel messages
    # =======================================================
    def initialize_rover(self):
        try:
            self.loop.run_until_complete(
                self.sphero_init()
            )
            return True
        except Exception as e:
            print(e)
            return False

    # =======================================================
    # Handler for received cmd_vel messages
    # =======================================================
    async def cmdvel_cb(self, vel_cmd):
        # convert to degress
        angular_vel_deg = vel_cmd.angular.z * 360.0 / (2.0 * pi)
        # $%^& Sphero has Y as forward... :(
        # Use X here and rotate in the TF messages.
        asyncio.create_task(self.rvr.drive_rc_si_units(
            linear_velocity=vel_cmd.linear.x,
            yaw_angular_velocity=angular_vel_deg,
            flags=0))

    # =======================================================
    # Check if all messages were received and publish them
    # =======================================================
    def check_if_need_to_send_msg(self, component):
        self.received_components.update(component)
        if self.received_components >= {'locator', 'quaternion', 'gyroscope',
                'velocity', 'accelerometer'}:
            self.received_components.clear()
            try:
                self.odom.header.stamp = self.get_clock().now().to_msg()
                self.pub_odom.publish(self.odom)

                self.odom_trans.header.stamp = self.odom.header.stamp

                self.odom_trans.transform.translation.x = self.position.x
                self.odom_trans.transform.translation.y = self.position.y
                self.odom_trans.transform.translation.z = self.position.z

                self.odom_trans.transform.rotation = self.orientation

                self.br.sendTransform(self.odom_trans)

                self.laser_trans.header.stamp = self.odom.header.stamp
                self.br.sendTransform(self.laser_trans)

                self.footprint_trans.header.stamp = self.odom.header.stamp
                self.br.sendTransform(self.footprint_trans)

                self.imu.header.stamp = self.odom.header.stamp
                self.imu.orientation = self.orientation
                self.imu.linear_acceleration = self.acceleration
                self.imu.angular_velocity = self.angular

                self.imu_pub.publish(self.imu)

                self.imu_trans.header.stamp = self.odom.header.stamp
                self.br.sendTransform(self.imu_trans)
            except Exception as e:
                self.logger.error("Exception publishing transformations: " + str(e))

    #=======================================================
    # Listener for Sphero messages
    #=======================================================
    # multiple messages can come in one packet
    def sphero_handler(self, data):
        try:
            received = set()
            if 'Locator' in data:
                received.add('locator')
                # Sphero see forward towards Y as opposed to ROS,
                # which see forward as X. Rotate 90 deg.
                self.position.x = data['Locator']['Y']
                self.position.y = -data['Locator']['X']
                self.position.z = 0.0

            if 'Quaternion' in data:
                received.add('quaternion')
                # Rotate 90 as per above.
                self.orientation.w = data['Quaternion']['W']
                self.orientation.x = data['Quaternion']['X']
                self.orientation.y = data['Quaternion']['Y']
                self.orientation.z = -data['Quaternion']['Z']

            if 'Gyroscope' in data:
                received.add('gyroscope')
                # convert to radians
                self.angular.x = data['Gyroscope']['X'] * 2.0 * pi / 360.0
                self.angular.y = data['Gyroscope']['Y'] * 2.0 * pi / 360.0
                self.angular.z = data['Gyroscope']['Z'] * 2.0 * pi / 360.0

            if 'Velocity' in data:
                received.add('velocity')
                self.linear.x = data['Velocity']['X']
                self.linear.y = data['Velocity']['Y']

            if 'Accelerometer' in data:
                received.add('accelerometer')
                self.acceleration.x = data['Accelerometer']['X']
                self.acceleration.y = data['Accelerometer']['Y']
                self.acceleration.z = data['Accelerometer']['Z']

            self.check_if_need_to_send_msg(received)

        except Exception as e:
            self.logger.error('Exception processing {}: {}'.format(data, e))

    #=======================================================
    # Listener for going to sleep
    #=======================================================
    async def sleep_handler(self):
        # Do not let the RVR sleep while we are connected
        try:
            await self.rvr.wake()
        except Exception as e:
            self.logger.error('Exception processing sleep: {}'.format(e))

    #=======================================================
    # Setup listeners
    #=======================================================
    async def ros_pump(self):
        while self.loop.is_running():
            try:
                rclpy.spin_once(self, timeout_sec=0)
                await asyncio.sleep(0.05)
            except Exception as e:
                self.logger.warn('Exception pumping ROS messages: ' + str(e))

    #=======================================================
    # Setup listeners
    #=======================================================
    async def setup_listeners(self):
        try:
            await self.rvr.sensor_control.add_sensor_data_handler(
                    service=RvrStreamingServices.locator,
                    handler=self.sphero_handler,
            )
            await self.rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.quaternion,
                handler=self.sphero_handler,
            )
            await self.rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.gyroscope,
                handler=self.sphero_handler,
            )
            await self.rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.velocity,
                handler=self.sphero_handler,
            )
            await self.rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.accelerometer,
                handler=self.sphero_handler,
            )
            await self.rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.imu,
                handler=self.sphero_handler,
            )

            await self.rvr.on_will_sleep_notify(handler=self.sleep_handler)

            await self.rvr.sensor_control.start(interval=75)

            # this will loop forever pumping ROS messages while
            # also sleeping to allow asyncio messages to be
            # processed.
            asyncio.create_task(self.ros_pump())
            #await self.ros_pump()
            self.logger.info('Listeners in place')
        except Exception as e:
            msg = "Sphero thread setup error: " + str(e)
            self.logger.error(msg)
            raise Exception(msg)

    # =======================================================
    # Thread for processing serial communication with Sphero
    # =======================================================
    def run(self):
        asyncio.ensure_future(self.setup_listeners())
        self.logger.info("Ready to roll!")
        while (not self.t_stop.is_set()):
            try:
                self.loop.run_forever()
            except KeyboardInterrupt:
                self.logger.info("Sphero thread detected keyboard interrupt")
                self.t_stop.set()
            except Exception as e:
                self.logger.error("Exception processing Sphero message:" + str(e))
        if self.loop.is_running():
            self.loop.close()
        self.loop.run_until_complete(
            asyncio.gather(
                self.rvr.drive_stop(),
                self.rvr.restore_initial_default_control_systems(),
                self.rvr.restore_default_control_system_timeout(),
                self.rvr.sensor_control.clear(),
                self.rvr.close()
            )
        )
        self.logger.info("Sphero thread ends")
