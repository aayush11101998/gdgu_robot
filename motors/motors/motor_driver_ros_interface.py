#!/usr/bin/env python3

# Rhino Motor Driver (RMCS 2303) - ROS INTERFACE
# ----------------------------------------------
# This script provides a ROS interface for controlling the Rhino Motor Driver (RMCS 2303).
# It allows for the control of two motors (left and right) using ROS topics.

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Int32, UInt16, UInt32, Float32
from . import motor_driver


class MotorClass:
    def __init__(self, node, port_name, slave_id, encoder_topic_name, velocity_topic_name):
        self.node = node
        self.motor = motor_driver.Controller(port_name, slave_id)
        self.slave_id = slave_id

        # --- USE SINGLE UNDERSCORES ---
        self._time_delay = 0.001
        self._previous_velocity = 0.0 # Initialize as float
        self._current_velocity = 0.0  # Initialize as float
        self._status_rotation_direction = ""
        self._velocity_scale_factor = 19 # Keep this scale factor
        # self._encoder_scale_factor = 0.5 # Not currently used, but corrected
        # --- END SINGLE UNDERSCORES ---

        # Initial encoder read (ensure method exists and handles potential None)
        self.encoder_ticks_32bit = 0 # Initialize
        try:
            initial_ticks = self.motor.get_position_32bit()
            if initial_ticks is not None:
                self.encoder_ticks_32bit = int(initial_ticks)
        except Exception as e:
            self.node.get_logger().warn(f"Motor {self.slave_id}: Failed to get initial encoder position: {e}")


        self.encoder_ticks_pub_32bit = self.node.create_publisher(UInt32, encoder_topic_name + "_32bit", 10)
        self.node.create_subscription(Float32, velocity_topic_name, self.velocity_callback, 10)
        self.timer = self.node.create_timer(0.1, self.encoder_transmitter)


    def velocity_callback(self, data):
        motor_velocity_radians_per_second = data.data
        # --- USE SINGLE UNDERSCORE ---
        self.node.get_logger().debug(f'Motor {self.slave_id}: Received velocity: {motor_velocity_radians_per_second:.2f}')

        # --- USE SINGLE UNDERSCORE ---
        scaled_velocity = abs(self._velocity_scale_factor * motor_velocity_radians_per_second)

        new_direction = ""
        command_sent = False

        # --- USE SINGLE UNDERSCORE ---
        if motor_velocity_radians_per_second < -0.001:
            # --- USE SINGLE UNDERSCORE --- (and compare scaled_velocity directly)
            if self._status_rotation_direction != "CW" or abs(scaled_velocity - self._current_velocity) > 0.1:
                self.node.get_logger().info(f'Motor {self.slave_id}: Sending CW command, speed: {scaled_velocity:.2f}')
                self.motor.set_speed(scaled_velocity)
                self.motor.turn_motor_cw()
                new_direction = "CW"
                command_sent = True
        # --- USE SINGLE UNDERSCORE ---
        elif motor_velocity_radians_per_second > 0.001:
            # --- USE SINGLE UNDERSCORE --- (and compare scaled_velocity directly)
             if self._status_rotation_direction != "CCW" or abs(scaled_velocity - self._current_velocity) > 0.1:
                self.node.get_logger().info(f'Motor {self.slave_id}: Sending CCW command, speed: {scaled_velocity:.2f}')
                self.motor.set_speed(scaled_velocity)
                self.motor.turn_motor_ccw()
                new_direction = "CCW"
                command_sent = True
        else: # Velocity is effectively zero
            # --- USE SINGLE UNDERSCORE ---
            if self._status_rotation_direction != "":
                self.node.get_logger().info(f'Motor {self.slave_id}: Sending Brake command')
                self.motor.brake()
                # self.motor.set_speed(0) # Optional: if brake doesn't set speed to 0
                new_direction = ""
                command_sent = True
                scaled_velocity = 0.0

        if command_sent:
             # --- USE SINGLE UNDERSCORE ---
             self._status_rotation_direction = new_direction
             # --- USE SINGLE UNDERSCORE ---
             self._current_velocity = scaled_velocity # Store the actual target speed
             # --- USE SINGLE UNDERSCORE ---
             self._previous_velocity = self._current_velocity # Align after command


    def encoder_transmitter(self):
        # (encoder_transmitter remains the same as the previous fix)
        try:
            enc_ticks = self.motor.get_position_32bit()
            if enc_ticks is not None:
                self.encoder_ticks_32bit = enc_ticks
                msg = UInt32()
                msg.data = int(self.encoder_ticks_32bit)
                self.encoder_ticks_pub_32bit.publish(msg)
            else:
                 self.node.get_logger().warn(f'Motor {self.slave_id}: Failed to get encoder ticks (returned None)')
        except Exception as e:
             self.node.get_logger().error(f'Motor {self.slave_id}: Error in encoder_transmitter: {e}')


def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('motor_ros_interface')

    left_motor = MotorClass(node, "/dev/ttyUSB0", 7, "lwheel_ticks", "lwheel_vtarget")
    right_motor = MotorClass(node, "/dev/ttyUSB1", 7, "rwheel_ticks", "rwheel_vtarget")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping Motors before exiting ...")
        left_motor.motor.brake()
        right_motor.motor.brake()
        del left_motor
        del right_motor
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
