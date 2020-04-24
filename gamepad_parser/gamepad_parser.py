#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rover_msgs.srv import ChangeLocomotionMode

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class GamepadParser(Node):
    # TODO: pass "allow undeclared parameters" making the parameter init easier

    def __init__(self):
        # Init Node
        self.node_name = 'gamepad_parser_node'
        super().__init__(self.node_name)

        # Init Params
        self.init_params()

        # Create Publishers
        self.rover_motion_cmd_pub = self.create_publisher(Twist, 'rover_motion_cmd', 10)
        self.ptu_cmd_pub = self.create_publisher(Twist, 'ptu_cmd', 10)

        # Create Subscriptions
        self.create_subscription(Joy, 'gamepad', self.gamepad_callback, 10)

        # Request Locomotion Mode Service
        self.change_locomotion_mode_cli = self.create_client(ChangeLocomotionMode, 'change_locomotion_mode')
        self.request = ChangeLocomotionMode.Request()
        self.client_futures = []

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def init_params(self):
        self.prev_data = Joy()

        # Percentage of deadzone in which no axis change is being considered. [0, 1]
        self.declare_parameter('deadzone')
        self.declare_parameter('continuous_data_streaming')

        self.deadzone = self.get_parameter('deadzone').value
        self.continuous_data_streaming = self.get_parameter('continuous_data_streaming').value

        # TODO: There should be a better way to have a default value
        if self.deadzone == None:
            self.deadzone = 0.2
            self.get_logger().warn('Deadzone was not declared. Used default value {} instead.'.format(self.deadzone))

        if self.continuous_data_streaming == None:
            self.continuous_data_streaming = True
            self.get_logger().warn('continuos_data_streaming was not declared. Used default value {} instead.'.format(self.continuous_data_streaming))

        # TODO: Find ratio that leads to realistic velocity values
        # Ratio from Joystick scalar to linear and angular velocities
        self.linear_speed_ratio = 0.5
        self.angular_speed_ratio = 0.5

        # Scaling with which the speed ratio can be changed during operations.
        # 0.1 = 10% increase, and reduction to 90% of previous value
        self.speed_ratio_scaling = 0.3

        # Ratio from Joystick scalar to PTU pan and tilt
        self.ptu_pan_speed_ratio = 0.5
        self.ptu_tilt_speed_ratio = 0.5

    def gamepad_callback(self, data):
        self.curr_data = data

        # Apply Deadzone to axis
        for index, axis in enumerate(self.curr_data.axes):
            # Sets axis value to 0 if it's
            if abs(axis) <= self.deadzone:
                self.curr_data.axes[index] = 0

        # Initialize the prev_data message the first time.
        if len(self.prev_data.buttons) == 0 or len(self.prev_data.axes) == 0:
            self.prev_data = data
            return

        # HANDLE BUTTONS
        # Mode Requests
        if self.button_pressed(3):  # Y
            # self.get_logger().info('Y PRESSED')
            # Services requests are added to the service queue
            self.request.new_locomotion_mode = 'wheel_walking_node'
            self.add_request_to_queue(self.request)

        if self.button_pressed(2):  # B
            self.request.new_locomotion_mode = 'stop_mode_node'
            self.add_request_to_queue(self.request)

        if self.button_pressed(0):  # X
            # self.get_logger().info('X PRESSED')
            self.request.new_locomotion_mode = 'simple_rover_locomotion_node'
            self.add_request_to_queue(self.request)

        if self.button_pressed(8):  # BACK Key
            self.get_logger().info('BACK PRESSED - No functions')

        # Velocity Scaling
        # LB
        if self.button_pressed(4):
            # self.get_logger().info('LB PRESSED')
            self.linear_speed_ratio = self.linear_speed_ratio * (1 + self.speed_ratio_scaling)
        # LT
        if self.button_pressed(6):
            # self.get_logger().info('LT PRESSED')
            self.linear_speed_ratio = self.linear_speed_ratio * (1 - self.speed_ratio_scaling)
        # RB
        if self.button_pressed(5):
            # self.get_logger().info('RB PRESSED')
            self.angular_speed_ratio = self.angular_speed_ratio * (1 + self.speed_ratio_scaling)
        # RT
        if self.button_pressed(7):
            # self.get_logger().info('RT PRESSED')
            self.angular_speed_ratio = self.angular_speed_ratio * (1 - self.speed_ratio_scaling)

        # HANDLE AXES
        # Steering
        if self.continuous_data_streaming or self.axis_changed(0) or self.axis_changed(1) or self.axis_changed(2) or self.any_button_pressed([4, 5, 6, 7]):
            # Fill rover_motion_cmd message
            rover_motion_cmd_msg = Twist()
            rover_motion_cmd_msg.linear.x = data.axes[1] * self.linear_speed_ratio
            rover_motion_cmd_msg.linear.y = data.axes[0] * self.linear_speed_ratio
            rover_motion_cmd_msg.linear.z = 0.0

            rover_motion_cmd_msg.angular.x = 0.0
            rover_motion_cmd_msg.angular.y = 0.0
            rover_motion_cmd_msg.angular.z = data.axes[2] * self.angular_speed_ratio

            self.rover_motion_cmd_pub.publish(rover_motion_cmd_msg)
            self.get_logger().debug('ROVER_MOTION_CMD_MSG SENT!')

        # PTU
        if self.continuous_data_streaming or self.axis_changed(4) or self.axis_changed(5):
            # Fill ptu_cmd message
            ptu_cmd_msg = Twist()
            ptu_cmd_msg.linear.x = 0.0
            ptu_cmd_msg.linear.y = 0.0
            ptu_cmd_msg.linear.z = 0.0

            ptu_cmd_msg.angular.x = data.axes[5] * self.ptu_tilt_speed_ratio
            ptu_cmd_msg.angular.y = data.axes[4] * self.ptu_pan_speed_ratio
            ptu_cmd_msg.angular.z = 0.0

            self.ptu_cmd_pub.publish(ptu_cmd_msg)
            self.get_logger().debug('PTU_CMD_MSG SENT!')

        self.prev_data = data

    # Adds a call to the futures list, which is checked after each spin.

    def add_request_to_queue(self, request):
        self.client_futures.append(self.change_locomotion_mode_cli.call_async(self.request))

    # Does something with the response of the service callback

    def parse_future_result(self, result):
        if result.success:
            self.get_logger().debug('Locomotion mode change result received.')
        if not result.success:
            self.get_logger().warn('Locomotion mode change failed. Msg: {}.'.format(result.msg))

    # Checks if a button was pressed by comparing it's current state to it's previous state

    def button_pressed(self, index):
        return self.prev_data.buttons[index] != self.curr_data.buttons[index] and self.curr_data.buttons[index]

    # Checks if a button from the supplied index selection is pressed.

    def any_button_pressed(self, buttons_index):
        for index in buttons_index:
            if self.button_pressed(index):
                return True
        return False

    # Compares current axis reading with previous axis reading

    def axis_changed(self, index):
        return self.prev_data.axes[index] != self.curr_data.axes[index]

    # Defines custom spin function, that checks if the service calls resolved after each spin.

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            # Necessary to call the services after the spin so they can resolve. If they don't, then they are added to a queue and tried the next time.
            incomplete_futures = []

            for f in self.client_futures:
                if f.done():
                    try:
                        res = f.result()
                    except Exception as e:
                        self.get_logger().warn('Service Call to ChangeLocomotionMode failed {}.'.format(e))
                    else:
                        self.parse_future_result(res)
                else:
                    incomplete_futures.append(f)

            if len(incomplete_futures) > 0:
                self.get_logger().debug('{} incomplete futures.'.format(len(incomplete_futures)))

            self.client_futures = incomplete_futures

    def stop(self):
        self.get_logger().info('\t{} STOPPED.'.format(self.node_name.upper()))


def main(args=None):
    rclpy.init(args=args)

    gamepad_parser = GamepadParser()

    try:
        gamepad_parser.spin()
    except KeyboardInterrupt:
        pass
    finally:
        gamepad_parser.stop()
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        gamepad_parser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
