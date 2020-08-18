#!/usr/bin/env python

from geometry_msgs.msg import Twist

from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


from rover_msgs.srv import ChangeLocomotionMode

from sensor_msgs.msg import Joy


class GamepadParser(Node):
    """Parse gamepad inputs."""

    # TODO: pass "allow undeclared parameters" making the parameter init easier
    def __init__(self):
        """Init Node."""
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
        self.change_locomotion_mode_cli = self.create_client(ChangeLocomotionMode,
                                                             'change_locomotion_mode')

        # Nav to Pose Action Client
        self.nav_to_pose_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.request = ChangeLocomotionMode.Request()
        self.client_futures = []

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def init_params(self):
        """Initialize Parameters."""
        self.prev_data = Joy()

        # Percentage of deadzone in which no axis change is being considered. [0, 1]
        self.declare_parameter('deadzone', 0.2)
        self.deadzone = self.get_parameter('deadzone').value

        self.declare_parameter('continuous_data_streaming_locomotion', True)
        self.continuous_data_streaming_locomotion = self.get_parameter(
            'continuous_data_streaming_locomotion').value

        self.declare_parameter('continuous_data_streaming_ptu', True)
        self.continuous_data_streaming_ptu = self.get_parameter(
            'continuous_data_streaming_ptu').value

        # TODO: Find ratio that leads to realistic velocity values
        # Ratio from Joystick scalar to linear and angular velocities
        self.linear_speed_ratio = 0.1
        self.angular_speed_ratio = 0.1

        # Scaling with which the speed ratio can be changed during operations.
        # 0.1 = 10% increase, and reduction to 90% of previous value
        self.speed_ratio_scaling = 0.3

        # Ratio from Joystick scalar to PTU pan and tilt
        self.ptu_pan_speed_ratio = 0.5
        self.ptu_tilt_speed_ratio = 0.5

    def gamepad_callback(self, data):
        """Handle the messages received from the gamepad."""
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
            self.get_logger().info('BACK PRESSED - Requesting Pose')
            self.request_pose()

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
        if self.continuous_data_streaming_locomotion or self.axis_changed(0) or self.axis_changed(
                1) or self.axis_changed(2) or self.any_button_pressed([4, 5, 6, 7]):
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
        if self.continuous_data_streaming_ptu or self.axis_changed(4) or self.axis_changed(5):
            # Fill ptu_cmd message
            ptu_cmd_msg = Twist()
            ptu_cmd_msg.linear.x = 0.0
            ptu_cmd_msg.linear.y = 0.0
            ptu_cmd_msg.linear.z = 0.0

            ptu_cmd_msg.angular.x = 0.0
            ptu_cmd_msg.angular.y = data.axes[5] * self.ptu_tilt_speed_ratio
            ptu_cmd_msg.angular.z = data.axes[4] * self.ptu_pan_speed_ratio

            self.ptu_cmd_pub.publish(ptu_cmd_msg)
            self.get_logger().debug('PTU_CMD_MSG SENT!')

        self.prev_data = data

    def request_pose(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = 1.5
        goal_msg.pose.pose.position.y = 0.5
        goal_msg.pose.pose.position.z = 0.25
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_to_pose_cli.wait_for_server()

        self.nav_to_pose_cli.send_goal_async(goal_msg)

    def add_request_to_queue(self, request):
        """Add a call to the futures list, which is checked after each spin."""
        self.client_futures.append(self.change_locomotion_mode_cli.call_async(self.request))

    def parse_future_result(self, result):
        """Do something with the response of the service callback."""
        if result.success:
            self.get_logger().debug('Locomotion mode change result received.')
        if not result.success:
            self.get_logger().warn('Locomotion mode change failed. Msg: {}.'.format(result.message))

    def button_pressed(self, index):
        """Check if a button was pressed by comparing it's current state to it's previous state."""
        return self.prev_data.buttons[index] != self.curr_data.buttons[
            index] and self.curr_data.buttons[index]

    def any_button_pressed(self, buttons_index):
        """Check if a button from the supplied index selection is pressed."""
        for index in buttons_index:
            if self.button_pressed(index):
                return True
        return False

    def axis_changed(self, index):
        """Compare current axis reading with previous axis reading."""
        return self.prev_data.axes[index] != self.curr_data.axes[index]

    def spin(self):
        """Check if the service calls resolved after each spin."""
        while rclpy.ok():
            rclpy.spin_once(self)
            # Necessary to call the services after the spin so they can resolve.
            # If they don't, then they are added to a queue and tried the next time.
            incomplete_futures = []

            for f in self.client_futures:
                if f.done():
                    try:
                        res = f.result()
                    except Exception as e:
                        self.get_logger().warn(
                            'Service Call to ChangeLocomotionMode failed {}.'.format(e))
                    else:
                        self.parse_future_result(res)
                else:
                    incomplete_futures.append(f)

            if len(incomplete_futures) > 0:
                self.get_logger().debug('{} incomplete futures.'.format(len(incomplete_futures)))

            self.client_futures = incomplete_futures

    def stop(self):
        """Shut down method."""
        self.get_logger().info('\t{} STOPPED.'.format(self.node_name.upper()))


def main(args=None):
    """Run node."""
    rclpy.init(args=args)

    gamepad_parser = GamepadParser()

    # Workaround since there is a bug in stopping python notes properly
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
