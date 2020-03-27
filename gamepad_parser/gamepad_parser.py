#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rover_msgs.srv import ChangeLocomotionMode

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class GamepadParser(Node):

    def __init__(self):
        # Init Node
        self.node_name = 'gamepad_parser_node'
        super().__init__(self.node_name)

        # Init Params
        self.init_params()

        # Create Publishers
        self.rover_motion_cmd_pub_ = self.create_publisher(Twist, 'rover_motion_cmd', 10)
        self.ptu_cmd_pub_ = self.create_publisher(Twist, 'ptu_cmd', 10)
        
        # Create Subscriptions
        self.create_subscription(Joy, 'gamepad', self.gamepad_callback, 10)

        # Request Locomotion Mode Service
        self.change_locomotion_mode_cli_ = self.create_client(ChangeLocomotionMode,'change_locomotion_mode')
        self.request = ChangeLocomotionMode.Request()
        self.client_futures = []
 
        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def init_params(self):
        self.prev_data = Joy()

        # Percentage of deadzone in which no axis change is being considered. [0, 1]
        self.deadzone_r = 0.2

        self.continuos_data_streaming = True

        # TODO: Find ratio that leads to realistic velocity values
        # Ratio from Joystick scalar to linear and angular velocities
        self.linear_speed_ratio = 0.5
        self.angular_speed_ratio = 0.5

        # Scaling with which the speed ratio can be changed during operations.
        # 0.1 = 10% increase, and reduction to 90% of previous value
        self.speed_ratio_scaling = 0.1

        # Ratio from Joystick scalar to PTU pan and tilt
        self.ptu_pan_speed_ratio = 0.5
        self.ptu_tilt_speed_ratio = 0.5

    def gamepad_callback(self, data):

        self.curr_data = data

        # Initialize the prev_data message the first time.
        if len(self.prev_data.buttons) == 0 or len(self.prev_data.axes) == 0:
            self.prev_data = data
            return

        ### HANDLE BUTTONS
        ## Mode Requests
        if self.button_pressed(3): # Y
            self.get_logger().info('Y PRESSED')

            # Services requests are added to the service queue
            self.request.new_locomotion_mode = 'TEST MODE'
            self.add_request_to_queue(self.request)

        if self.button_pressed(8): # BACK Key
            self.get_logger().info('BACK PRESSED')

        ## Velocity Scaling
        # LB
        if self.button_pressed(4):
            self.get_logger().info('LB PRESSED')
            self.linear_speed_ratio = self.linear_speed_ratio * (1 + self.speed_ratio_scaling)
        # LT
        if self.button_pressed(6):
            self.get_logger().info('LT PRESSED')
            self.linear_speed_ratio = self.linear_speed_ratio * (1 - self.speed_ratio_scaling)
        # RB
        if self.button_pressed(5):
            self.get_logger().info('RB PRESSED')
            self.angular_speed_ratio = self.angular_speed_ratio * (1 + self.speed_ratio_scaling)
        # RT
        if self.button_pressed(7):
            self.get_logger().info('RT PRESSED')
            self.angular_speed_ratio = self.angular_speed_ratio * (1 - self.speed_ratio_scaling)

        ### HANDLE AXES
        ## Steering
        if self.handle_axis(0) or self.handle_axis(1) or self.handle_axis(2) or self.any_button_pressed([4, 5, 6, 7]):
            # Fill rover_motion_cmd message
            rover_motion_cmd_msg = Twist()
            rover_motion_cmd_msg.linear.x = data.axes[1] * self.linear_speed_ratio
            rover_motion_cmd_msg.linear.y = data.axes[0]* self.linear_speed_ratio
            rover_motion_cmd_msg.linear.z = 0.0
            
            rover_motion_cmd_msg.angular.x = 0.0
            rover_motion_cmd_msg.angular.y = 0.0
            rover_motion_cmd_msg.angular.z = data.axes[2] * self.angular_speed_ratio

            self.rover_motion_cmd_pub_.publish(rover_motion_cmd_msg)
            self.get_logger().debug('ROVER_MOTION_CMD_MSG SENT!')

        ## PTU
        if self.handle_axis(4) or self.handle_axis(5):
            # Fill ptu_cmd message
            ptu_cmd_msg = Twist()
            ptu_cmd_msg.linear.x = 0.0
            ptu_cmd_msg.linear.y = 0.0
            ptu_cmd_msg.linear.z = 0.0
            
            ptu_cmd_msg.angular.x = data.axes[5] * self.ptu_tilt_speed_ratio
            ptu_cmd_msg.angular.y = data.axes[4] * self.ptu_pan_speed_ratio
            ptu_cmd_msg.angular.z = 0.0

            self.ptu_cmd_pub_.publish(ptu_cmd_msg)
            self.get_logger().debug('PTU_CMD_MSG SENT!')

        self.prev_data = data


    def add_request_to_queue(self, request):
        self.client_futures.append(self.change_locomotion_mode_cli_.call_async(self.request))

    def parse_future_result(self, future):
        print(future.result().response)

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            # Necessary to call the services after the spin so they can resolve. If they don't, then they are added to a queue and tried the next time.
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    res = f.result()
                    self.parse_future_result(f)
                else:
                    incomplete_futures.append(f)

            # self.get_logger().warn('{} incomplete futures.'.format(len(incomplete_futures)))
            self.client_futures = incomplete_futures

    def button_pressed(self, index):
        return self.prev_data.buttons[index] != self.curr_data.buttons[index] and self.curr_data.buttons[index]

    def any_button_pressed(self, buttons_index):
        for index in buttons_index:
            if self.button_pressed(index):
                return True
        return False

    # Compares current axis reading with previous axis reading
    # Only applies if joystick is outside of the deadzone
    def handle_axis(self, index):
        # Check if data should be streamed at all times or only if it changed.
        if self.continuos_data_streaming:
            return abs(self.curr_data.axes[index]) >= self.deadzone_r
        else:
            return self.prev_data.axes[index] != self.curr_data.axes[index] and abs(self.curr_data.axes[index]) >= self.deadzone_r

    def stop(self):
        rospy.loginfo("{} STOPPED.".format(self.node_name.upper()))


def main(args=None):
    rclpy.init(args=args)

    gamepad_parser = GamepadParser()

    gamepad_parser.spin()

    # TODO: catch CTRL+C exception and close node gracefully.
    gamepad_parser.stop()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gamepad_parser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()