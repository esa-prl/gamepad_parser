#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from simple_rover_locomotion.srv import ChangeLocomotionMode

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class GamepadParser(Node):

    def __init__(self):
        self.node_name = 'gamepad_parser_node'
        super().__init__(self.node_name)

        # Init Params
        self.init_params()

        # Create Publishers
        self.rover_motion_cmd_pub_ = self.create_publisher(Twist, 'rover_motion_cmd', 10)
        
        # Request Locomotion Mode Service
        self.change_locomotion_mode_cli_ = self.create_client(ChangeLocomotionMode,'change_locomotion_mode')
        self.request = ChangeLocomotionMode.Request()
        self.client_futures = []

        # Create Subscriptions
        self.create_subscription(Joy, 'gamepad', self.gamepad_callback, 10)
 
        self.get_logger().info('{} started.'.format(self.node_name))

    def init_params(self):
        self.prev_data = Joy()


    def gamepad_callback(self, data):
        ## HANDLE BUTTONS

        # Check if button message has changed:
        if data.buttons == self.prev_data.buttons and data.axes == self.prev_data.axes:
            return
        else:
            self.prev_data = data

        # Dis- or enable motors
        if data.buttons[9]: # START Key
            self.get_logger().debug('START PRESSED')

            self.request.locomotion_mode = 'WHEELWALKING'

            self.add_request_to_queue(self.request)

        # Reset setpoint angle
        if data.buttons[8]: # BACK Key
            self.get_logger().debug('BACK PRESSED')

        ## HANDLE AXES
        # Check if any rover velocities axis are pressed and send command
        if data.axes[0] != 0.0 or data.axes[1] != 0.0 or data.axes[2] != 0.0:

            self.get_logger().debug('ROVER_MOTION_CMD_MSG_SENT!')

            # Fill rover_motion_cmd message
            rover_motion_cmd_msg = Twist()
            rover_motion_cmd_msg.linear.x = data.axes[0]
            rover_motion_cmd_msg.linear.y = -data.axes[1]
            rover_motion_cmd_msg.linear.z = 0.0
            
            rover_motion_cmd_msg.angular.x = 0.0
            rover_motion_cmd_msg.angular.y = 0.0
            rover_motion_cmd_msg.angular.z = data.axes[2]

            self.rover_motion_cmd_pub_.publish(rover_motion_cmd_msg)
            self.send_stop_command = True


    def add_request_to_queue(self, request):
        self.client_futures.append(self.change_locomotion_mode_cli_.call_async(self.request))

    def parse_future_result(self, future):
        # print(future.request())
        print(future.result().response)

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    res = f.result()
                    self.parse_future_result(f)
                else:
                    incomplete_futures.append(f)

            # self.get_logger().warn('{} incomplete futures.'.format(len(incomplete_futures)))
            
            self.client_futures = incomplete_futures

    def stop(self):
        rospy.loginfo("{} STOPPED.".format(self.node_name.upper()))


def main(args=None):
    rclpy.init(args=args)

    gamepad_parser = GamepadParser()

    gamepad_parser.spin()

    gamepad_parser.stop()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gamepad_parser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()