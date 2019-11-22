#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class GamepadParser(Node):

    def __init__(self):
        self.node_name = 'gamepad_parser_node'
        super().__init__(self.node_name)

        # Create Publishers
        self.rover_motion_cmd_pub_ = self.create_publisher(Twist, 'rover_motion_cmd', 10)

        # Create Subscriptions
        self.create_subscription(Joy, 'gamepad', self.gamepad_callback, 10)
 
        self.get_logger().info('{} started.'.format(self.node_name))

    def gamepad_callback(self, data):
        ## HANDLE BUTTONS
        # Handle Thrusters

        # Dis- or enable motors
        if data.buttons[9]: # START Key
            self.get_logger().info('START PRESSED')


        # Reset setpoint angle
        if data.buttons[8]: # BACK Key
            self.get_logger().info('BACK PRESSED')

        ## HANDLE AXES
        # Compute Left joystick heading angle
        self.get_logger().info('Axes: {} {}'.format(data.axes[0], data.axes[1]))

        if (abs(data.axes[0])+abs(data.axes[1])) > 0:
            # if math.sqrt(pow(data.axes[0],2) + pow(data.axes[1],2)) > 0.8:
                # self.yaw_goal = math.atan2(data.axes[0],data.axes[1])*180/math.pi
            self.get_logger().info('Publishing stuff')


            # Fill rover_motion_cmd message
            rover_motion_cmd_msg = Twist()
            rover_motion_cmd_msg.linear.x = 0.0
            rover_motion_cmd_msg.linear.y = 1.0
            rover_motion_cmd_msg.linear.z = 2.0
            
            rover_motion_cmd_msg.angular.x = 1.0
            rover_motion_cmd_msg.angular.y = 2.0
            rover_motion_cmd_msg.angular.z = 3.0

            self.rover_motion_cmd_pub_.publish(rover_motion_cmd_msg)
    
    def stop(self):
        rospy.loginfo("{} STOPPED.".format(self.node_name.upper()))


def main(args=None):
    rclpy.init(args=args)

    gamepad_parser = GamepadParser()

    rclpy.spin(gamepad_parser)

    gamepad_parser.stop()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gamepad_parser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()