#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from custom_msgs.msg import TorqueInput, States

class single_inverted_pendulum_interface(Node):
    def __init__(self):   
        super().__init__('simple inverted pendulum interfacer')
        self.subscription = self.create_subscription(States,'/state_feedback',self.interface ,10) 
        self.publisher = self.create_publisher(TorqueInput,'/torque_input',10 )

    def interface(self, msg: States):
        self.get_logger().info(f"message: {msg}")
        torque_input = TorqueInput()
        torque_input.torque_value = 1.0
        if msg.theta < 0:
            torque_input.torque_value = -1.0
        self.publisher.publish(torque_input)


def main(args=None):
    rclpy.init(args=args)

    node = single_inverted_pendulum_interface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

