#!/usr/bin/env python3

#use of pid to balance the pendulum
import rclpy 
from rclpy.node import Node
from rclpy.time import Time
import numpy as np

from custom_msgs.msg import TorqueInput, States

Kp = 0.2
Kd = 0.001
Ki = 4.0

class single_inverted_pendulum_balance(Node):
    
    def __init__(self):  
        self.prev_error = 0 
        self.integral_error = 0        

        super().__init__('simple inverted pendulum balancer')
        self.subscription = self.create_subscription(States,'/state_feedback',self.balance ,10) 
        self.publisher = self.create_publisher(TorqueInput,'/torque_input',10 )
        self.prev_time = self.get_clock().now()

    def time_diff(self, time1: Time, time2 : Time):
        return (time1.nanoseconds-time2.nanoseconds)/10e9

    def balance(self, msg: States):
        measured_value = msg.theta
        current_time = self.get_clock().now()
        error = np.pi - measured_value
        
        self.integral_error = self.integral_error + error
        derivative_error = (error - self.prev_error)/self.time_diff(current_time, self.prev_error)

        change = Kp*error + Ki*self.integral_error + Kd*derivative_error

        torque_input = TorqueInput()
        torque_input.torque_value = measured_value + change
        if msg.theta < 0:
            torque_input.torque_value *= -1
        self.publisher.publish(torque_input)

        self.prev_error = error
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)

    node = single_inverted_pendulum_balance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



        

