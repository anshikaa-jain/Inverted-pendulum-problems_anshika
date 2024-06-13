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

class single_inverted_pendulum_swing_up(Node):
    
    def __init__(self):  
        self.prev_value = 0
        self.prev_error = 0 
        self.integral_error = 0
        self.prev_torque = 5         #max_torque

        super().__init__('simple inverted pendulum balancer')
        self.subscription = self.create_subscription(States,'/state_feedback',self.swing_up ,10) 
        self.publisher = self.create_publisher(TorqueInput,'/torque_input',10 )

        self.prev_time = self.get_clock().now()

    def time_diff(self, time1: Time, time2 : Time):
        return (time1.nanoseconds-time2.nanoseconds)/10e9

    def swing_up(self, msg: States):
        measured_value = msg.theta
        current_time = self.get_clock().now()

        torque_input = TorqueInput()

        #if change in angles is greater than 0.1 then torque = 0
        if (measured_value-self.prev_value) >= 0.1:
            torque_input.torque_value = 0.0

        #if angle is less than 150 and the difference in prev angle and curr angle is less than 0.001 then reverse the torque    
        elif (msg.theta) < 5*np.pi /6 :
            if (measured_value-self.prev_value) < 0.001 :
                self.prev_torque *= -1 
            torque_input.torque_value = self.prev_torque

        #else PID
        else: 
            error = np.pi - measured_value    # pi is desired value
            
            self.integral_error = self.integral_error + error
            derivative_error = (error - self.prev_error)/self.time_diff(current_time, self.prev_error)

            change = Kp*error + Ki*self.integral_error + Kd*derivative_error

            torque_input.torque_value = measured_value + change
            if msg.theta < 0:
                torque_input.torque_value *= -1

        self.publisher.publish(torque_input)
        self.prev_value = measured_value
        self.prev_error = error
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)

    node = single_inverted_pendulum_swing_up()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



        

