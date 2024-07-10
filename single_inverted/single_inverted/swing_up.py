#!/usr/bin/env python3

#use of pid to balance the pendulum
import rclpy 
from rclpy.node import Node
from rclpy.time import Time
import numpy as np

from custom_msgs.msg import TorqueInput, States

Kp = 15.0
Kd = 5.0
Ki = 0.002
dt = 1/500
class single_inverted_pendulum_swing_up(Node):
    
    def __init__(self):  
        super().__init__('simple_inverted_pendulum_balancer')
        self.subscription = self.create_subscription(States,'/state_feedback',self.swing_up ,10) 
        self.publisher = self.create_publisher(TorqueInput,'/torque_input',10 )

        self.prev_time = self.get_clock().now()
        self.prev_error = 0 
        self.integral_error = 0

    def time_diff(self, time1: Time, time2: Time) -> float:
        time1_msg = time1.to_msg()
        time2_msg = time2.to_msg()
        sec_diff = time1_msg.sec - time2_msg.sec
        nanosec_diff = time1_msg.nanosec - time2_msg.nanosec
        return (sec_diff + nanosec_diff / 1e9)

    def swing_up(self, msg: States):
        measured_value = abs(msg.theta)
        current_time = self.get_clock().now()
        torque_input = TorqueInput()
        

        if (msg.theta > np.pi/2 or msg.theta < -(np.pi/2)):
            error = np.pi - measured_value
            self.integral_error = self.integral_error + error
            derivative_error = (error - self.prev_error)/ self.time_diff(current_time, self.prev_time)


            change = Kp*error + Ki*self.integral_error + Kd*derivative_error

            torque_input.torque_value = measured_value + change

            if msg.theta < 0:
                torque_input.torque_value *= -1
            if torque_input.torque_value > 5:
                torque_input.torque_value = 5.0
            if torque_input.torque_value < -5:
                torque_input.torque_value = -5.0
        
            self.prev_error = error
            self.prev_time = current_time

        if (msg.theta <= np.pi/2 and msg.theta >= -(np.pi/2)):
                if msg.theta_dot > 0 :
                    torque_input.torque_value = 3.0
                else:
                    torque_input.torque_value = -3.0

        self.publisher.publish(torque_input)

def main(args=None):
    rclpy.init(args=args)

    node = single_inverted_pendulum_swing_up()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



        

