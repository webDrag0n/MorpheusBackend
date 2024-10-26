#!/usr/bin/env python

import random
import rclpy

from rclpy.node import Node

from unity_robotics_demo_msgs.msg import H1ControlCommand

class H1ControlPublisher(Node):

    def __init__(self):
        super().__init__('h1_control_publisher')
        self.publisher_ = self.create_publisher(H1ControlCommand, 'H1ControlCommand', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.motor_target_angle = 0

        self.do_publish()

    def do_publish(self, h1_control_command=None):
        if h1_control_command == None:
            return
        # h1_control_command = H1ControlCommand()
        # h1_control_command.left_hip_yaw = random.random() * 90
        # h1_control_command.left_hip_roll = random.random() * 90
        # h1_control_command.left_hip_pitch = random.random() * 90
        # h1_control_command.left_knee = random.random() * 90
        # h1_control_command.left_ankle = random.random() * 90
        # h1_control_command.right_hip_yaw = random.random() * 90
        # h1_control_command.right_hip_roll = random.random() * 90
        # h1_control_command.right_hip_pitch = random.random() * 90
        # h1_control_command.right_knee = random.random() * 90
        # h1_control_command.right_ankle = random.random() * 90
        # h1_control_command.torso = random.random() * 90
        # h1_control_command.left_shoulder_pitch = random.random() * 90
        # h1_control_command.left_shoulder_roll = random.random() * 90
        # h1_control_command.left_shoulder_yaw = random.random() * 90
        # h1_control_command.left_elbow = random.random() * 90
        # h1_control_command.right_shoulder_pitch = random.random() * 90
        # h1_control_command.right_shoulder_roll = random.random() * 90
        # h1_control_command.right_shoulder_yaw = random.random() * 90
        # h1_control_command.right_elbow = random.random() * 90
        
        self.get_logger().info(f'Publishing: {h1_control_command}')
        self.publisher_.publish(h1_control_command)
        
    def timer_callback(self):
        # quit()
        pass


def main(args=None):
    rclpy.init(args=args)

    color_pub = H1ControlPublisher()

    while rclpy.ok():
        rclpy.spin_once(color_pub)

    #color_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
