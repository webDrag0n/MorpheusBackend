
import time
from sys import argv
import os
from os.path import dirname, join, abspath
import math

import numpy as np
from numpy.linalg import norm, solve
from scipy.special import comb

import pinocchio
import rclpy
from rclpy.node import Node
from unity_robotics_demo_msgs.msg import H1ControlCommand
from h1_control_publisher import H1ControlPublisher

# This path refers to Pinocchio source code but you can define your own directory here.
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")
# You should change here to set up your own URDF file or just pass it as an argument of this example.
urdf_filename = ("./right_h1_arm.urdf")
# Load the urdf model
ik_model = pinocchio.buildModelFromUrdf(urdf_filename)
# Create data required by the algorithms
ik_data = ik_model.createData()
# Sample a random configuration
q=pinocchio.neutral(ik_model)

eps=1e-4
IT_MAX=1000
DT=1e-1
damp=1e-12
JOINT_ID = 5

total_t = 0.0
dt = 0.01

# 定义Bezier曲线的控制点
control_points = np.array([
    [0.2790, -0.2135, 0.0881],  # 起始点
    [0.4, -0.4, 0.3]   # 结束点
    # [0.3, -0.215, 0.09],  # 起始点
    # [0.4, -0.215, 0.3]   # 结束点
]).T
TOTAL_T = 1.0

def inverse_kinematics(pos):
    global q
    oMdes = pinocchio.SE3(np.eye(3), np.array(pos))
    i = 0

    while True:
        pinocchio.forwardKinematics(ik_model, ik_data, q)
        iMd = ik_data.oMi[JOINT_ID].actInv(oMdes)
        err = pinocchio.log(iMd).vector  # in joint frame
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
        J = pinocchio.computeJointJacobian(ik_model, ik_data, q, JOINT_ID)  # in joint frame
        J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pinocchio.integrate(ik_model, q, v * DT)
        i += 1
    q_list = q.flatten().tolist()
    return [q_list[1], q_list[2], q_list[4], q_list[5]]

def bezier(vertices, t):
    NumPoint = vertices.shape[1] - 1  # 点的个数

    x = (1 - t)**NumPoint * vertices[0, 0]
    y = (1 - t)**NumPoint * vertices[1, 0]
    z = (1 - t)**NumPoint * vertices[2, 0]
    for j in range(NumPoint):
        w = np.math.factorial(NumPoint) / (np.math.factorial(j) * np.math.factorial(NumPoint - j)) * (1 - t)**(NumPoint - j) * t**j
        x += w * vertices[0, j + 1]
        y += w * vertices[1, j + 1]
        z += w * vertices[2, j + 1]

    return x, y, z

def handle_get_next_position():
    global total_t, dt
    target_point = bezier(control_points, total_t)
    is_ok = False
    total_t += dt
    if total_t <= 0 or total_t >= TOTAL_T:
        is_ok = True
        dt = -dt
    return target_point


def main(args=None):
    rclpy.init(args=args)

    h1_pub = H1ControlPublisher()
    h1_control_command = H1ControlCommand()

    while rclpy.ok():
        target_point = handle_get_next_position()
        joint_angles = inverse_kinematics(target_point)
        h1_control_command.right_shoulder_pitch = joint_angles[0]
        h1_control_command.right_shoulder_roll = joint_angles[1]
        h1_control_command.right_shoulder_yaw = joint_angles[2]
        h1_control_command.right_elbow = joint_angles[3]
        
        # h1_control_command.right_shoulder_pitch = -0.9
        # h1_control_command.right_shoulder_roll = 0.0
        # h1_control_command.right_shoulder_yaw = 0.0
        # h1_control_command.right_elbow = 0.0
        h1_pub.do_publish(h1_control_command)
        rclpy.spin_once(h1_pub)

    #color_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
