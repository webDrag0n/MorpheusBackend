#!/usr/bin/env python

import rospy
from iserve_msgs.msg import JointCommand
from bezier_msgs.srv import NextPosition, NextPositionResponse
from std_msgs.msg import Header
import numpy as np
from scipy.special import comb

# 定义Bezier曲线的控制点
control_points = np.array([
    [0.2790, -0.2135, 0.0881],  # 起始点
    [0.4, -0.4, 0.3]   # 结束点
]).T
TOTAL_T = 1.0

# def bezier_curve(points, t):
#     # 确保输入点是numpy数组
#     p0 = np.array(points[0])
#     p1 = np.array(points[1])
    
#     # 计算Bezier曲线上的点
#     point = ((TOTAL_T - t) * p0 + t * p1)/TOTAL_T
#     return point.tolist()

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

total_t = 0.0
dt = 0.01

def handle_get_next_position(req):
    global total_t, dt
    target_point = bezier(control_points, total_t)
    is_ok = False
    total_t += dt
    if total_t <= 0 or total_t >= TOTAL_T:
        is_ok = True
        dt = -dt
    return NextPositionResponse(target_point, is_ok)

def position_server():
    rospy.init_node('position_server')
    s = rospy.Service('get_next_position', NextPosition, handle_get_next_position)
    rospy.loginfo("Ready to provide positions...")
    rospy.spin()

def publish_joint_command():
    # 初始化節點
    rospy.init_node('joint_command_publisher', anonymous=True)

    # 創建一個發布者，發布到 'joint_command' topic
    pub = rospy.Publisher('/right/armJoint/joint_command', JointCommand, queue_size=10)

    # 設置發布頻率
    rate = rospy.Rate(10)  # 10 Hz

    # 創建一個 JointCommand 消息
    joint_command = JointCommand()

    # 填充消息的 header 部分
    joint_command.header = Header()
    joint_command.header.seq = 0
    joint_command.header.stamp = rospy.Time.now()
    joint_command.header.frame_id = "base_link"

    # 填充消息的其他部分
    joint_command.seq = 0
    joint_command.stamp = rospy.Time.now()
    joint_command.position = [0.1, 0.2, 0.3, 0.4]  # 示例位置

    while not rospy.is_shutdown():
        # 更新消息的時間戳
        joint_command.header.stamp = rospy.Time.now()
        joint_command.stamp = rospy.Time.now()

        # 發布消息
        pub.publish(joint_command)

        # 增加序列號
        joint_command.header.seq += 1
        joint_command.seq += 1

        # 休眠以達到所需的發布頻率
        rate.sleep()

if __name__ == '__main__':
    try:
        # publish_joint_command()
        position_server()
    except rospy.ROSInterruptException:
        pass
