#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from iserve_msgs.msg import JointCommand
from bezier_msgs.srv import NextPosition
import mujoco
import mujoco.viewer
import time
import os
import numpy as np
from scipy.special import comb

import pinocchio
from numpy.linalg import norm, solve
from sys import argv
from os.path import dirname, join, abspath
import math

ws_path = os.environ.get('ROS_PACKAGE_PATH').split(':')[0]
model = mujoco.MjModel.from_xml_path(ws_path + '/iserve_controller/config/urdf/h1_description/mjcf/scene.xml')
data = mujoco.MjData(model)

# This path refers to Pinocchio source code but you can define your own directory here.
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")
# You should change here to set up your own URDF file or just pass it as an argument of this example.
urdf_filename = ("/home/joshua/Desktop/WOERK/uh_test/src/unitree_mujoco/script/right_h1_arm.urdf")
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

def get_next_position():
    try:
        rospy.wait_for_service('get_next_position', timeout=1)
        get_next_position = rospy.ServiceProxy('get_next_position', NextPosition)
        resp = get_next_position()
        return resp.pos, resp.is_ok
    except rospy.ROSException as e:
        rospy.logerr("Service not available after 1 seconds: %s", e)
        return None, True
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None, True


def publish_joint_states(model, data, pub):
    joints_of_interest = [
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint", 
        "right_shoulder_yaw_joint", 
        "right_elbow_joint"
    ]

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()

    for joint_name in joints_of_interest:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id != -1:  # Check if the joint name is valid
            joint_pos = data.qpos[model.jnt_qposadr[joint_id]]
            joint_vel = data.qvel[model.jnt_dofadr[joint_id]]
            joint_state.name.append(joint_name)
            joint_state.position.append(joint_pos)
            joint_state.velocity.append(joint_vel)

    pub.publish(joint_state)

def handle_joint_command(msg):
    joint_position = msg.position
    for id, joint_name in enumerate(["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint"]):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id != -1:
                qpos_index = model.jnt_qposadr[joint_id]
                data.qpos[qpos_index] = joint_position[id]

def main():

    rospy.init_node('joint_state_publisher', anonymous=True)
    pub = rospy.Publisher('/right/armJoint/joint_states', JointState, queue_size=1)
    sub = rospy.Subscriber('/right/armJoint/joint_command', JointCommand, handle_joint_command)


    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth = 180
        viewer.cam.elevation = -20
        viewer.cam.lookat[2] = 1.0
        viewer.cam.distance = 2.5

        while not rospy.is_shutdown() and viewer.is_running():
            step_start = time.time()

            # 计算Bezier曲线上的目标点
            pos, is_ok = get_next_position()
            if pos is not None:
                if is_ok:
                    rospy.loginfo("All positions reached. Task completed.")
                else:
                    # 逆运动学计算，将目标点转换为关节角度
                    joint_angles = inverse_kinematics(pos)
                    for id, joint_name in enumerate(["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint"]):
                            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                            if joint_id != -1:
                                qpos_index = model.jnt_qposadr[joint_id]
                                data.qpos[qpos_index] = joint_angles[id]
            else:
                rospy.loginfo("No task available.")

            mujoco.mj_step(model, data)
            viewer.sync()
            
            publish_joint_states(model, data, pub)
            
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
