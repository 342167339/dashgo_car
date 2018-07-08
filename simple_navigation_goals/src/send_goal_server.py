#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler  
from visualization_msgs.msg import Marker  
from math import radians, pi, sqrt 
from simple_navigation_goals.srv import GoalAndRespond

import os
import sys
import time
from collections import namedtuple

from nav_msgs.msg import Odometry  # 获取机器人信息
from geometry_msgs.msg import Twist

from plotter.plotter import get_data_container
from plotter.simulation_plotter import SimulationPlotter
from trajectory.builder import create_trajectory
from util.builder import create_controller
from util.results import export_results
from util.angle import get_euler_orientation

SimInfo = namedtuple('SimInfo', 'time, max_v, max_w')

DELTA_T = 0.1  # this is the sampling time
SIM_INFO = {
    'linear': SimInfo(30.0, 0.2, 1.35),
    'circular': SimInfo(300.0, 0.11, 1.25),
    'squared': SimInfo(300.0, 0.11, 1.25),
    'lemniscate': SimInfo(300.0, 0.125, 1.25),
    'epitrochoid': SimInfo(600.0, 0.162, 1.25),
    'lissajous': SimInfo(300.0, 0.162, 1.25)
}

IMPLEMENTED = {
    'euler':
        ('linear', 'circular', 'squared',
         'lemniscate', 'epitrochoid', 'lissajous'),
    'pid':
        ('linear'),
}

def get_pose(message):
    global current_pose
    current_pose = message.pose.pose
    # print(str(current_pose))
    
def get_pose_odom(message):
    global current_pose
    current_pose = message.pose.pose
    # print(str(current_pose))

def get_twist(message):
    global current_twist
    current_twist = message
    # print(str(current_twist))


def move(req):
    CONTROLLER = 'euler'
    TRAJECTORY = 'linear'

    current_twist = Twist()
    twist_publisher = rospy.Publisher('/cmd_vel',  # cmd_vel_mux/input/teleop
                                      Twist,
                                      queue_size=1)
    # goals = req.goals.poses[0]  # 目标位置一个
    goals = req.goals.poses  # 目标位置
    tolerance_pose = 0.05  # 位置偏移
    tolerance_theta = 0.15  # 角度偏移
    # while current_pose is None or current_twist is None:
    while current_pose is None:
        print("hello\n")
        pass

    while goals:  # 当列表非空时
        goal = goals[0]  # 选择第一个点
        # x_vel = 0.05  # 速度0.05
        delta_x = req.goals.poses[0].position.x - current_pose.position.x
        delta_y = req.goals.poses[0].position.y - current_pose.position.y
        dis = sqrt(delta_x**2 + delta_y**2)
        print("distance = " + str(dis))
        STEPS = int(dis / 0.01)  # 步数
        print(STEPS)
        # 由于字典不能赋值，创建一个列表保存linear的值
        sim_info = {
            'steps': STEPS,
            'max_v': SIM_INFO['linear'].max_v,
            'max_w': SIM_INFO['linear'].max_w
        }
        i = 0
        trajectory = create_trajectory(TRAJECTORY, current_pose, goal, sim_info)  # 创建路径
        data_container = get_data_container(CONTROLLER)  # 保存数据
        controller = create_controller(trajectory, CONTROLLER, DELTA_T, sim_info)  # 创建控制器
        rate = rospy.Rate(int(1 / DELTA_T))
        # 位置控制器
        while not rospy.is_shutdown() and not controller.goal_reached(current_pose, goal, tolerance_pose):
            # compute_control_actions()
            # global i
            controller.compute_control_actions(current_pose, current_twist, i)
            data_container['t'].append(i * DELTA_T)
            data_container['x'].append(current_pose.position.x)
            data_container['x_ref'].append(controller.x_ref_n)
            data_container['y'].append(current_pose.position.y)
            data_container['y_ref'].append(controller.y_ref_n)
            data_container['theta'].append(controller.theta_n)
            data_container['theta_ref'].append(controller.theta_ref_n)
            data_container['v_c'].append(controller.v_c_n)
            data_container['w_c'].append(controller.w_c_n)
            data_container['zeros'].append(0)

            twist = Twist()
            twist.linear.x = controller.v_c_n
            twist.angular.z = controller.w_c_n
            twist_publisher.publish(twist)

            i += 1
            # print(str(i))
            rate.sleep()

        # 角度控制器
        delta_theta = get_euler_orientation(current_pose.orientation)[2] - get_euler_orientation(goal.orientation)[2]  # 角度差值
        time.sleep(2)  # 休眠两秒
        while not rospy.is_shutdown() and abs(delta_theta) > tolerance_theta:
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = -0.5 * delta_theta
            twist_publisher.publish(twist)

            rate.sleep()
            delta_theta = get_euler_orientation(current_pose.orientation)[2] - get_euler_orientation(goal.orientation)[2]  # 角度差值
            print("current_pose_orientation = " + str(get_euler_orientation(current_pose.orientation)[2]))
            print("goal_orientation = " + str(get_euler_orientation(goal.orientation)[2]))
        time.sleep(5)  # 休眠两秒

        del goals[0]  # 删除第一个goal

    print('Simulation was completed successfully!')
    export_results(data_container,
                   CONTROLLER, TRAJECTORY,
                   os.sep.join(__file__.split(os.sep)[:-2]) + '/results.db')
    print('Data was exported successfully!')
    print('Plotting results...')
    plotter = SimulationPlotter(data_container)
    plotter.plot_results()
    print('Press Ctrl+C to terminate program.')



    return True

def send_goal_server():
    rospy.init_node('send_goal_receive_respond_server')
    s = rospy.Service('abc', GoalAndRespond, move)
    # subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, get_pose)
    subscriber = rospy.Subscriber('/odom', Odometry, get_pose_odom)
    subscriber2 = rospy.Subscriber('/smoother_cmd_vel', Twist, get_twist)  # cmd_vel
    
    current_pose = None

    print('Ready to receive goal.')
    rospy.spin()

if __name__ == "__main__":
    send_goal_server()