#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
from .trajectory import Trajectory


class LinearTrajectory(object, Trajectory):
    def __init__(self, current_pose, goal, STEPS):
        Trajectory.__init__(self)
        self.v_x = 0
        self.v_y = 0
        self.x_0 = current_pose.position.x
        self.y_0 = current_pose.position.y
        self.goal_x = goal.position.x
        self.goal_y = goal.position.y
        self.delta_x = self.goal_x - self.x_0
        self.delta_y = self.goal_y - self.y_0
        self.steps = STEPS  # 步数

    def get_position_at(self, i):
        super(LinearTrajectory, self).get_position_at(i)
        # print("step = " + str(self.steps) + "  i = " + str(i))
        # print("goal_x = " + str(self.goal_x) + "  goal_y = " + str(self.goal_y))
        # print("pose_x = " + str(self.x_0) + "  pose_y = " + str(self.y_0))
        # print("delta_x = " + str(self.delta_x) + "  delta_y = " + str(self.delta_y))
        if i < self.steps:
            self.position.x = self.delta_x * (i/self.steps) + self.x_0
            self.position.y = self.delta_y * (i/self.steps) + self.y_0
            # print("i/self.steps = " + str(i/self.steps))
        else:
            self.position.x = self.goal_x
            self.position.y = self.goal_y
        # print("-----reference positon------")
        # print(self.position)
        # print("-------------end------------")
        return self.position

    def get_name(self):
        return str(LinearTrajectory.__name__).replace('Trajectory', '').lower()
