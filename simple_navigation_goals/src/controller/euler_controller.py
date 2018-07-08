#!/usr/bin/env python
from math import atan2, sin, cos

from .controller import Controller


class EulerMethodController(Controller):
    def __init__(self, trajectory, simulation_data, control_constants):
        Controller.__init__(self, trajectory, simulation_data)
        self.K_X = control_constants['x']
        self.K_Y = control_constants['y']
        self.K_THETA = control_constants['theta']

    def set_next_reference(self):
        reference = self.trajectory.get_position_at(self.i + 1)
        self.x_ref_n_plus_1 = reference.x
        self.y_ref_n_plus_1 = reference.y

    def get_delta_x_n(self):
        return self.x_ref_n_plus_1 - self.K_X * (self.x_ref_n - self.x_n) - self.x_n

    def get_delta_y_n(self):
        return self.y_ref_n_plus_1 - self.K_Y * (self.y_ref_n - self.y_n) - self.y_n

    def get_delta_theta_n(self):
        return self.theta_ref_n - self.K_THETA * (self.theta_ref_n - self.theta_n) - self.theta_n

    def compute_theta_ez_n(self):
         return atan2(self.get_delta_y_n(), self.get_delta_x_n())

    def compute_v_c_n(self):
        delta_x_n = self.get_delta_x_n()
        delta_y_n = self.get_delta_y_n()
        # print("-----delta positon------")
        # print("delta_x = " + str(delta_x_n))
        # print("delta_y = " + str(delta_y_n))

        return (delta_x_n * cos(self.theta_ref_n) + delta_y_n * sin(self.theta_ref_n)) / self.delta

    def compute_w_c_n(self):
        w_n = self.get_delta_theta_n() / self.delta
        # print("delta_theta_n = " + str(self.get_delta_theta_n()))

        # limit angular velocity to be between -pi and pi rad/s
        return atan2(sin(w_n), cos(w_n))

    def compute_control_actions(self, pose, twist, i):
        self.i = i
        self.set_current_orientation(pose.orientation)
        self.set_current_position(pose.position)
        self.set_current_reference(self.trajectory.get_position_at(i))
        # print(self.trajectory.get_position_at(i))
        self.set_next_reference()

        self.theta_ref_n = self.compute_theta_ez_n()
        self.v_c_n = self.compute_v_c_n()
        # print("linear velocity = " + str(self.v_c_n))
        self.w_c_n =  self.compute_w_c_n()
        # print("rotational velocity = " + str(self.w_c_n))
        # print("----------end-----------")


    def goal_reached(self, current_pose, goal, tolerance):
        if (current_pose.position.x - goal.position.x) ** 2 + (current_pose.position.y - goal.position.y) ** 2 < tolerance ** 2:
            return True
        else:
            return False
