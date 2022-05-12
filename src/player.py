#!/usr/bin/env python
# encoding: utf-8

""" TBD
"""

__author__ = "Enrique Heredia Aguado"
__license__ = "TBD"
__version__ = "0.0.1"
__maintainer__ = "Enrique Heredia Aguado"
__email__ = "enrique.heredia@goumh.umh.es"
__status__ = "Prototype"

import numpy as np

from artelib.tools import buildT

class Player():

    def __init__(self, robot) -> None:
        self.robot = robot


    def null_space(self, J):
        """
            Obtain a unit vector in the direction of the J null space.
            Consider m as DOF of the application of interest.
        """
        u, s, vh = np.linalg.svd(J, full_matrices=True)

        # Check that last item is actually 0
        if s[-1] < 0.001:
            print("[ERROR] [Player::null_space] - no null space in S matrix")

        qd = vh[-1]

        return qd


    def move_null_space(self, target_in):
        """
            Moves the robotic arm to target

            :param target_in: target to positionate the robot
        """

        # initial arm position
        q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, -np.pi / 2, 0.1, 0.1, 0.1])
        self.robot.set_joint_target_positions(q0, precision=True)

        # ok perform n movements in null space
        n_movements_in_null_space = 150
        q=q0
        q_path = []
        qd_path = []

        for i in range(0, n_movements_in_null_space):
            # print('[INFO] [Player::move_null_space] - Movement number: ', i)

            J, Jv, Jw = self.robot.get_jacobian(q)
            qd = self.null_space(J)

            # Always takes neagtive qd if second is negative
            if qd[2] < 0:
                qd = -qd

            qd = np.dot(DELTA_TIME, qd)
            q = q + qd
            [q, _] = self.robot.apply_joint_limits(q)
            q_path.append(q)
            qd_path.append(qd)
        self.robot.set_joint_target_trajectory(q_path, precision='last')