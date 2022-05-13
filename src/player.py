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

# from artelib.tools import buildT
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.inverse_kinematics import moore_penrose_damped
from artelib.euler import Euler
from artelib.path_planning import n_movements, generate_target_positions, generate_target_orientations_Q
from artelib.tools import  compute_kinematic_errors


class Player():

    def __init__(self, robot) -> None:
        self.robot = robot
        self.joint_coordinates = np.array([-np.pi / 4, 0, 0, -np.pi / 4, 0, 0, 0])

        self.robot.set_joint_target_positions(self.joint_coordinates, precision=True)
        self.robot.wait(20)

        print("[INFO] [Player::__init__] - Initialization completed")
     

    def diff_w_central(self, q, qcentral, K):
        qd0 = []
        # Computes qd0 that minimezes W
        qd0 = [-2*(q[index]-qcentral[index]) for index in range(0, len(q))]
        # print("qd0:", qd0)
        return np.array(qd0)


    def null_space_projector(self, J):
        # EJERCICIO: IMPLEMENTE UN PROYECTOR AL ESPACIO NULO P

        # we are far from a singularity
        ij = np.eye(7)

        manip = np.linalg.det(np.dot(J, J.T))
        if manip > .01 ** 2:
            iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))

        else:
            K = 0.01 * np.eye(np.min(J.shape))
            iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + K))

        iJ = np.dot(iJ, J)
        P = np.eye(7) - iJ
        return P


    def minimize_w_central(self, J, q, qc, K):
        qd0 = self.diff_w_central(q, qc, K)
        P = self.null_space_projector(J)

        #EJERCICIO: PROYECTE QD0 AL ESPACIO NULO CON EL PROYECTOR P
        #HAGA QUE QDB TENGA NORMA UNITARIA (SI SU NORMA NO ES CERO)
        qdb = np.dot(P, qd0)
        if np.linalg.norm(qdb) != 0.0:
            qdb = qdb/np.linalg.norm(qdb) # [qdb[index]/np.linalg.norm(qdb) for index in range(0, len(qdb))]
        return np.array(qdb)
    

    def inversekinematics_line(self, time_in, target_position, target_orientation, q0 = None, vmax=0.5):
        """
            Produces trajectory to move to a point

            :param time_in: time step as default
            :param target_position: target position as array
            :param target_orientation: target orientation as an Euler object
            :param q0: initial joint position of the arm
            :param vmax: linear velocity of the planner.
        """
         # Ttarget = buildT(target_position, target_orientation)
        Ttarget = HomogeneousMatrix(target_position, Euler(target_orientation))
        if q0 is None:
            q = self.joint_coordinates
        else:
            q = q0
            
        Ti = self.robot.directkinematics(q0)
        Qcurrent = Ti.Q()
        Qtarget = Euler(target_orientation).Q()
        p_current = Ti.pos()
        p_target = Ttarget.pos()
        n = n_movements(p_current, p_target, vmax)
        # generate n target positions
        target_positions = generate_target_positions(p_current, p_target, n)
        # generating quaternions on the line. Use SLERP to interpolate between quaternions
        target_orientations = generate_target_orientations_Q(Qcurrent, Qtarget, len(target_positions))
        q_path = []
        q = q0
        # now try to reach each target position on the line
        for i in range(0, len(target_positions)):
            q = self.inversekinematics_point(time_in, target_position=target_positions[i],
                                            target_orientation=target_orientations[i], q0=q)
            q_path.append(q)
        return q_path

    def inversekinematics_point(self, time_in, target_position, target_orientation, q0 = None):
        """
            Produces kinematic solution to a given point

            :param time_in: time step as default
            :param target_position: target position as array
            :param target_orientation: target orientation as an Euler object
            :param q0: initial joint position of the arm
        """
        Ttarget = HomogeneousMatrix(target_position, target_orientation)
        q = q0
        max_iterations = 500
        qc = [0, 0, np.pi, 0, 0, np.pi/2, 0]
        K = [0, 0, 1, 0, 0, 1, 0]

        for i in range(0, max_iterations):
            # print('[INFO] [Player::inversekinematics] - Iteration number: ', i)
            Ti = self.robot.directkinematics(q)
            e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
            # print('[INFO] [Player::inversekinematics] - e: ', e)
            # print('[INFO] [Player::inversekinematics] - errordist, error orient: ', error_dist, error_orient)
            if error_dist < 0.01 and error_orient < 0.01:
                # print('[INFO] [Player::inversekinematics] - Converged!!')
                break

            J, Jv, Jw = self.robot.get_jacobian(q)
            # compute joint speed to achieve the reference
            qda = moore_penrose_damped(J, e)
            qdb = self.minimize_w_central(J, q, qc, K)
            qdb = 0.3 * np.linalg.norm(qda) * qdb
            qd = qda + qdb
            [qd, _, _] = self.robot.check_speed(qd)
            qd = np.dot(time_in, qd)
            q = q + qd
            [q, _] = self.robot.apply_joint_limits(q)
        return q

    def move(self, time_in, target_position, target_orientation) -> None:
        
        """
            Moves to a given target position and orientation

            :param time_in: time step as default
            :param target_position: target position as array
            :param target_orientation: target orientation as an Euler object
        """
        print("[INFO] [Player::move] - Move to position: " + str(target_position) + ", with orientation: " + str(target_orientation))
        print("[INFO] [Player::move] - Current joint position is: " + str(self.robot.get_joint_positions()))
        path = self.inversekinematics_line(time_in, target_position, target_orientation, self.robot.get_joint_positions())

        # set the target we are willing to reach on Coppelia
        self.robot.set_target_position_orientation(target_position, target_orientation)
        self.robot.set_joint_target_trajectory(path, precision='last')