#!/usr/bin/env python3
# encoding: utf-8

""" This file has the different classes handling the initializatio and comunication between pyARTE library and
    CoppeliaSim
"""

__author__ = "Enrique Heredia Aguado"
__license__ = "TBD"
__version__ = "0.0.1"
__maintainer__ = "Enrique Heredia Aguado"
__email__ = "enrique.heredia@goumh.umh.es"
__status__ = "Prototype"

import time
import sim
import sys

from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
import numpy as np



class CopInterface():
    """ 
        This class handle the Python - CoppeliaSim interface. Makes initializatio and handle communication between them making use of the
        pyARTE library to handle robots
    """

    def __init__(self) -> None:
        self.player1_handler = None
        self.player2_handler = None
        self.ball_handler = None
        self.clientID = None

        self.player1_handler, self.player2_handler, self.ball_handler = self.init_simulation()

        print("[INFO] [CopInterface::__init__] - Initialization completed")

    def init_sim(self):
        # Python connect to the CoppeiaSim client
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

        if clientID != -1:
            print("[INFO] [CopInterface::init_sim] - Connected to remote API server.")
            # stop previous simiulation
            sim.simxStopSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
            time.sleep(3)
            sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
            # enable the synchronous mode
            sim.simxSynchronous(clientID=clientID, enable=True)
        else:
            print("[ERROR] [CopInterface::init_sim] - Connection not successful.")
            sys.exit("[FATAL] [CopInterface::init_sim] - Connection failed,program ended!")
        return clientID


    def init_simulation(self):
        """ 
            Initializes the coppelia simulator storing the handlers for both player robots based on pyARTE objects and
            the ball handler to post its position during the game

            :return: player1 pyARTE object to handle Kuka robot
            :return: player2 pyARTE object to handle Kuka robot
            :return: ball_handler object
        """

        self.clientID = self.init_sim()

        # Get the handles of the relevant objects
        errorCode, robotbase1 = sim.simxGetObjectHandle(self.clientID, 'KUKA_PLAYER_1', sim.simx_opmode_oneshot_wait)
        errorCode, robotbase2 = sim.simxGetObjectHandle(self.clientID, 'KUKA_PLAYER_2', sim.simx_opmode_oneshot_wait)
        errorCode, end_effector = sim.simxGetObjectHandle(self.clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

        armjoints = []
        for index in range(1,8): # in LUA tag starts from 1 to 7, both included
            errorCode, q = sim.simxGetObjectHandle(self.clientID, 'LBR_iiwa_14_R820_joint'+str(index), sim.simx_opmode_oneshot_wait)
            # TBD check errorCode!!
            armjoints.append(q)
        
        # TBD check errorCode!!
        errorCode, gripper_joint1 = sim.simxGetObjectHandle(self.clientID, 'RG2_openCloseJoint', sim.simx_opmode_oneshot_wait)
        errorCode, target = sim.simxGetObjectHandle(self.clientID, 'target', sim.simx_opmode_oneshot_wait)
        errorCode, camera = sim.simxGetObjectHandle(self.clientID, 'camera', sim.simx_opmode_oneshot_wait)

        # Generate classes for both Kuka robots to handle connection
        gripper = GripperRG2(clientID=self.clientID, joints=[gripper_joint1])
        player1_handler = RobotKUKALBR(clientID=self.clientID, wheeljoints=[],
                            armjoints=armjoints, base=robotbase1,
                            end_effector=end_effector, gripper=gripper,
                            target=target, camera=camera)
                            
        player2_handler = RobotKUKALBR(clientID=self.clientID, wheeljoints=[],
                            armjoints=armjoints, base=robotbase2,
                            end_effector=end_effector, gripper=gripper,
                            target=target, camera=camera)

        errorCode, ball_handler = sim.simxGetObjectHandle(self.clientID, 'ball', sim.simx_opmode_oneshot_wait)

        return player1_handler, player2_handler, ball_handler


    
    def set_ball_position(self, position_in) -> None:
        """ 
            Set ball position in CoppeliaSim

            :param position_in: input position in which to place the ball object in the simulatio
        """
        # TBD check errorCode!!
        errorCode = sim.simxSetObjectPosition(self.clientID, self.ball_handler, -1, position_in, sim.simx_opmode_oneshot_wait)


    def __del__(self) -> None:
        """
            Class destructor
        """
        print("[INFO] [CopInterface::__del__] - Stopping both arms and simulation")
        self.player1_handler.stop_arm()
        self.player1_handler.stop_simulation()

        self.player2_handler.stop_arm()
        self.player2_handler.stop_simulation()