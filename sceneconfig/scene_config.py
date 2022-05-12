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



class PingCopInterface():
    """ 
        This class handle the PingPong - CoppeliaSim interface. Makes initializatio and handle communication between them making use of the
        pyARTE library to handle robots
    """

    def __init__(self) -> None:
        self.player1_handler = None
        self.player2_handler = None
        self.ball_handler = None
        self.clientID = None

        self.player1_handler, self.player2_handler, self.ball_handler = self.init_simulation()

        print("[INFO] [PingCopInterface::__init__] - Initialization completed")

    
    def init_simulation(self):
        """ 
            Initializes the coppelia simulator storing the handlers for both player robots based on pyARTE objects and
            the ball handler to post its position during the game

            :return: player1 pyARTE object to handle Kuka robot
            :return: player2 pyARTE object to handle Kuka robot
            :return: ball_handler object
        """

        # Python connect to the CoppeliaSim client
        sim.simxFinish(-1)
        self.clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

        if self.clientID != -1:
            print("[INFO] [PingCopInterface::init_simulation] - Connected to remote API server.")
            # stop previous simiulation
            sim.simxStopSimulation(clientID=self.clientID, operationMode=sim.simx_opmode_blocking)
            time.sleep(3)
            sim.simxStartSimulation(clientID=self.clientID, operationMode=sim.simx_opmode_blocking)
            # enable the synchronous mode
            sim.simxSynchronous(clientID=self.clientID, enable=True)
        else:
            print("[ERROR] [PingCopInterface::init_simulation] - Connection not successful.")
            sys.exit("[FATAL] [PingCopInterface::init_simulation] - Connection failed,program ended!")

        armjoints = []

        # Get the handles of the relevant objects
        errorCode, robotbase = sim.simxGetObjectHandle(self.clientID, 'KUKA_PLAYER_1', sim.simx_opmode_oneshot_wait)
        errorCode, end_effector = sim.simxGetObjectHandle(self.clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

        for index in range(1,7):
            errorCode, q = sim.simxGetObjectHandle(self.clientID, 'LBR_iiwa_14_R820_joint'+str(index), sim.simx_opmode_oneshot_wait)
            # TBD check errorCode!!
            armjoints.append(q)
        
        # TBD check errorCode!!
        errorCode, gripper_joint1 = sim.simxGetObjectHandle(self.clientID, 'RG2_openCloseJoint', sim.simx_opmode_oneshot_wait)
        errorCode, target = sim.simxGetObjectHandle(self.clientID, 'target', sim.simx_opmode_oneshot_wait)
        errorCode, camera = sim.simxGetObjectHandle(self.clientID, 'camera', sim.simx_opmode_oneshot_wait)

        errorCode, ball_handler = sim.simxGetObjectHandle(self.clientID, 'ball', sim.simx_opmode_oneshot_wait)


        # Generate classes for both Kuka robots to handle connection
        gripper = GripperRG2(clientID=self.clientID, joints=[gripper_joint1])
        player1_handler = RobotKUKALBR(clientID=self.clientID, wheeljoints=[],
                            armjoints=armjoints, base=robotbase,
                            end_effector=end_effector, gripper=gripper,
                            target=target, camera=camera)
                            
        player2_handler = None

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
        print("[INFO] [PingCopInterface::__del__] - Stopping both arms and simulation")
        self.player1_handler.stop_arm()
        self.player1_handler.stop_simulation()