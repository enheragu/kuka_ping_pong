#!/usr/bin/env python
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

from artelib.grippers import GripperRG2
from artelib.kukalbr import RobotKUKALBR
from artelib.ur5 import RobotUR5
from artelib.scene import Sphere
import numpy as np

# Standard delta time for Coppelia
DELTA_TIME = 50.0/1000.0

def init_simulation_pingpong():
    # Python connect to the CoppeliaSim client

    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print("Connected to remote API server")
        # stop previous simiulation
        sim.simxStopSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
        time.sleep(3)
        sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
        # enable the synchronous mode
        sim.simxSynchronous(clientID=clientID, enable=True)
    else:
        print("Connection not successful")
        sys.exit("Connection failed,program ended!")

    armjoints = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint6', sim.simx_opmode_oneshot_wait)
    errorCode, q7 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint7', sim.simx_opmode_oneshot_wait)

    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)
    armjoints.append(q7)
    
    robot = RobotKUKALBR(clientID=clientID, wheeljoints=[],
                         armjoints=armjoints, base=robotbase,
                         end_effector=end_effector, gripper=gripper,
                         target=target, camera=camera)
    return robot