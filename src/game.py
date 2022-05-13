#!/usr/bin/env python3
# encoding: utf-8

""" TBD
"""

__author__ = "Enrique Heredia Aguado"
__license__ = "TBD"
__version__ = "0.0.1"
__maintainer__ = "Enrique Heredia Aguado"
__email__ = "enrique.heredia@goumh.umh.es"
__status__ = "Prototype"


import time as time
import numpy as np

from sceneconfig.scene_config import CopInterface
from src.scenario_objs import Ball
from src.player import Player

class PingPongGame():

    def __init__(self) -> None:
        self.simulation_interface = CopInterface()
        
        self.ball = Ball()

        # Robot handler is provided to player as it includes pyARTE useful methods to be used in player class
        self.pl1_base_coord = [1.3, -0.2, 0.0787]
        print("[INFO] [PingPongGame::__init__] - Creating robot 1 handler")
        self.player1 = Player(self.simulation_interface.player1_handler)


        # Robot handler is provided to player as it includes pyARTE useful methods to be used in player class
        self.pl2_base_coord = [-1.3, -0.2, 0.0787]
        print("[INFO] [PingPongGame::__init__] - Creating robot 2 handler")
        self.player2 = Player(self.simulation_interface.player2_handler)

        print("[INFO] [PingPongGame::__init__] - Initialization completed")


    def step(self, time_in) -> None:
        """
            Updates game to next step given the time lapse

            :param time_in: time lapse from previous update to get the position based on previous conditios
        """
        self.ball.step(time_in)
    
    def play(self, time_in) -> None:
        """
            Starts the game as it is

            :param time_in: time step as default
        """
        print("[INFO] [PingPongGame::play] - Start game")

        target_position, time_delay = self.ball.estimate_until_x_position(-0.9, 1000, time_in/5)
        #target position is in world frame, needs transform to robot base frame
        print("[INFO] [Game::play] - Target position in world frame is: " + str(target_position))
        target_position = [target_position[i] - self.pl1_base_coord[i] for i in range(len(target_position))]
        print("[INFO] [Game::play] - Target position in robot base frame is: " + str(target_position))

        target_orientation = [0, np.pi/8, 0]

        self.player1.move(time_in, target_position, target_orientation)
        

    def scenario_update_loop(self, time_step_in) -> None:
        """
            Handles update of ball position

            :param time_step_in: time step to update loop
        """
        
        print("[INFO] [PingPongGame::scenario_update_loop] - Starting loop")
        start_time = time.time()
        while True:
            if (time.time() - start_time) > time_step_in:
                start_time = time.time()
                self.step(time_step_in)

                ball_position = self.ball.get_current_position()
                self.simulation_interface.set_ball_position(ball_position)
        