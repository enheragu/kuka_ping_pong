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
from sceneconfig.scene_config import PingCopInterface
from src.scenario_objs import Ball
from src.player import Player

class PingPongGame():

    def __init__(self) -> None:
        self.simulation_interface = PingCopInterface()
        
        self.ball = Ball()

        # Robot handler is provided to player as it includes pyARTE useful methods to be used in player class
        self.player1 = Player(self.simulation_interface.player1_handler)

    def step(self, time_in) -> None:
        """
            Updates game to next step given the time lapse

            :param time_in: time lapse from previous update to get the position based on previous conditios
        """
        self.ball.step(time_in)
    
    def play(self, time_in) -> None:
        pass

    def scenario_update_loop(self, time_step_in) -> None:
        """
            Handles update of ball position

            :param time_step_in: time step to update loop
        """
        start_time = time.time()
        while True:
            if (time.time() - start_time) > time_step_in:
                start_time = time.time()
                self.step(time_step_in)

                ball_position = self.ball.get_current_position()
                self.simulation_interface.set_ball_position(ball_position)
        