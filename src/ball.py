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


class Ball:
    """
        Ball object that handles physics of the ping pong ball (position, trajectories, etc)
    """
    
    def __init__(self) -> None:

        # All three are stored as [X,Y,Z] component of position, speed and acceleratio
        self.current_position = [0, 0, 0]
        self.current_speed = [0, 0, 0]
        self.current.acc = [0, 0, -9.81]
        

    def step(self, in_time) -> None:
        """
            Computes the next position and speed given time lapse between previous update and now.

            :param in_time: time lapse from previous update to get the position based on previous conditios
        """
        pass

    def get_current_position(self) -> None:
        pass

    def get_next_position(self) -> None:
        pass
