#!/usr/bin/env python3
# encoding: utf-8
"""
TDB

"""
__author__ = "Enrique Heredia Aguado"
__license__ = "TBD"
__version__ = "0.0.1"
__maintainer__ = "Enrique Heredia Aguado"
__email__ = "enrique.heredia@goumh.umh.es"
__status__ = "Prototype"

from game import PingPongGame
from threading import Thread

# Standard delta time for Coppelia
DELTA_TIME = 50.0/1000.0

GAME_UPDATE_RATE = 25.0/1000.0

if __name__ == "__main__":
    game = PingPongGame()

    print("[INFO] [main] - Start dinamics update thread")
    scenario_uptade_th = Thread(target=game.scenario_update_loop, args=[GAME_UPDATE_RATE])
    scenario_uptade_th.start()
    
    print("[INFO] [main] - Start game")
    
    game.play(GAME_UPDATE_RATE)