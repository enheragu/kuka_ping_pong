#!/usr/bin/env python3
# encoding: utf-8

""" Objects contained in the scenario to be used during simulation
"""

__author__ = "Enrique Heredia Aguado"
__license__ = "TBD"
__version__ = "0.0.1"
__maintainer__ = "Enrique Heredia Aguado"
__email__ = "enrique.heredia@goumh.umh.es"
__status__ = "Prototype"


import time as time
from threading import Lock


class Paddle():
    """
        Paddle object to store information about paddle to compute later collisions
    """
    def __init__(self) -> None:
        self.current_position = None
        self.diameter = 0.15


class Table():
    """
        Talbe object to store information about table (size of its bounding box and position)
    """
    def __init__(self) -> None:
        # Position of the center of the bounding box
        self.current_position = [0.025, -0.2, 0.075]
        self.bounding_box = [1.6, 1.1, 0.75]

    def get_worldframed_bbox(self) -> list:
        # return table bounding box coordinates in world coord reference
        iter = range(len(self.current_position))
        return [[self.current_position[i] - self.bounding_box[i]/2, self.current_position[i] + self.bounding_box[i]/2] for i in iter]
  


class Ball():
    """
        Ball object that handles physics of the ping pong ball (position, trajectories, etc)
    """
    
    def __init__(self) -> None:

        # All three are stored as [X,Y,Z] component of position, speed and acceleratio
        self.current_position = [0, 0, 1]
        self.current_speed = [-0.7, -0.01, 0]
        self.current_acc = [0.0, 0.0, -4.9] # slowed gravity and speed to have time to simulate

        # Size of the ball
        self.size = 0.04

        self.mutex = Lock()
    
    def get_worldframed_bbox(self, current_position) -> list:
        # Return ball bounding box coordinates in world coord reference
        iter = range(len(current_position))
        return [[current_position[i] - self.size/2, current_position[i] + self.size/2] for i in iter]


    def internal_step(self, time_in, current_position_in, current_speed_in, current_acc_in) -> tuple:
        """ 
            Computes the next position and speed and return them to be stored 
            :param time_in: time lapse from previous update to get the position based on previous conditios
        """
        current_position = current_position_in
        current_speed = current_speed_in

        # Well.. not so elegant way to collide. Changes speed in Z direction
        if check_table_collision(self, current_position):
            current_speed[2] = abs(current_speed[2])
        
        # Update speed based on acc and then position with current updated speed 
        iter = range(len(current_position_in))
        current_speed = [(current_speed_in[i] + time_in * current_acc_in[i]) for i in iter]
        current_position = [(current_position_in[i] + time_in * current_speed_in[i]) for i in iter]
        # print("[INFO] [Ball::internal_step] - Current position in this iteration is: " + str(current_position))
        return current_position, current_speed

    def step(self, time_in) -> None:
        """
            Computes the next position and speed given time lapse between previous update and now.

            :param time_in: time lapse from previous update to get the position based on previous conditios
        """

        with self.mutex:
            self.current_position, self.current_speed = self.internal_step(time_in, self.current_position, self.current_speed, self.current_acc)


    def get_current_position(self) -> list:
        """
            Acces method to get current position. Is the last computed position performed in last step call
        """
        with self.mutex:
            return self.current_position

    def estimate_next_position(self, time_in, step_in) -> list:
        """
            Estimates ball position given a time lapse from current position. Takes step size into account to 
            habe a better estiamtion (more iterations can consume more time).

            :param time_in: Time from now in which the estimation is needed
            :param step_in: time step to compute the final solution. Note that if step is too big some collisions might be ignored
        """
        iterations = time_in / step_in

        # Starts with current situation and integrates the future one
        with self.mutex:    
            current_position = self.current_position
            current_speed = self.current_speed
            current_acc = self.current_acc

        for step in range(iterations):
            current_position, current_speed = self.internal_step(step_in, current_position, current_speed, current_acc)

        return current_position

    def estimate_until_x_position(self, x_plane_in, time_in, step_in) -> tuple:
        """
            Estimates position when the ball reaches a X plane. Returns that position along with the time lapse 
            estimated to reach that point

            :param x_plane_in: x coordinate of the plane to compute position
            :param time_in: time limit. If no solution is found within this time lapse the function return None
            :param step_in: time step to compute the final solution. Note that if step is too big some collisions might be ignored
        """
        start_time = time.time()

        # Stores time lapse in which the ball is supposed to cross the given plane
        total_time = 0
        # Starts with current situation and integrates the future one
        with self.mutex:    
            current_position = self.current_position
            current_speed = self.current_speed
            current_acc = self.current_acc

        while True:
            # Timeouted!
            if (time.time() - start_time) > time_in:
                print("[ERROR] [Ball::estimate_until_x_position] - Timeout!")
                break

            current_position, current_speed = self.internal_step(step_in, current_position, current_speed, current_acc)
            total_time += step_in

            # Check if position was reached
            if (current_position[0] >= x_plane_in and x_plane_in >= 0) or \
                (current_position[0] <= x_plane_in and x_plane_in < 0):
                return current_position, total_time

        print("[ERROR] [Ball::estimate_until_x_position] - No estimation found!")
        return None



def check_table_collision(ball_in, current_ball_position, table_in = Table()) -> bool:
    
    ball_bbox = ball_in.get_worldframed_bbox(current_ball_position)
    tab_bbox = table_in.get_worldframed_bbox()
    
    # print("[INFO] [check_table_collision] - Ball_pose: " + str(ball_in.current_position))
    # print("[INFO] [check_table_collision] - Ball_bbox: " + str(ball_bbox))
    # print("[INFO] [check_table_collision] - Tab_bbox: " + str(tab_bbox))

    # Check with upper Z from table and check that ball is in X,Y table area.
    if (ball_bbox[2][0] <= tab_bbox[2][1] and \
        ball_bbox[0][0] >= tab_bbox[0][0] and ball_bbox[0][1] <= tab_bbox[0][1] and \
        ball_bbox[1][0] >= tab_bbox[1][0] and ball_bbox[1][1] <= tab_bbox[1][1]):
        print("[INFO] [check_table_collision] - Collision with table detected")
        return True

    elif (ball_bbox[2][0] <= 0): # floor collision
        # print("[INFO] [check_table_collision] - Collision with floor detected")
        return True
    
    return False
