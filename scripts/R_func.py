import sys
import os

cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))

from udpclient import RClient
from map_func import Map

import time
import logging
import numpy as np
import threading

global logger

class Target(object):
    def __init__(self, target_type : str, target_vals : list):
        self.type = target_type

        self.x = ''
        self.y = ''
        self.angle = ''

        if self.type == "POS":
            self.x = target_vals[0]
            self.y = target_vals[1]
        elif self.type == "ROT":
            self.angle = target_vals[0]
        elif self.type == "POPULATE_MAP":
            pass
        else:
            raise Exception("Invalid target type")
        

class R_Client_Extend(RClient):
    def __init__(self, host, angles : list, port,user_deprecate='',id_deprecate=''):
        super().__init__(host,port,user_deprecate='',id_deprecate='')
        self.goto_margin = 10  # margin to know we have arrived 
        self.angle_margin = 5 # [deg] 
        self.map = Map()
        self.sensor_angles = angles
        self.cur_loc = []
        self.cur_dir = []
        self.cur_readings = []
        self.status = 'idle'   # ['idle' / 'in_process']
        self.dest_reached = False
        self.next_target = None
        self.current_target = None

    def get_data(self):
        """Use the 'sense' method, put data in appropriating struct"""
        sense_data = self.sense()
        self.cur_loc = sense_data[0:1]
        self.cur_dir = sense_data[2:3]
        self.cur_readings = sense_data[4:]


    def goto(self, target : Target):
        """Go to a specific point defined by [X, Y]
        target is of type 'POS'
        The path will be linear"""

        assert target.type == "POS"

        # This functin will implement intermediate 'targets' of rotational type to reach the goal

        # 1. Using the current location values, and the target values, calculate initial values:
        #       a. distance
        #       b. angle to rotate to

        # While the goal is not reached:

            # 2. Send the rotation command
            # 3. Send the movement command - should be done at segments if exceeds the limit?

            # 4. check if the goal was reached. If not - Do 2,3 again


        # when finished, clear the target, change the status
        
        self.current_target = None
        self.status = 'idle'

    def populate_map(self):
        """This function will slowly spin the robot around itself,
        in order to gather information about the environment (update the map)
        """

        # make the loop for rotation and time.sleep

        self.current_target = None
        self.status = 'idle'

    def turn(self, angle: int):
        """Turn the robot by 'angle' [deg]"""
        pass

    def moveincircle(self, radius : int, init_dir : str):
        """Move the robot in a circle starting from the initial location 
        the center of the circle will be generated to the left / right from the robot"""
        pass

    def make_decision(self):
        """based on the current location, and current map
        make a decision, and return the GOTO next location"""

        self.current_target = None

    def update_map(self):
        """This is the function for a thread to update the map"""
        while not self.dest_reached:
            # 1. Update the cur data
            self.get_data()

            # 2. Update the Map
            self.map.update(loc = self.cur_loc,
                            dir = self.cur_dir,
                            meas = self.cur_readings,
                            sensor_angles = self.sensor_angles)

            time.sleep(0.1)

    def reach_destination(self, target):
        """The main function which will look for a path to find to reach the goal"""

        logger.info('Initializing the Algorithm')

        # start the mapping thread
        self.mapping_thread = threading.Thread(target=self.update_map)
        self.mapping_thread.start()

        while not self.dest_reached:

            # 3. If the robot is currently not during process or action, find the next decision
            if self.status ==  'idle':
                self.make_decision()   # will update the 'current_target' property of the object
                self.status = 'in_progress'
            else:
                # 4. Check if the robot has finished the current task
                if self.current_target.type == "POS":
                    self.goto(self.current_target)
                elif self.current_target.type == "POPULATE_MAP":
                    self.populate_map()
                else:
                    # no decision was taken?
                    pass


            time.sleep(0.1)

