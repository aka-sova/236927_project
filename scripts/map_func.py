
import numpy as np

class Map(object):
    def __init__(self, size_x : int = 10000, size_y :int = 10000):
        self.probs = np.zeros((size_x,size_y)) # 10 by 10 meters. Each pixel is cm^2
        self.bin_map = np.zeros((size_x,size_y))

    def update(self, loc : list, dir : list, meas : list, sensor_angles : list):
        """This function will update the map based on the recieived parameters:
        loc : Location of the agent 
        dir : Direction of the agent (dx, dy)
        meas : List of measurements
        sensor_angles : list of angles corresponding to the measurements
        
        As output, the probs of the map will be updated
        """
        pass

    def gen_binary(self):
        """By using the probabilities map, the binary map will be generated"""
        pass