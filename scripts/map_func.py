import copy
import pickle
import os
import sys
cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))
sys.path.append(os.path.join(cur_loc, 'scripts'))

import numpy as np
from tkinter import *
import math
import timeit
import shutil
from scipy import ndimage
import matplotlib
from matplotlib import image

import C_CONSTANTS

class Map(object):
    def __init__(self, logger, size_x : int = 1000, size_y :int = 1000, sensor_angles : list = [-45, 0, 45], output_loc: str = '', output_temp_loc : str = '', artifacts_loc : str = ''):
        
        self.x = size_x # rows
        self.y = size_y # cols

        # maps
        self.bin_map = np.zeros((size_x,size_y), dtype=np.bool)         # original sensors data
        self.filled_map = np.zeros((size_x,size_y), dtype=np.bool)      # map with 'filled' gaps between measuremenets
        self.inflated_map = np.zeros((size_x,size_y), dtype=np.bool)    # map with 'inflated' obstacle
        
        # locations on disk
        self.output_loc = output_loc
        self.output_temp_loc = output_temp_loc
        self.artifacts_loc = artifacts_loc

        self.sensor_angles = sensor_angles
        self.logger = logger

    def update(self, cur_loc : list, dir_angle, meas : list):
        """This function will update the map based on the recieived parameters:
        cur_loc : Location of the agent (list of x, y)
        dir_angle : Direction of the agent (float) (deg)
        meas : List of measurements
        
        As output, the probs of the map will be updated
        """

        # for now, implement the binary map directly
        # perhaps will be good enough

        for meas_data, sensor_angle in zip(meas, self.sensor_angles):
            sensor_true_angle = dir_angle + sensor_angle

            # if sensor_angle != -45:
            #     continue

            if meas_data > 0:
                # print("Current X {} Y {}".format(cur_loc[0], cur_loc[1]))
                # print("Meas data : " + str(meas_data))
                # print("sensor_true_angle : " + str(sensor_true_angle))

                # those are x, y in the camera frame
                x = int(meas_data * math.cos(sensor_true_angle*math.pi/180) + cur_loc[0])
                y = int(meas_data * math.sin(sensor_true_angle*math.pi/180) + cur_loc[1])

                # camera frame
                # print("Obstacle at X : {}  Y : {} : ".format(x,y))

                # csv frame
                col, row = self.to_csv_coords(x, y)

                # print("CSV ROW {} COL {}".format(row,col))

                # mark the map in this location as obstacle
                self.bin_map[row][col] = 1

                # 1. Analyze the pixels around within a certain range. If a pixel is 'occupied', 
                # mark all the pixels in between as 'occupied'. Robot cannot pass in between those pixels.

                self.filled_map = Map.connect_nearby_obstacles(input_map = self.bin_map, filled_map = self.filled_map, row = row, col = col, logger=self.logger)

        # 2. mark also all the nearest pixels within the range as 'occupied'
        self.inflated_map = Map.inflate_obstacles(input_map = self.filled_map, logger = self.logger)

    @staticmethod
    def inflate_obstacles(input_map : np.ndarray, logger = None):
        """mark all the nearest pixels within the range as 'occupied'"""

        start = timeit.default_timer()
        
        # we inflate the 'filled_map' by using the moorphological 'dilation' operation
        iterations = C_CONSTANTS.MAP_INFLATE_RANGE - 1

        inflated_map = ndimage.binary_dilation(input_map, iterations=iterations).astype(input_map.dtype)

        stop = timeit.default_timer()
        if logger is not None:
            logger.info('[MAPPING] Inflating map. Time elapsed :  {}'.format(stop - start)) 

        return inflated_map
        
    @staticmethod
    def connect_nearby_obstacles(input_map: np.ndarray, filled_map: np.ndarray, row : int, col : int, logger = None):
        """Analyze the pixels around within a certain range. If a pixel is 'occupied', 
        mark all the pixels in between as 'occupied'. Robot cannot pass in between those pixels.
        row, col - indexes of the pixel to analyze """

        # we input filled_map, but check the nearest occupied pixels in input_map, 
        # so that we won't increase the size of the obstacles in region where they don't exist

        map_size_x = input_map.shape[0]
        map_size_y = input_map.shape[1]

        # filled_map = copy.deepcopy(input_map)

        start = timeit.default_timer()

        # first, mark the required pixel as occupied
        filled_map[row][col] = 1

        filter_kernel = Map.generate_bin_filter(map_size_x, map_size_y, row, col)

        # use the elementwise multiplication to mark only the pixels with obstacles
        filtered_img = np.multiply(input_map, filter_kernel)

        # remove the original pixel itself from the image
        filtered_img[row][col] = 0

        # indexes of non-zero elements
        out_ind = np.transpose(np.nonzero(filtered_img))

        # for each non zero element, mark all the elements in between as occupied
        for non_zero_arr in out_ind:
            nonzero_row = non_zero_arr[0]
            nonzero_col = non_zero_arr[1]

            # find a slope
            d_row = row - nonzero_row
            d_col = col - nonzero_col
            slope = (float)(d_row / d_col)

            # build a line
            x_start = min(col, nonzero_col)
            x_end = max(col, nonzero_col)
            num_points = x_end - x_start
            x_arr = np.linspace(x_start, x_end, num_points)
            y0 = row - slope * col

            y_arr = x_arr * slope + y0

            for x, y in zip(x_arr, y_arr):

                # x is col
                try:
                    # putting try so won't be out of boundaries
                    filled_map[(int)(np.floor(y))][(int)(x)] = 1
                    filled_map[(int)(np.ceil(y))][(int)(x)] = 1
                except:
                    pass

        stop = timeit.default_timer()
        if logger is not None:
            logger.info('[MAPPING] Connecting obstacles in map. Time elapsed :  {}'.format(stop - start))            

        return filled_map

    @staticmethod
    def generate_bin_filter(map_size_x, map_size_y, row, col):
        """ use the morphological 'dilation' operation to create a filter """
        
        iterations = C_CONSTANTS.MAP_CONNECT_RANGE-1

        # generate filter of the size of the image
        filt = np.zeros((map_size_x, map_size_y))
         
        # filter is always of uneven size, thus we know for sure there's a center pixel
        filt[row, col] = 1
        filt = ndimage.binary_dilation(filt, iterations=iterations).astype(filt.dtype)

        return filt

    def to_csv_coords(self, x, y):
        """Change to csv coordinate system (only absolute values)"""


        csv_row = int(self.x/2 - x)
        csv_col = int(self.y/2 + y)

        return csv_col, csv_row

    def save_clear_output(self):
        """Will save the output in a specified format - CSV"""

        start = timeit.default_timer()

        output_fd = open(self.output_temp_loc, 'w')
        for row in range(self.bin_map.shape[0]):
            for col in range(self.bin_map.shape[1]):
                output_fd.write(str(int(self.bin_map[row][col])) + ",")
            output_fd.write('\n')

        output_fd.close()

        shutil.copy(self.output_temp_loc, self.output_loc)

        stop = timeit.default_timer()

        self.logger.info('[MAPPING] Saving the map as CSV. Time elapsed :  {}'.format(stop - start))  
        

    def save_clear_output_pickle(self):
        """Will save the output in a specified format - PICKLE"""

        start = timeit.default_timer()

        pickle.dump(self.bin_map, open(self.output_temp_loc, "wb"))
        shutil.copy(self.output_temp_loc, self.output_loc)

        stop = timeit.default_timer()

        self.logger.info('[MAPPING] Saving the map as PICKLE. Time elapsed :  {}'.format(stop - start))  
            

    def save_maps_debug(self):
        """Save maps as images for debug & further analysis"""

        start = timeit.default_timer()

        matplotlib.image.imsave(os.path.join(self.artifacts_loc, 'bin_map.png'), self.bin_map)
        matplotlib.image.imsave(os.path.join(self.artifacts_loc, 'filled_map.png'), self.filled_map)
        matplotlib.image.imsave(os.path.join(self.artifacts_loc, 'inflated_map.png'), self.inflated_map)

        stop = timeit.default_timer()

        self.logger.info('[MAPPING] Saving the maps as PNG IMAGES. Time elapsed :  {}'.format(stop - start))  


    def generate_the_graph(self):
        """The initialization of the graph"""
        pass

    def generate_path_to_goal(self):
        """The main function for the path generation.

        Input : the 'inflated' map, which is the occupancy grid with the resolution of 1 cm^2
        Output : a list of Targets, which will lead to the goal
        
        We use a graph which is generated and updated all the time as new information comes in"""
        



        pass


