
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

                self.connect_nearby_obstacles(row, col)

        # 2. mark also all the nearest pixels within the range as 'occupied'
        self.inflate_obstacles()

    def inflate_obstacles(self):
        """mark all the nearest pixels within the range as 'occupied'"""
        
        # we inflate the 'filled_map' by using the moorphological 'dilation' operation
        iterations = C_CONSTANTS.MAP_INFLATE_RANGE - 1

        self.inflated_map = ndimage.binary_dilation(self.filled_map, iterations=iterations).astype(self.filled_map.dtype)
        

    def connect_nearby_obstacles(self, row, col):
        """Analyze the pixels around within a certain range. If a pixel is 'occupied', 
        mark all the pixels in between as 'occupied'. Robot cannot pass in between those pixels.
        row, col - indexes of the pixel to analyze """

        # first, mark the required pixel as occupied
        self.filled_map[row][col] = 1

        filter_kernel = self.generate_bin_filter(row, col)

        # use the elementwise multiplication to 
        filtered_img = np.multiply(self.bin_map, filter_kernel)

        # indexes of non-zero elements
        out_ind = np.transpose(np.nonzero(filtered_img))

        # for each non zero element, mark all the elements in between as occupied
        for non_zero_idx in out_ind:
            nonzero_row = non_zero_idx[out_ind][0]
            nonzero_col = non_zero_idx[out_ind][1]

            # find a slope
            d_row = row - nonzero_row
            d_col = col - nonzero_col
            slope = (float)(d_row / d_col)

            # build a line
            x_start = min(col, nonzero_col)
            x_end = max(col, nonzero_col)
            num_points = x_end - x_start
            x_arr = np.linspace(x_start, x_end, num_points)
            y0 = row

            y_arr = x_arr * slope + y0

            for x, y in zip(x_arr, y_arr):

                # x is col
                try:
                    # putting try so won't be out of boundaries
                    self.filled_map[np.floor(y)][x] = 1
                    self.filled_map[np.ceil(y)][x] = 1
                except:
                    pass



    def generate_bin_filter(self, row, col):
        """ use the morphological 'dilation' operation to create a filter """
        iterations = C_CONSTANTS.MAP_CONNECT_RANGE-1

        # generate filter of the size of the image
        filt = np.zeros(self.x, self.y) 
         
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

        # it takes 3.5 seconds to do it,
        # so perhaps is not the best option to work online

        start = timeit.default_timer()

        output_fd = open(self.output_temp_loc, 'w')
        for row in range(self.bin_map.shape[0]):
            for col in range(self.bin_map.shape[1]):
                output_fd.write(str(int(self.bin_map[row][col])) + ",")
            output_fd.write('\n')

        output_fd.close()

        shutil.copy(self.output_temp_loc, self.output_loc)

        stop = timeit.default_timer()

        self.logger.info('Saving the map. Time elapsed :  {}'.format(stop - start))  
        

    def save_maps_debug(self):
        """Save maps as images for debug & further analysis"""

        self.logger.info("Saving the maps as images for debug")

        matplotlib.image.imsave(os.path.join(self.artifacts_loc, 'bin_map.png'), self.bin_map)
        matplotlib.image.imsave(os.path.join(self.artifacts_loc, 'filles_map.png'), self.filled_map)
        matplotlib.image.imsave(os.path.join(self.artifacts_loc, 'inflated_map.png'), self.inflated_map)