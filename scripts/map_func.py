
import numpy as np
from tkinter import *
import math
import timeit
import shutil

class Map(object):
    def __init__(self, logger, size_x : int = 1000, size_y :int = 1000, sensor_angles : list = [-45, 0, 45], output_loc: str = '', output_temp_loc : str = ''):
        
        self.x = size_x
        self.y = size_y

        self.bin_map = np.zeros((size_x,size_y), dtype=np.bool)
        self.output_loc = output_loc
        self.output_temp_loc = output_temp_loc
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

    def to_csv_coords(self, x, y):
        """Change to csv coordinate system (only absolute values)"""


        csv_row = int(self.x/2 - x)
        csv_col = int(self.y/2 + y)

        return csv_col, csv_row

    def save_clear_output(self):
        """Will save the output in a specified format"""

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
        
    def save_temp_output(self):
        """Save only the new data that was received"""
        pass