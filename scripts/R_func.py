import sys
import os
import traceback

cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))

from udpclient import RClient
from map_func import Map

import time
import logging
import numpy as np
import threading
import math

global logger

def init_logger(logger_address : str):
    # create a logger
    logging.basicConfig(filename=logger_address, 
                        level=logging.DEBUG,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        datefmt='%m/%d %I:%M:%S',
                        filemode='w')
    logger = logging.getLogger(__name__)

    # basic logger usage
    # logger.debug('This message should go to the log file')
    # logger.info('So should this')
    # logger.warning('And this, too')
    # logger.error('error message')
    # logger.critical('critical message')

    return logger  


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
    def __init__(self, host, calib_folder, angles : list, logger_location, port,user_deprecate='',id_deprecate=''):
        super().__init__(host,port,user_deprecate='',id_deprecate='')
        self.goto_margin = 10  # margin to know we have arrived 
        self.angle_margin = 5 # [deg] 
        self.map = Map()
        self.sensor_angles = angles
        self.cur_loc = []
        self.cur_dir = []
        self.cur_angle = ''
        self.cur_readings = []
        self.status = 'idle'   # ['idle' / 'in_process']
        self.dest_reached = False
        self.next_target = None
        self.current_target = None
        self.calib_folder = calib_folder
        self.logger = init_logger(logger_location)


    def get_data(self):
        """Use the 'sense' method, put data in appropriating struct"""
        sense_data = self.sense()

        while sense_data ==  [0.0,0.0,0.0,0.0,0.0]:
            self.logger.info("Sense data is empty. Retrying")
            time.sleep(1.0)
            sense_data = self.sense()


        self.cur_loc = sense_data[0:2]
        self.cur_dir = sense_data[2:4]
        self.cur_angle = math.atan2(self.cur_dir[1], self.cur_dir[0]) * 180 / math.pi
        self.cur_readings = sense_data[4:]

    def calc_metrics(self, target: Target):
        """Calculate the distance and the angle towards the target"""

        # find dist
        trg = np.array((target.x, target.y))
        origin = np.asarray(self.cur_loc)
        dist = np.linalg.norm(trg - origin) # in [cm]


        # find current angle of the robot
        # angle_rad = math.atan2(self.cur_dir[1], self.cur_dir[0])
        # angle_deg = angle_rad*180/math.pi

        # find the angle between the current location and the target
        y_diff =  target.y - self.cur_loc[1]
        x_diff =  target.x - self.cur_loc[0]

        angle_rad = math.atan2(y_diff, x_diff)
        angle_deg = angle_rad*180/math.pi

        return (dist, angle_deg)


    def goto(self, target : Target):
        """Go to a specific point defined by [X, Y]
        target is of type 'POS'
        The path will be linear"""

        assert target.type == "POS"
        goal_reached = False
        max_absolute_encoder_val = 1000     # maximum value that can be given to encoder
        max_absolute_dist = 32              # corresponds to max_encoder_val                - MEASURE!
        reach_margin = 10                   # defines when we have reached the destination


        convert_rate = 0.1                  # [encoder clicks / cm]                         - MEASURE!!!

        max_dist_val = 20 # maximum allowed distance to travel at once
        # max_encoder_val = max_dist_val * convert_rate
        max_encoder_val = 500


        # This functin will implement intermediate 'targets' of rotational type to reach the goal

        # 1. Using the current location values, and the target values, 
        # calculate distance and angle
        distance, angle_deg = self.calc_metrics(target)

        # While the goal is not reached:

            # 2. Send the rotation command
            # 3. Send the movement command - should be done at segments if exceeds the limit?

            # 4. check if the goal was reached. If not - Do 2,3 again

        while not goal_reached:

            self.turn(angle_deg - self.cur_angle)
            time.sleep(1) # long enough?
            # calculate the required encoder value for the engines
            encoder_val = min(distance * convert_rate, max_encoder_val)

            self.drive(encoder_val, encoder_val)
            time.sleep(1)

            if distance <= reach_margin:
                goal_reached = True
            else:
                # calculate new distance and angle. 
                distance, angle_deg = self.calc_metrics(target)

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




    def self_calib_pos(self, range_cmd : tuple = (300, 1000) , step : int = 100):
        """This method will calibrate the encoder commands to drive to the 
        agent actual movement. Calibration is based on the measurements from the sensorts of the robot
        Since the command to movement relations is not linear, we will rather
        use the interpolation method to get the correct encoder command"""
        
        self.logger.info('Initializing the Position Command calibration')
        self.logger.info('In range of ({0} - {1}) with a step of {2}'.format(range_cmd[0], range_cmd[1] + step, step))

        os.makedirs(self.calib_folder, exist_ok=True)
        output_fd = open(os.path.join(self.calib_folder, 'pos_calib.csv'), 'w')
        output_fd.write('Encoder_cmd, Movement\n')
        
        try: 
            for current_step in range(range_cmd[0], range_cmd[1] + step, step):
                self.logger.info('Current step = {}'.format(current_step))
                self.get_data()
                init_pos = np.asarray(self.cur_loc)
                print(init_pos)

                self.drive(current_step, current_step)

                time.sleep(2.0) # should be enough?


                self.get_data()
                final_pos = np.asarray(self.cur_loc)
                print(final_pos)
                # calculate the movement
                dist = np.linalg.norm(final_pos - init_pos) # in [cm]
                output_fd.write('{0}, {1}\n'.format(current_step, str(round(dist, 2))))
                self.logger.info('\t\tCalculated distance: {}'.format(round(dist, 2)))
        except:
            var = traceback.format_exc()
            self.logger.info('Encountered exception: ' + var)


        finally:
            output_fd.close()

        self.logger.info('Calibration ended')


    def self_calib_rot(self, range_cmd : tuple = (300, 1000) , step : int = 100, direction : str = 'right'):
        """This method will calibrate the encoder commands to drive to the 
        agent rotation. Calibration is based on the measurements from the sensorts of the robot
        Since the command to movement relations is not linear, we will rather
        use the interpolation method to get the correct encoder command
        
        direction can be one of ['left', 'right']"""

        if direction == 'right':
            mutiplier = 1
        else:
            mutiplier = -1

        
        
        self.logger.info('Initializing the Rotation Command calibration')
        self.logger.info('In range of ({0} - {1}) with a step of {2}'.format(range_cmd[0], range_cmd[1], step))

        os.makedirs(self.calib_folder, exist_ok=True)
        output_fd = open(os.path.join(self.calib_folder, 'rot_calib.csv'), 'w')
        output_fd.write('Encoder_cmd, Rotation\n')
        
        try: 
            for current_step in range(range_cmd[0], range_cmd[1] + step, step):
                self.logger.info('Current step = {}'.format(current_step))
                self.get_data()
                init_angle = self.cur_angle
                print('init angle : ' + str(init_angle))

                # choosing a direction
                self.drive(current_step * mutiplier, -current_step* mutiplier)

                time.sleep(2.0) # should be enough?


                self.get_data()
                final_angle = self.cur_angle
                print('final angle : ' + str(final_angle))
                # calculate the rotation

                # if direction is 'right', angle has to grow
                if direction == 'right':
                    if final_angle > init_angle:
                        rotation = final_angle - init_angle
                    else:
                        final_angle = final_angle + 360
                        rotation = final_angle - init_angle
                else:
                    pass

                output_fd.write('{0}, {1}\n'.format(current_step, str(round(rotation, 2))))
                self.logger.info('\t\tCalculated rotation: {}'.format(round(rotation, 2)))
        except:
            var = traceback.format_exc()
            self.logger.info('Encountered exception: ' + var)


        finally:
            output_fd.close()

        self.logger.info('Calibration ended')