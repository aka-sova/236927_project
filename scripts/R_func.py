
import sys
import os
import traceback

cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))

from api_src.udpclient import RClient
from map_func import Map
import C_CONSTANTS
from RRT import RRTStar, Target

import time
import logging
import numpy as np
import threading
import math
from scipy import interpolate
import csv

import socket

global logger

def normalize_angle(angle):
    if angle > 180 :
        return (angle - 360)
    elif angle < -180:
        return (angle + 360)
    else:
        return angle


def init_logger(logger_address : str):
    # create a logger

    logging.basicConfig(filename=logger_address, 
                        level=logging.DEBUG,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        datefmt='%m/%d %I:%M:%S',
                        filemode='w')
    logger = logging.getLogger(__name__)

    # put debug msgs here
    fh = logging.FileHandler(logger_address)
    fh.setLevel(logging.DEBUG)

    # put info messages in stream
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)

    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')  
    ch.setFormatter(formatter)
    fh.setFormatter(formatter)

    # add the handlers to logger
    logger.addHandler(ch)
    logger.addHandler(fh)

    # only debug will go into file

    # basic logger usage
    # logger.debug('This message should go to the log file')
    # logger.info('So should this')
    # logger.warning('And this, too')
    # logger.error('error message')
    # logger.critical('critical message')

    return logger  

class Obstacle_Interference(Exception):
   """Raised when the obstacle is in the way of the GOTO maneuver"""
   pass

      

class R_Client_Extend(RClient):
    def __init__(self, host, calib_folder, angles : list, logger_location, planner_logger_location, map_output_loc, map_output_temp_loc, map_inflated_output_loc, map_inflated_output_temp_loc, artifacts_loc, port, user_deprecate='',id_deprecate=''):
        super().__init__(host,port,user_deprecate='',id_deprecate='')
        self.goto_margin = 10  # margin to know we have arrived 
        self.angle_margin = 5 # [deg] 

        self.cur_loc = [-9999, -9999]   # values mean location is not set 
        self.cur_dir = []               # direction 
        self.cur_angle = -9999          # means angle is not set
        self.cur_readings = []
        self.status = 'idle'            # ['idle' / 'in_process']
        self.dest_reached = False
        self.current_target = None
        self.calib_folder = calib_folder
        self.logger = init_logger(logger_location)
        self.artifacts_loc = artifacts_loc

        self.map = Map(logger = self.logger,
                       sensor_angles = angles, 
                       output_loc = map_output_loc,
                       output_temp_loc=map_output_temp_loc,
                       output_inflated_loc = map_inflated_output_loc,
                       output_inflated_temp_loc = map_inflated_output_temp_loc,
                       artifacts_loc = artifacts_loc,
                       size_x = 500,
                       size_y = 500)
        
        planner_logger = init_logger(planner_logger_location)
        self.planner = RRTStar(logger = self.logger,
                               max_iter = 50,
                               expand_dis = 70,
                               path_resolution = 5.0,
                               connect_circle_dist = 50.0,
                               goal_sample_rate=20)

        # calib tables indicate which commands to give to achieve certain poses
        self.read_calib_tables()

    def read_calib_tables(self):
        """read the calibration tables for position and rotation and save them in the object"""


        # First for position
        pos_calib_file = os.path.join(self.calib_folder, 'pos_calib.csv')
        if os.path.exists(pos_calib_file):
            with open(pos_calib_file, mode='r') as infile:
                reader = csv.reader(infile)
                headers = next(reader, None) # read the headers so they won't be in dict
                mydict = {rows[0]:rows[1] for rows in reader}

            if len(mydict.keys()) > 0:
                pos_movement_vals = np.fromiter(mydict.values(), dtype=float)
                pos_encoder_vals = np.fromiter(mydict.keys(), dtype=float)

                self.pos_interp = interpolate.interp1d(pos_movement_vals, pos_encoder_vals)

            else:
                self.logger.warning("THE POSITION CALIBRATION IS NOT PERFORMED!")
        else:
            self.logger.warning("THE POSITION CALIBRATION IS NOT YET PERFORMED!")

        # Now for rotation
        rot_calib_file = os.path.join(self.calib_folder, 'rot_calib.csv')
        if os.path.exists(pos_calib_file):
            with open(rot_calib_file, mode='r') as infile:
                reader = csv.reader(infile)
                headers = next(reader, None) # read the headers so they won't be in dict
                mydict = {rows[0]:rows[1] for rows in reader}

            if len(mydict.keys()) > 0:
                rot_movement_vals = np.fromiter(mydict.values(), dtype=float)
                rot_encoder_vals = np.fromiter(mydict.keys(), dtype=float)

                self.rot_interp = interpolate.interp1d(rot_movement_vals, rot_encoder_vals)
            else:
                self.logger.warning("THE ROTATION CALIBRATION IS NOT YET PERFORMED!")
        else:
            self.logger.warning("THE ROTATION CALIBRATION IS NOT YET PERFORMED!")

    def get_data(self):
        """Use the 'sense' method, put data in appropriating struct"""
        sense_data = self.sense()

        while sense_data ==  [0.0,0.0,0.0,0.0,0.0] or (sense_data[0] == -9999 or sense_data[1] == -9999):
            self.logger.info("Sense data is erroneous. Retrying")

            if (sense_data ==  [0.0,0.0,0.0,0.0,0.0]):
                self.logger.info("Sense data is empty")
                
            if  (sense_data[0] == -9999 or sense_data[1] == -9999):
                self.logger.info("Sense data is -9999")


            time.sleep(1.0)
            sense_data = self.sense()

            print(sense_data)


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

        # corrdinate system is flipped, but so are the axes, and the robot coordinate system has 0 degrees looking up
        angle_rad = math.atan2(y_diff, x_diff)
        angle_deg = angle_rad*180/math.pi

        return (dist, angle_deg)


    def goto(self, target : Target):
        """Go to a specific point defined by [X, Y]
        target is of type 'POS'
        The path will be linear"""

        assert target.type == "POS"
        goal_reached = False

        # This functin will implement intermediate 'targets' of rotational type to reach the goal

        # 1. Using the current location values, and the target values, 
        # calculate distance and angle

        self.current_target = target
        self.get_data()
        self.logger.info("GOTO Target location received : [{0} {1}]".format(target.x, target.y))
        self.logger.debug("Current location : [{0} {1}]".format(self.cur_loc[0], self.cur_loc[1]))
        self.logger.debug("Current angle: {}".format(self.cur_angle))

        distance, angle_deg = self.calc_metrics(target)
        self.logger.debug("Calculated:\n\tdistance : {0}\n\tangle: {1}".format(distance, angle_deg))

        # While the goal is not reached:

            # 2. Send the rotation command
            # 3. Send the movement command - should be done at segments if exceeds the limit?

            # 4. check if the goal was reached. If not - Do 2,3 again

        while not goal_reached:

            # calculate the smallest angle required to perform
            angle_to_rotate = normalize_angle(angle_deg - self.cur_angle)


            self.logger.debug("Performing turn of {}".format(angle_to_rotate))

            turn_acceptable = False

            while not turn_acceptable:
                self.turn(angle_to_rotate)
                
                time.sleep(1.0) # long enough?
                # calculate the angle discrepancy
                self.get_data()
                angle_to_rotate = normalize_angle(angle_deg - self.cur_angle)
                discrepancy = abs(angle_to_rotate)
                self.logger.debug("Turn performed. current angle {},  discrepancy: {}".format(self.cur_angle, discrepancy))
                if discrepancy < 5:
                    turn_acceptable = True
                else:
                    self.logger.debug("Turn has too big discrepancy. Trying again")
                    self.logger.debug("Performing turn of {}".format(angle_to_rotate))


            
            # calculate the required encoder value for the engines
            self.drive_distance_long(distance)

            # check if reached the target
            self.get_data()
            distance, angle_deg = self.calc_metrics(target)

            self.logger.debug("STEP FINISHED")
            self.logger.debug("Current location : [{0} {1}]".format(self.cur_loc[0], self.cur_loc[1]))
            self.logger.debug("Target location : [{0} {1}]".format(target.x, target.y))
            self.logger.debug("Calculated:\n\tdistance : {0}\n\tangle: {1}".format(distance, angle_deg))

            if distance <= C_CONSTANTS.REACH_MARGIN:
                self.logger.debug("Distance is less than a margin of {}. GOTO done".format(C_CONSTANTS.REACH_MARGIN))
                goal_reached = True
            else:
                self.logger.debug("Distance is larger than a margin of {}. Performing additional steps".format(C_CONSTANTS.REACH_MARGIN))
                

        # when finished, clear the target, change the status

        self.get_data()
        self.logger.debug("\nGOTO FINISHED")
        self.logger.debug("Current location : [{0} {1}]".format(self.cur_loc[0], self.cur_loc[1]))
        self.logger.debug("Target location : [{0} {1}]".format(target.x, target.y))


        self.current_target = None
        self.status = 'idle'

    def populate_map(self):
        """This function will slowly spin the robot around itself,
        in order to gather information about the environment (update the map)
        """

        # make the loop for rotation and time.sleep

        self.current_target = None
        self.status = 'idle'

    def turn_small(self, angle: int):
        """Turn only 1 time in boundaries of the interpolation"""
        # interpolate the required command from the calibration information
        command_value = int(self.rot_interp(abs(angle)))


        if angle > 0 :
            self.drive(command_value, -command_value)
        elif angle < 0:
            self.drive(-command_value, command_value)
        else:
            pass

    def turn(self, angle: int):
        """Turn the robot by 'angle' [deg]
        We use the calibration data to interpolate the required encoder commands for the motors"""
        
        max_rot_angle = max(self.rot_interp.x)
        min_rot_angle = min(self.rot_interp.x)

        if abs(angle) > max_rot_angle:
            # drive in parts
            turn_segments = (int)(abs(angle) / max_rot_angle) + 1

            # make all segments of same angular distance
            turn_amount = (float)(angle / turn_segments)

            for _ in range(turn_segments):
                self.turn_small(turn_amount)
                time.sleep(1.0)
        elif abs(angle) < min_rot_angle:
            # do nothing. too small angle to move
            # perform 2 big turns, which will eventually bring the robot to correct angle

            # with base_turn = 30, and min_rot_angle ~= 20,  we won't rotate more than 50 deg, which is ok
            if angle > 0:
                base_turn = 30
            else:
                base_turn = -30

            self.turn_small(-base_turn)
            time.sleep(1.0)

            self.turn_small(angle + base_turn)
            time.sleep(1.0)
        else:
            # within the interpolation boundaries
            self.turn_small(angle)



    def drive_distance_long(self, distance: int):
        """Will drive any distance, even in parts"""
        max_driving_dist = max(self.pos_interp.x)

        if distance < max_driving_dist:
            self.drive_distance_short(distance)
            time.sleep(1.0)
        else:
            # drive in parts - we drive one segment less than needed on purpose!
            drive_segments = (int)(distance / max_driving_dist)

            # make all segments of same length
            segment_path = (float)(distance / drive_segments)

            for _ in range(drive_segments):
                self.drive_distance_short(segment_path)
                time.sleep(1.0)

            


    def drive_distance_short(self, distance: int):
        """Received distance in [cm] and make an appropriate command to move forward"""

        # will drive only the maximum distance from the interpolation values
        distance = min(max(self.pos_interp.x), distance)

        if distance < min(self.pos_interp.x):
            # don't drive if the distance is too small
            return

        command_value = int(self.pos_interp(abs(distance)))
        self.drive(command_value, command_value)

    def moveincircle(self, radius : int, init_dir : str):
        """Move the robot in a circle starting from the initial location 
        the center of the circle will be generated to the left / right from the robot"""

        pass

    def make_decision(self):
        """based on the current location, and current map
        make a decision, and return the GOTO next location"""

        self.current_target = None

    def reach_destination(self, target):
        """The main function which will look for a path to find to reach the goal"""

        self.logger.info('Initializing the Algorithm')
        self.dest_reached = False

        while not self.dest_reached:
            goto_list = self.planner.find_path(self.cur_loc, target = target, map = self.map)

            success_code = self.follow_goto_list(goto_list)   # 0 is success
            
            if success_code == 0:
                # check that the destination was indeed reached
                distance, angle_deg = self.calc_metrics(target)
                if distance < C_CONSTANTS.REACH_MARGIN:
                    self.dest_reached = True
                    self.logger.info('Reached the destination')
  

    def follow_goto_list(self, goto_list):
        
        for goto_target in goto_list:
            try:
                self.goto(goto_target)
            except Obstacle_Interference:
                self.logger.info("Found obstacle on the way. Recalculating the route")
                return 1

        return 0


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
                logger.info("INIT POS : {}".format(str(init_pos)))

                self.drive(current_step, current_step)

                time.sleep(2.0) # should be enough?


                self.get_data()
                final_pos = np.asarray(self.cur_loc)
                logger.info("END  POS : {}".format(str(final_pos)))
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
                logger.info('INIT  ANGLE: {}'.format(str(init_angle)))

                # choosing a direction
                self.drive(current_step * mutiplier, -current_step* mutiplier)

                time.sleep(2.0) # should be enough?


                self.get_data()
                final_angle = self.cur_angle
                logger.info('FINAL ANGLE: {}'.format(str(final_angle)))
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

    
    def open_controller(self):
        """Open the GUI controller, which will let me control the car"""

        # TODO fill this function
        pass

    def init_sense_thread(self):
        """Initialize the sensors thread that will operate always"""
        self.local_pose_thread=threading.Thread(target=self.sense_thread)
        self.local_pose_thread.start()

    def sense_thread(self):
        while True:
            self.get_data()
            time.sleep(C_CONSTANTS.SENSE_UPDATE_FREQ)

    def init_mapping_thread(self):
        """Initialize the sensors thread that will operate always"""
        self.local_pose_thread=threading.Thread(target=self.mapping_update_thread)
        self.local_pose_thread.start()  

        self.mapping_thread=threading.Thread(target=self.mapping_save_thread)
        self.mapping_thread.start()  

        if C_CONSTANTS.DEBUG == True:
            self.debug_map_save_thread=threading.Thread(target=self.mapping_save_png_thread)
            self.debug_map_save_thread.start()  


    def mapping_save_png_thread(self):
        """This is the function for a thread to save the maps as PNG files"""
        while True:
            self.map.save_maps_debug()
            time.sleep(C_CONSTANTS.MAP_SAVE_DEBUG_FREQ)


    def mapping_update_thread(self):
        """This is the function for a thread to update the map"""
        while True:
            self.map.update(cur_loc = self.cur_loc,
                            dir_angle = self.cur_angle,
                            meas = self.cur_readings)

            time.sleep(C_CONSTANTS.MAP_UPDATE_FREQ)

    def mapping_save_thread(self):
        """This is the function for a thread to update the map"""
        while True:
            self.map.save_clear_output_pickle()

            time.sleep(C_CONSTANTS.MAP_SAVE_FREQ)        

    def init_local_sockets(self):
        """Initialize the local sockets which will send the data to the visualizer"""

        self.local_pose_thread=threading.Thread(target=self.local_pose_loop)
        self.local_pose_thread.start()


    def local_pose_loop(self):
        print("Opening a server for pose transmission")

        max_tries_reconnect = 5
        current_tries_am = 0

        HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((HOST, PORT))
        sock.listen() #  enables a server to accept() connections.
        conn, addr = sock.accept()

        with conn:
            print('Connected by', addr)
            while True:

                if (current_tries_am > max_tries_reconnect):
                    # end current socket and thread, and wait for new connection
                    sock.close()
                    self.local_pose_thread=threading.Thread(target=self.local_pose_loop)
                    self.local_pose_thread.start()
                    break

                self.get_data()

                send_data_tuple = ((int)(self.cur_loc[0]), (int)(self.cur_loc[1]), (int)(self.cur_angle))

                # pack the target object. If exists
                if self.current_target != None:
                    if self.current_target.type == "POS":
                        target_tuple = ((int)(self.current_target.x), (int)(self.current_target.y))
                    else:
                        target_tuple = (-9999, -9999)
                else:
                    target_tuple = (-9999, -9999)

                total_tuple = send_data_tuple + target_tuple

                # a = calcsize('iiiii')
                # print("Sending " + str(a))
                data_to_send_bytes = pack('iiiii', total_tuple[0], 
                                                    total_tuple[1],
                                                    total_tuple[2],
                                                    total_tuple[3],
                                                    total_tuple[4])
                try:
                    conn.sendall(data_to_send_bytes)
                    # print("Sending data")
                    time.sleep(C_CONSTANTS.VIS_LOC_SEND_FREQ)
                    current_tries_am = 0 # succeeded to reconnect, set counter to 0
                except:
                    print("Transmission to Visualizer failed. Retrying ({}/{})".format(current_tries_am, max_tries_reconnect))
                    # print(total_tuple)
                    current_tries_am += 1
                    time.sleep(1.0)


                 