import traceback

import pickle
import math
from struct import *
import socket
import time
from PIL import Image
from PIL import ImageTk

import numpy as np
from numpy import genfromtxt

from R_func import init_logger
from map_func import Map

import timeit

import tkinter as tk
from tkinter import Tk
from tkinter import filedialog
from tkinter import messagebox

import copy

import threading

import sys
import os

cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))
sys.path.append(os.path.join(cur_loc, 'scripts'))

import errno
from errnames import get_error_name

import C_CONSTANTS

class GUI_MAP(tk.Tk):
    def __init__(self, name, map_bg_loc = '', map_input_loc = '', map_input_inflated_loc = '', logger_location = ''):
        super().__init__()
        self.title(name)
        self.map_bg_loc = map_bg_loc
        self.map_input_loc = map_input_loc
        self.map_input_inflated_loc = map_input_inflated_loc

        self.logger = init_logger(logger_location)

        self.size_x = 500
        self.size_y = 500
        self.obstacle_drawn_map = np.zeros((self.size_x,self.size_y), dtype=np.bool)
        self.obstacle_inflated_drawn_map = np.zeros((self.size_x,self.size_y), dtype=np.bool)
        self.obstacles_cvs_list = [] # list of obstacles objects - rectangles of size of pixel


        self.bin_map = np.zeros((self.size_x,self.size_y), dtype=np.bool)         # original sensors data
        self.filled_map = np.zeros((self.size_x,self.size_y), dtype=np.bool)      # map with 'filled' gaps between measuremenets
        self.inflated_map = np.zeros((self.size_x,self.size_y), dtype=np.bool)    # map with 'inflated' obstacle

        self.x = -9999
        self.y = -9999
        self.angle = -9999

        self.connected = False

        self.target_x = -9999
        self.target_y = -9999
        self.cur_loc_shape = None

        self.create_menus()
        self.add_frames()


    def create_menus(self):

        self.logger.info("Creating MENUS")

        Main_menu = tk.Menu(master = self)

        Program_options_menu = tk.Menu(master = Main_menu, tearoff = 0)

        Program_options_menu.add_command(label = "Options")
        Program_options_menu.add_command(label = "RUN", command = self.run_command)
        Program_options_menu.add_separator()
        Program_options_menu.add_command(label = "EXIT", command = self.exit_program)

        Main_menu.add_cascade(label = "Program", menu = Program_options_menu)
        Main_menu.add_command(label = "About", command = self.about_win)

        self.config(menu = Main_menu)

    def add_frames(self):

        self.logger.info("Creating FRAMES")

        self.create_side_frame()
        self.create_canvas_frame()

    def create_side_frame(self):
        """The side frame with some information on the current robot health"""

        self.Side_Frame = tk.Frame(master=self)
        self.Side_Frame.pack(side = tk.LEFT, padx = 20, pady = 10, fill = tk.BOTH)



        self.status_var = tk.StringVar()
        self.x_var = tk.StringVar()
        self.y_var = tk.StringVar()
        self.angle_var = tk.StringVar()

        self.goto_var_x = tk.StringVar()
        self.goto_var_y = tk.StringVar()
        

        self.create_label_frame(master = self.Side_Frame, label_text = "STATUS: ", label_target = self.status_var)
        self.create_label_frame(master = self.Side_Frame, label_text = "", label_target = None)
        self.create_label_frame(master = self.Side_Frame, label_text = "X     : ", label_target = self.x_var)
        self.create_label_frame(master = self.Side_Frame, label_text = "Y     : ", label_target = self.y_var)
        self.create_label_frame(master = self.Side_Frame, label_text = "ANGLE : ", label_target = self.angle_var)
        self.create_label_frame(master = self.Side_Frame, label_text = "", label_target = None)
        self.create_label_frame(master = self.Side_Frame, label_text = "GOTO X:", label_target = self.goto_var_x)
        self.create_label_frame(master = self.Side_Frame, label_text = "GOTO Y:", label_target = self.goto_var_y)
        

        self.status_var.set("DISCONNECTED")
        self.x_var.set(self.x)
        self.y_var.set(self.y)
        self.angle_var.set(self.angle)

        self.goto_var_x.set(self.target_x)
        self.goto_var_y.set(self.target_y)

    @staticmethod
    def create_label_frame(master, label_text : str, label_target):

        dump_frame = tk.Frame(master = master)
        dump_frame.pack(side = tk.TOP)

        dump_lbl = tk.Label(master = dump_frame, text = label_text, width = 15)
        dump_lbl.pack(side = tk.LEFT)

        dump_lbl = tk.Label(master = dump_frame, textvariable = label_target, width = 15)
        dump_lbl.pack(side = tk.RIGHT)


    @staticmethod
    def create_entry_frame(master, label_text : str, label_target):

        dump_frame = tk.Frame(master = master)
        dump_frame.pack(side = tk.TOP)

        dump_lbl = tk.Label(master = dump_frame, text = label_text, width = 15)
        dump_lbl.pack(side = tk.LEFT)

        dump_lbl = tk.Entry(master = dump_frame, textvariable = label_target, width = 15)
        dump_lbl.pack(side = tk.RIGHT)


    def create_canvas_frame(self):
        """The main frame with a mao on it, which will be updated in real time"""

        self.logger.info("Creating CANVAS")    

        self.canvas_frame = tk.Frame(master = self)
        self.canvas_frame.pack(side = tk.RIGHT, padx = 20, pady = 20)

        # create the canvas
        self.map_cvs = tk.Canvas(master = self.canvas_frame, width=self.size_x, height=self.size_y, bg="white")
        self.map_cvs.pack()

        self.create_grid_graphics()



        self.cur_loc_object = None
        self.cur_dir_object = None
        self.cur_trg_object = None
        self.cur_obstacles_map = None
        self.cvs_bg_img = None

        self.logger.info("Initializing the agent pose + go drawing thread")
        self.pose_thread = threading.Thread(target=self.update_pose_thread)
        self.pose_thread.start()

        self.logger.info("Initializing the obstacles drawing threads")

        if C_CONSTANTS.DRAW_REGULAR_OBSTACLES :
            self.map_draw_thread = threading.Thread(target=self.draw_map_proc)
            self.map_draw_thread.start()

        if C_CONSTANTS.DRAW_INFLATED_OBSTACLES :     
            self.map_bg_draw_thread = threading.Thread(target=self.draw_map_bg_proc)
            self.map_bg_draw_thread.start()

    def create_grid_graphics(self):
        """Create origin, grid, etc"""
        # draw the origin and coordinate system with arrows
        # the X arrow - this is '-Y' in the canvas plane
        self.origin_X = self.map_cvs.create_line(self.size_y/2, 
                                                       self.size_x/2,
                                                       self.size_y/2, 
                                                       self.size_x/2 - 60, 
                                                       arrow=tk.LAST)

        self.map_cvs.create_text(self.size_y/2,
                                self.size_x/2 - 80, 
                                fill="darkblue",font="Arial 14 bold",
                                text="X")
                                
        # the Y arrow - this is '-Y' in the canvas plane
        self.origin_Y = self.map_cvs.create_line(self.size_y/2, 
                                                       self.size_x/2,
                                                       self.size_y/2 + 60, 
                                                       self.size_x/2, 
                                                       arrow=tk.LAST)           

        self.map_cvs.create_text(self.size_y/2 + 80,
                                self.size_x/2 , 
                                fill="darkblue",font="Arial 14 bold",
                                text="Y")

        self.create_gridlines(line_distance = 100)   


    def update_pose_thread(self):
        """thread for updating the location"""

        self.logger.info("Opening a pose thread")
        

        HOST = '127.0.0.1'  # The server's hostname or IP address
        PORT = 65432        # The port used by the server

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # trying to connect until the server listens
        while not self.connected:
            try:
                sock.connect((HOST,PORT))
                sock.setblocking(0)
                self.connected = True
            except socket.error as e:
                errnum = e.errno
                reason=get_error_name(errnum)
                self.logger.info("Server not found. Reason: " + reason)

        self.logger.info("Client socket established")
        self.status_var.set("CONNECTED")

        while True:
            try:
                data, addr = sock.recvfrom(20)
                if len(data)==0:
                    time.sleep(0.05)
                else:
                    # data = data[-20:]
                    # print("Received : " + str(len(data)))
                    data = unpack('iiiii', data)
                    self.logger.debug("Received data={}".format(data))

                    self.x = data[1] + self.size_y/2 # receive the updated position
                    self.y = - data[0] + self.size_x/2 # receive the updated position
                    self.angle = data[2] # receive the updated angle

                    self.target_x = data[4]
                    self.target_y = - data[3]

                    self.update_labels()

                    if self.x != -9999 and self.y != -9999 and self.angle != -9999:
                        self.create_cur_shape()

            except socket.error as e:
                errnum = e.errno
                if errnum!=errno.EAGAIN:
                    reason=get_error_name(errnum)
                    # print("Socket Error ({}): {}".format(errnum,reason))
                # time.sleep(1.0)

    def update_labels(self):
        """Update the GUI labels"""

        x_robot, y_robot = self.to_cam_coords(self.x, self.y)

        self.x_var.set(x_robot)
        self.y_var.set(y_robot)
        self.angle_var.set(self.angle)
        self.goto_var_x.set(self.target_x)
        self.goto_var_y.set(self.target_y)

    def to_cam_coords(self, col, row):
        """Change to csv coordinate system (only absolute values)"""

        x = self.size_x/2 - row 
        y = - self.size_y/2 + col 

        return x, y


    def create_cur_shape(self):
        """Delete the current location shape, and create new one at x,y"""

        start = timeit.default_timer()

        if self.cur_loc_object != None:
            self.map_cvs.delete(self.cur_loc_object)

        if self.cur_dir_object != None:
            self.map_cvs.delete(self.cur_dir_object)   
            
        if self.cur_trg_object != None:
            self.map_cvs.delete(self.cur_trg_object)   

        # create ab object which symbolizes the current agent location
        self.cur_loc_object = self.map_cvs.create_rectangle( 
                         -5, -5, 5, 5, fill = "blue")
        self.map_cvs.move(self.cur_loc_object, self.x, self.y)

        # create an arrow which shows its direction
        arrow_length = 70 # px
        self.cur_dir_object = self.map_cvs.create_line(self.x, 
                                                       self.y,
                                                       self.x + arrow_length*math.sin(self.angle*math.pi/180), 
                                                       self.y - arrow_length*math.cos(self.angle*math.pi/180), 
                                                       arrow=tk.LAST,
                                                       fill="blue")

        if self.target_x != -9999 and self.target_y != -9999:
            # meaning we have a POS target
            # draw it with a red dot
            r = 5
            self.cur_trg_object = self.map_cvs.create_oval(self.target_x-r,
                                                           self.target_y-r,
                                                           self.target_x+r,
                                                           self.target_y+r, 
                                                           fill="#5E5E5E")

        stop = timeit.default_timer()
        self.logger.info('[DRAWING] Drawing self + GOTO. Time elapsed :  {}'.format(stop - start)) 

    def create_gridlines(self, line_distance):
        """Add gridlines to canvas"""

        x0 = (self.size_x/2) % 100
        y0 = (self.size_y/2) % 100

        if x0 != 0:
            self.map_cvs.create_line(x0, 0, x0, self.size_y, fill="#476042")

        if y0 != 0:
            self.map_cvs.create_line(0, y0, self.size_x, y0, fill="#476042")

        # vertical lines at an interval of "line_distance" pixel
        for x in range(line_distance, self.size_x ,line_distance):
            self.map_cvs.create_line(x0 + x, 0, x0 + x, self.size_y, fill="#476042")
        # horizontal lines at an interval of "line_distance" pixel
        for y in range(line_distance,self.size_y,line_distance):
            self.map_cvs.create_line(0, y0 + y, self.size_x, y0 + y, fill="#476042")


    def draw_map_proc(self):
        """Thread process to draw the obstacles from csv"""
        while True:
            self.draw_the_obstacles_from_csv()
            time.sleep(C_CONSTANTS.VIS_MAP_OBSTACLES_UPDATE)

    def draw_map_bg_proc(self):
        """Thread process to draw the obstacles from csv"""
        while True:
            self.draw_inflated_obstacles()
            time.sleep(C_CONSTANTS.VIS_MAP_BG_UPDATE)

    def draw_inflated_obstacles(self):
        """Draw the inflated obstacles from the output PNG file from main process"""

        start = timeit.default_timer()

        from PIL import Image

        if os.path.isfile(self.map_bg_loc):


            self.clear_obstacle_pixels()
            
            if self.cvs_bg_img != None:
                self.map_cvs.delete(self.cvs_bg_img)   


            img = Image.open(self.map_bg_loc)
            img = img.resize((self.size_x,self.size_y), Image.ANTIALIAS)
            photoImg =  ImageTk.PhotoImage(img)

            self.img = photoImg
            self.cvs_bg_img = self.map_cvs.create_image((0, 0), anchor=tk.NW, image=self.img)
            self.update_idletasks()

        self.create_grid_graphics()
        
        # clear the drawn obstacles, they have to be redrawn
        self.obstacle_drawn_map = np.zeros((self.size_x,self.size_y), dtype=np.bool)
        self.draw_the_obstacles_from_csv()

        stop = timeit.default_timer()
        self.logger.info('[DRAWING] Changing background and all connected. Time elapsed :  {}'.format(stop - start)) 



        
    def clear_obstacle_pixels(self):
        """remove all the drawn obstacle pixels"""
        for obstacle in self.obstacles_cvs_list:
            self.map_cvs.delete(obstacle)

        self.obstacles_cvs_list = []



    def draw_the_obstacles_from_csv(self):
        """Main method to draw the obstacles
        from the input csv format"""

        start = timeit.default_timer()





        # if self.cur_obstacles_map != None:
        #     self.map_cvs.delete(self.cur_dir_object)  

        if not os.path.exists(self.map_input_loc):
            return

        # if not os.path.exists(self.map_input_inflated_loc):
        #     return

        with open(self.map_input_loc, "rb" ) as f:
            self.bin_map = pickle.load(f)

        # with open(self.map_input_inflated_loc, "rb" ) as f:
        #     self.inflated_map = pickle.load(f)      

        # FOR CSV:
        
        # try:
        #     bin_map = np.genfromtxt(self.map_input_loc, delimiter=',')
        #     # print("Obstacles data received! Shape = [{} on {}]".format(bin_map.shape[0], bin_map.shape[1]))
        # except:
        #     var = traceback.format_exc()
        #     self.logger.WARNING('Reading from CSV not successful. exception: ' + var)
        #     return

        stop = timeit.default_timer()
        self.logger.info('[DRAWING] Reading raw obstacles PICKLE file. Time elapsed :  {}'.format(stop - start)) 

        # Drawingt the RAW obstacles data

        start = timeit.default_timer()

        for row in range(self.bin_map.shape[0]):
            for col in range(self.bin_map.shape[1]):
                # check that obstacle wasn't drawn yet
                if self.bin_map[row][col] == 1 and self.obstacle_drawn_map[row][col] == False:
                    # print("Found obstacle at row {} col {}".format(row,col))
                    # print("Placing at {}".format(row + self.size_x/2 - 10))

                    # print("Obstacle at ROW {} COL {}".format(row,col))
                    self.obstacle_drawn_map[row][col] = True # mark that this obstacle is drawn already

                    new_obstacle_pixel = self.map_cvs.create_rectangle(col - 1, row-1, 
                                                  col + 1, row+1, fill = "#000538")
                    self.obstacles_cvs_list.append(new_obstacle_pixel)

                # if self.inflated_map[row][col] == 1 and self.obstacle_inflated_drawn_map[row][col] == False and self.bin_map[row][col] == 0:
                #     self.obstacle_inflated_drawn_map[row][col] = True # mark that this obstacle is drawn already

                #     self.map_cvs.create_rectangle(col - 1, row-1, 
                #                                   col + 1, row+1, outline = "red")


        self.update_idletasks()



        stop = timeit.default_timer()
        self.logger.info('[DRAWING] Drawing raw obstacles. Time elapsed :  {}'.format(stop - start)) 




    def movement(self, x = 0, y = 0):
        """Change the location of the object on canvas to the actual location"""
        self.map_cvs.move(self.cur_loc_shape, x, y)

    # for motion in negative x direction 
    def left(self, event): 
        print(event.keysym)
        self.x += 5
        self.movement(x = -5)
      
    # for motion in positive x direction 
    def right(self, event): 
        print(event.keysym) 
        self.x += 5
        self.movement(x = +5)
      
    # for motion in positive y direction 
    def up(self, event): 
        print(event.keysym) 
        self.y -= 5
        self.movement(y = -5)
      
    # for motion in negative y direction 
    def down(self, event): 
        print(event.keysym)
        self.y += 5
        self.movement(y = +5)


    def run_command(self):
        pass

    def exit_program(self):
        self.destroy()


    def about_win(self):
        messagebox.showinfo("Map of the robot location")



if __name__ == '__main__':

    cur_loc = os.getcwd()
    bg_src = os.path.join(cur_loc, 'artifacts', 'inflated_map.png')
    map_input_loc = os.path.join(cur_loc, 'output','map.p')
    map_input_inflated_loc = os.path.join(cur_loc, 'output','map_inflated.p')
    logger_location = os.path.join(cur_loc, 'artifacts','logger_vizual.log')

    master = GUI_MAP(name = "MAP", map_bg_loc=bg_src, map_input_loc = map_input_loc, map_input_inflated_loc = map_input_inflated_loc, logger_location = logger_location)

    # This will bind arrow keys to the tkinter 
    # toplevel which will navigate the image or drawing 
    # master.bind("<KeyPress-Left>", lambda e: master.left(e)) 
    # master.bind("<KeyPress-Right>", lambda e: master.right(e)) 
    # master.bind("<KeyPress-Up>", lambda e: master.up(e)) 
    # master.bind("<KeyPress-Down>", lambda e: master.down(e)) 

    master.mainloop()


    ### IDEA on how to received and transmit data:
    # This gui will be a SEPARATE program - call it VISUALIZER 
    # which will constantly read for the file with 
    # Coordinates, current angle, current GOTO location

    # The other (main) client will always trasmit to this application


    # Things we need to transmit 
    #   1. Location, Angle, GOTO position  - done through socket
    #   2. MAP data  - done through pickle file
    #   3. inflated map data - through changing background image, if needed