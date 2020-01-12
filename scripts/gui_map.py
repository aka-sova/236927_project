from struct import *
import socket
import time
from PIL import Image
from PIL import ImageTk

import tkinter as tk
from tkinter import Tk
from tkinter import filedialog
from tkinter import messagebox

import threading

import sys
import os

cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))
sys.path.append(os.path.join(cur_loc, 'scripts'))

import errno
from errnames import get_error_name

class GUI_MAP(tk.Tk):
    def __init__(self, name, map_bg_loc = ''):
        super().__init__()
        self.title(name)
        self.map_bg_loc = map_bg_loc
        self.x = 0
        self.y = 0
        self.cur_loc_shape = None

        self.create_menus()
        self.add_frames()

    def create_menus(self):

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

        self.create_side_frame()
        self.create_canvas_frame()

    def create_side_frame(self):
        """The side frame with some information on the current robot health"""

        self.Side_Frame = tk.Frame(master=self)
        self.Side_Frame.pack(side = tk.LEFT, padx = 20, pady = 10, fill = tk.BOTH)

        lbl = tk.Label(master = self.Side_Frame, text = "Hello", width = 25)
        lbl.pack(side = tk.LEFT) # or 'left'
        pass

    def create_canvas_frame(self):
        """The main frame with a mao on it, which will be updated in real time"""

        img_size = [500, 500]

        self.canvas_frame = tk.Frame(master = self)
        self.canvas_frame.pack(side = tk.RIGHT, padx = 20, pady = 20)

        # create the canvas
        self.map_cvs = tk.Canvas(master = self.canvas_frame, width=img_size[0], height=img_size[1])
        self.map_cvs.pack()

        # bring the background image
        from PIL import Image

        img = Image.open(self.map_bg_loc)
        img = img.resize((img_size[0],img_size[1]), Image.ANTIALIAS)
        photoImg =  ImageTk.PhotoImage(img)

        self.img = photoImg
        self.map_cvs.create_image((0, 0), anchor=tk.NW, image=self.img)

        #  we assume the map is correct and that the coordinate system origin is at [0, 0]

        # try to draw a shape of circle

        # delete and change its location 
        self.x = img_size[0]/2
        self.y = img_size[1]/2


        self.cur_loc_shape = self.map_cvs.create_rectangle( 
                         -5, -5, 5, 5, fill = "blue")

        self.map_cvs.move(self.cur_loc_shape, self.x, self.y)  # first move will move to the x, y args

        self.pose_thread = threading.Thread(target=self.update_pose_thread)
        self.pose_thread.start()

    def update_pose_thread(self):
        """thread for updating the location"""

        print("Opening a pose thread")

        HOST = '127.0.0.1'  # The server's hostname or IP address
        PORT = 65432        # The port used by the server

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST,PORT))
        sock.setblocking(0)

        print("Client socket established")

        while True:
            try:
                data, addr = sock.recvfrom(1024)
                if len(data)==0:
                    time.sleep(0.05)
                else:
                    data = unpack('lll', data)
                    print("Received data='{}'\n".format(data))

                    self.x += data[0]
                    self.movement(x = data[0])

            except socket.error as e:
                errnum = e.errno
                if errnum!=errno.EAGAIN:
                    reason=get_error_name(errnum)
                    # print("Socket Error ({}): {}".format(errnum,reason))
                time.sleep(0.05)

        # while True:
        #     time.sleep(1)
        #     self.movement(x = 5)


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

    def circle(self, canvas, x, y, r, fill):
        id = canvas.create_oval(x-r,y-r,x+r,y+r, fill=fill)
        return id



    def run_command(self):
        pass

    def exit_program(self):
        self.destroy()


    def about_win(self):
        messagebox.showinfo("Map of the robot location")



cur_loc = os.getcwd()
bg_src = os.path.join(cur_loc, 'scripts', 'map_src', 'map_clean.png')

master = GUI_MAP(name = "MAP", map_bg_loc=bg_src)

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

# The other (main) client will always WRITE into this file
# 


# Things we need to transmit 
#   1. Location, Angle, GOTO position (if exists)
#   2. MAP data  - should be done incrementally? Do it incrementally, but add button "GET FULL DATA"
#                   which will read the whole MAP data file


#   Main process will create temp file, then copy to TARGET file - once in 0.5 [s]. Overwrite it every time
#   VISUALIZER process will search for TARGET file every 0.2 [s]. Read it and and destroy
