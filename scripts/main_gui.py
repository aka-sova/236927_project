


import tkinter as tk
from tkinter import Tk
from tkinter import filedialog
from tkinter import messagebox

import sys
import os

cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))
sys.path.append(os.path.join(cur_loc, 'scripts'))

import errno
from errnames import get_error_name

from R_func import init_logger, R_Client_Extend
from map_func import Map
from gui_map import GUI_MAP
from RRT import Target

class Parser_dummy(object):
    def __init__(self):
        pass



class Main_Gui(tk.Tk):
    def __init__(self, name, args):
        super().__init__()
        self.title(name)
        self.logger = init_logger(args.logger_location)
        self.args = args
        self.stop_command_activated = False

        self.stop_all_threads = False


        self.create_menus()
        self.add_frames()




    def create_menus(self):

        self.logger.info("Creating MENUS")

        Main_menu = tk.Menu(master = self)

        Program_options_menu = tk.Menu(master = Main_menu, tearoff = 0)

        Program_options_menu.add_command(label = "Options")
        Program_options_menu.add_command(label = "RUN", command = self.RUN_COMMAND)
        Program_options_menu.add_separator()
        Program_options_menu.add_command(label = "EXIT", command = self.exit_program)

        Main_menu.add_cascade(label = "Program", menu = Program_options_menu)
        Main_menu.add_command(label = "About", command = self.about_win)

        self.config(menu = Main_menu)

    def add_frames(self):

        self.logger.info("Creating FRAMES")

        self.create_status_comm_frame()     # defne the communication status and params
        self.create_status_robot_frame()    # define the robot telemetry and current status
        self.create_activation_frame()      # input GOAL frame with params, RUN & STOP btns


    def create_status_comm_frame(self):
        """ defne the communication status and params"""

        self.Comm_Status_Frame = tk.Frame(master=self)
        self.Comm_Status_Frame.config(highlightthickness=1, highlightcolor="black", highlightbackground="black")
        self.Comm_Status_Frame.pack(side = tk.LEFT, padx = 20, pady = 10, fill = tk.BOTH)

        status_lbl = tk.Label(master = self.Comm_Status_Frame, text = "COMMUNICATION STATUS", width = 25)
        status_lbl.pack(side = tk.TOP)

        self.Connect_frame = tk.Frame(master=self.Comm_Status_Frame)
        self.Connect_frame.pack(side = tk.TOP, padx = 20, pady = 10, fill = tk.BOTH)

        self.ip_var = tk.StringVar()
        self.port_var = tk.StringVar()

        GUI_MAP.create_entry_frame(master = self.Connect_frame, label_text = "IP: ", label_target = self.ip_var)
        GUI_MAP.create_entry_frame(master = self.Connect_frame, label_text = "PORT: ", label_target = self.port_var)
        
        self.ip_var.set("192.168.1.158")
        self.port_var.set("2777")

        connect_btn = tk.Button(master = self.Connect_frame, text ="Connect", command = self.connect_robot)
        connect_btn.pack()

        # create status labels
        self.robot_status_var = tk.StringVar()
        self.vizual_status_var = tk.StringVar()
        self.sensors_status_var = tk.StringVar()

        GUI_MAP.create_label_frame(master = self.Connect_frame, label_text = "Robot: ", label_target = self.robot_status_var)
        GUI_MAP.create_label_frame(master = self.Connect_frame, label_text = "Vizualization: ", label_target = self.vizual_status_var)
        GUI_MAP.create_label_frame(master = self.Connect_frame, label_text = "Sensors: ", label_target = self.sensors_status_var)


        self.robot_status_var.set("Disconnected")
        self.vizual_status_var.set("Disconnected")
        self.sensors_status_var.set("Disconnected")



    def create_status_robot_frame(self):
        """ define the robot telemetry and current status"""

        self.Robot_Status_Frame = tk.Frame(master=self)
        self.Robot_Status_Frame.config(highlightthickness=1, highlightcolor="black", highlightbackground="black")
        self.Robot_Status_Frame.pack(side = tk.LEFT, padx = 20, pady = 10, fill = tk.BOTH)

        status_lbl = tk.Label(master = self.Robot_Status_Frame, text = "ROBOT STATUS", width = 15)
        status_lbl.pack(side = tk.TOP)        

        self.lbl_pose_x = tk.StringVar()
        self.lbl_pose_y = tk.StringVar()
        self.lbl_angle = tk.StringVar()
        self.lbl_status = tk.StringVar()
        self.lbl_goto_x = tk.StringVar()
        self.lbl_goto_y = tk.StringVar()

        GUI_MAP.create_label_frame(master = self.Robot_Status_Frame, label_text = "X: ", label_target = self.lbl_pose_x)
        GUI_MAP.create_label_frame(master = self.Robot_Status_Frame, label_text = "Y: ", label_target = self.lbl_pose_y)
        GUI_MAP.create_label_frame(master = self.Robot_Status_Frame, label_text = "Angle: ", label_target = self.lbl_angle)
        GUI_MAP.create_label_frame(master = self.Robot_Status_Frame, label_text = "Status: ", label_target = self.lbl_status)
        GUI_MAP.create_label_frame(master = self.Robot_Status_Frame, label_text = "", label_target = None)
        GUI_MAP.create_label_frame(master = self.Robot_Status_Frame, label_text = "next GOTO X: ", label_target = self.lbl_goto_x)
        GUI_MAP.create_label_frame(master = self.Robot_Status_Frame, label_text = "next GOTO Y: ", label_target = self.lbl_goto_y)


        self.lbl_pose_x.set("N/A")
        self.lbl_pose_y.set("N/A")
        self.lbl_angle.set("N/A")
        self.lbl_status.set("N/A")
        self.lbl_goto_x.set("N/A")
        self.lbl_goto_y.set("N/A")


    def create_activation_frame(self):
        """ input GOAL frame with params, RUN & STOP btns"""

        self.Activation_Frame = tk.Frame(master=self)
        self.Activation_Frame.config(highlightthickness=1, highlightcolor="black", highlightbackground="black")
        self.Activation_Frame.pack(side = tk.LEFT, padx = 20, pady = 10, fill = tk.BOTH)

        activation_lbl = tk.Label(master = self.Activation_Frame, text = "CONTROL PANEL", width = 25)
        activation_lbl.pack(side = tk.TOP)      

        self.destination_X_var = tk.StringVar()
        self.destination_Y_var = tk.StringVar()

        GUI_MAP.create_entry_frame(master = self.Activation_Frame, label_text = "Destination X: ", label_target = self.destination_X_var)
        GUI_MAP.create_entry_frame(master = self.Activation_Frame, label_text = "Destination Y: ", label_target = self.destination_Y_var)
        
        self.destination_X_var.set("")
        self.destination_X_var.set("")

        RUN_btn = tk.Button(master = self.Activation_Frame, text ="RUN", command = self.RUN_COMMAND, width = 25, pady=5)
        STOP_btn = tk.Button(master = self.Activation_Frame, text ="STOP", command = self.STOP_COMMAND, width = 25,pady=5)
        EXIT_btn = tk.Button(master = self.Activation_Frame, text ="EXIT", command = self.exit_program, width = 25,pady=5)

        RUN_btn.pack()
        STOP_btn.pack()
        EXIT_btn.pack()


    def connect_robot(self):


        host = self.ip_var.get()
        port = (int)(self.port_var.get())

        self.logger.info("Conneting to {} - {}".format(host, port))

        self.robot = R_Client_Extend(host = host, 
                        port = port,
                        angles = [45, 0, -45],
                        calib_folder = self.args.calib_folder,
                        logger = self.logger,
                        map_output_loc = self.args.map_output_loc,
                        map_output_temp_loc = self.args.map_output_temp_loc,
                        map_inflated_output_loc = self.args.map_inflated_output_loc,
                        map_inflated_output_temp_loc = self.args.map_inflated_output_temp_loc,
                        artifacts_loc = self.args.artifacts_loc,
                        main_gui = self)


        self.robot.connect()
        self.robot_status_var.set("Connected")

        # self.vizual_status_var.set("Disconnected")
        # self.sensors_status_var.set("Disconnected")

        self.robot.init_sense_thread()           # get info from the sensors
        self.robot.init_mapping_thread()         # update & save map
        self.robot.init_local_sockets()          # send the Pose to the visualizer


    def RUN_COMMAND(self):

        self.stop_command_activated = False

        target_x = (int)(self.destination_X_var.get())
        target_y = (int)(self.destination_Y_var.get())

        destination_location = [target_x, target_y]

        target = Target(target_type = "POS", target_vals = destination_location)

        self.robot.reach_destination(target = target)

    def STOP_COMMAND(self):
        self.stop_command_activated = True  # constantly check if was activated inside the robot thread

    def exit_program(self):
        if 'self.robot' in locals():
            self.robot.terminate()

        self.destroy()
        sys.exit()

    def about_win(self):
        messagebox.showinfo("GUI for robot obstacles avoidance task. Course 236927. Alexander Shender, 2020")


if __name__ == '__main__':

    cur_loc = os.getcwd()

    # create output/artifacts dirs
    os.makedirs(os.path.join(cur_loc, 'artifacts'), exist_ok=True)
    os.makedirs(os.path.join(cur_loc, 'output'), exist_ok=True)

    # put all arguments into 1 struct
    args = Parser_dummy()
    
    args.calib_folder = os.path.join(cur_loc, 'scripts', 'calib')
    args.logger_location = os.path.join(cur_loc, 'artifacts','logger.log')

    args.map_output_loc = os.path.join(cur_loc, 'output','map.p')
    args.map_output_temp_loc = os.path.join(cur_loc, 'output','map_temp.p')

    args.map_inflated_output_loc = os.path.join(cur_loc, 'output','map_inflated.p')
    args.map_inflated_output_temp_loc = os.path.join(cur_loc, 'output','map_inflated_temp.p')

    args.artifacts_loc = os.path.join(cur_loc, 'artifacts')


    main_gui = Main_Gui("Robot GUI", args)

    main_gui.mainloop()
