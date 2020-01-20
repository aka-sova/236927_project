


import os
import sys
cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))
sys.path.append(os.path.join(cur_loc, 'scripts'))


from udpclient import RClient
from map_func import Map
from R_func import Target
import C_CONSTANTS

import numpy as np
import random
import copy
import pickle

class RRTStar(object):

    class Node:
        """
        Class defining an RRT Node
        """

        def __init__(self, row, col):
            self.row = row
            self.col = col
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0            

    def __init__(self, logger, max_iter, expand_dis, path_resolution, connect_circle_dist, goal_sample_rate):

        self.logger = logger
        self.max_iter = max_iter
        self.path_resolution = path_resolution
        self.connect_circle_dist = connect_circle_dist
        self.goal_sample_rate = goal_sample_rate

        self.start = None
        self.end = None
        # self.min_rand = rand_area[0]
        # self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.node_list = []


        self.map = None
        self.map_size_x = None
        self.map_size_y = None
        self.output_togo_list = []

        self.collision_margin = 1


    def find_path(self, init_loc : list, dest_loc : list, map : Map):
        """Use the R* algorithm to find the path to the destination
        Input: 

        init_loc - initial location [ROW, COL]
        dest_loc - final location   [ROW, COL]
        map      - Map object, containing all the map information

        Output:
        list of GOTO Target objects defining the required path
        """


        self.logger.info("Finding a path from [{} {}] to [{} {}]".format(init_loc[0], init_loc[1], dest_loc[0], dest_loc[1]))

        self.map = map
        self.map_size_x = map.bin_map.shape[0]
        self.map_size_y = map.bin_map.shape[1]
        self.output_togo_list = []

        # 1. Check if the path is obstacle free
        self.start = RRTStar.Node(init_loc[0], init_loc[1])
        self.end = RRTStar.Node(dest_loc[0], dest_loc[1])

        
        if self.check_line_collision(self.start, self.end):
            self.logger.info("No collision spotted, path is free")
            self.output_togo_list.append(Target(target_type = "POS", target_vals = [self.end.x, self.end.y]))
            return self.output_togo_list

        else:
            self.logger.info("Collision detected, initializing algorithm")
            # 2.1 if not, generate a tree
            self.node_list = [self.start]
            for i in range(self.max_iter):

                self.logger.info("Iter:", i, ", number of nodes:", len(self.node_list))

                rnd = self.get_random_node()
                nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
                new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)

                if self.check_collision(new_node, self.obstacle_list):
                    near_inds = self.find_near_nodes(new_node)
                    new_node = self.choose_parent(new_node, near_inds)
                    if new_node:
                        self.node_list.append(new_node)
                        self.rewire(new_node, near_inds)

                if animation and i % 5 == 0:
                    self.draw_graph(rnd)

                if (not search_until_max_iter) and new_node:  # check reaching the goal
                    last_index = self.search_best_goal_node()
                    if last_index:
                        return self.generate_final_course(last_index)

                print("reached max iteration")


            # 2.2 Optimize the path, connect nodes which have no obstacles in between

        print("Finito")

    
    def check_static_collision(self, node : RRTStar.Node):
        """Check is the node is inside the 'occupied' pixel
        return TRUE if no collision takes place"""

        if self.map.bin_map[node.x][node.y] == 0:
            return True
        return False

    def check_line_collision(self, node_start : RRTStar.Node, node_finish : RRTStar.Node):
        """Check if the line between the nodes crosses any of the occupied pixels
        return TRUE if path is collision FREE"""

        # for each obstacle pixel, check if it intercepts the line between 2 nodes
        obstacles = np.transpose(np.nonzero(self.map.inflated_map))

        # build a line function describing the path from start to finish
        # X = col
        # Y = row

        min_row = min(node_finish.row, node_start.row)
        max_row = max(node_finish.row, node_start.row)
        min_col = min(node_finish.col, node_start.col)
        max_col = max(node_finish.col, node_start.col)

        for obstacle in obstacles:
            nonzero_row = obstacle[0]
            nonzero_col = obstacle[1]

            # 1. check if the obstacle is outside of the region
            if nonzero_row < min_row or nonzero_row > max_row or nonzero_col < min_col or nonzero_col > max_col:
                continue
        
            # 2. check what is the discrepancy of it if we put it into the line function
            p1 = np.array([node_start.col, node_start.row])
            p2 = np.array([node_finish.col, node_finish.row])
            p3 = np.array([nonzero_col,nonzero_row])

            distance = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)

            if distance < self.collision_margin:
                return False

        return True

    def get_random_node(self):
        """Generate a new random node"""

        rnd = self.Node(random.uniform(0, self.map_size_x),
                        random.uniform(0, self.map_size_y))

        return rnd


