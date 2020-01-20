


import os
import sys
cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))
sys.path.append(os.path.join(cur_loc, 'scripts'))


from api_src.udpclient import RClient
from map_func import Map


import C_CONSTANTS

import numpy as np
import math
import random
import copy
import pickle

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
                new_node = self.steer(from_node = self.node_list[nearest_ind], to_node =  rnd, extend_length = self.expand_dis)

                if self.check_static_collision(new_node):
                    near_inds = self.find_near_nodes(new_node)
                    new_node = self.choose_parent(new_node, near_inds)
                    if new_node:
                        self.node_list.append(new_node)
                        self.rewire(new_node, near_inds)


            self.logger.info("Tree construction completed")

            # 2.3 Choose the closest node to the target
            self.logger.info("Finding a route") 
            self.output_togo_list = self.find_route()

            if self.output_togo_list is not []:
                # remove nodes with no collision
                self.optimize_path()
            else:
                self.logger.WARNING("Route was not found! ")                         

            # 2.2 Optimize the path, connect nodes which have no obstacles in between

        self.logger.info("Path generation successful")     
        return self.output_togo_list

    
    def check_static_collision(self, node):
        """Check is the node is inside the 'occupied' pixel
        return TRUE if no collision takes place"""

        if self.map.bin_map[node.x][node.y] == 0:
            return True
        return False

    def check_line_collision(self, node_start, node_finish):
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


    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        """Calculate minimum distance node. Return the INDEX of the Node with minimal distance"""

        dlist = [(node.row - rnd_node.row) ** 2 + (node.col - rnd_node.col)  ** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind


    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.col - from_node.col
        dy = to_node.row - from_node.row
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


    def steer(self, from_node, to_node, extend_length=float("inf")):

        """If the node is further than the 'extend_length', bring it closer, but in same direction"""

        new_node = self.Node(to_node.x, to_node.y)
        new_node.parent = from_node
        d, theta = self.calc_distance_and_angle(from_node, to_node)

        if d < extend_length:
            # no need to steer
            new_node.parent = from_node
            return new_node

        else: 
            # steer to a length of the increment
            new_node.x = from_node.x + extend_length * math.cos(theta)
            new_node.y = from_node.y + extend_length * math.sin(theta)

            return new_node

    def find_near_nodes(self, new_node):
        """Find nearby nodes in a certain radius - return INDIXES of those nodes"""

        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))  # check this value
        dist_list = [(node.col - new_node.col) ** 2 + (node.row - new_node.row) ** 2 for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
        return near_inds


    def choose_parent(self, new_node, near_inds):
        """From all the nodes nearby, choose the node, upon connecting to which, the cost will be MININAL"""
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]

            if self.check_line_collision(near_node, new_node):

                # t_node = self.steer(near_node, new_node)
                # if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node

        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        # new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        return new_node        

    def calc_new_cost(self, from_node, to_node):
        """Calculate the cost of the to_node, if it connects to 'from_node'"""
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d


    def rewire(self, new_node, near_inds):
        """Recalculate whether connecting nearby nodes to the 'new_node' will improve their cost"""

        for i in near_inds:
            near_node = self.node_list[i]

            no_collision = self.check_line_collision(near_node, new_node)

            if no_collision:
                new_cost = self.calc_new_cost(from_node = new_node, to_node = near_node)
                improved_cost = near_node.cost > new_cost

                if improved_cost:
                    near_node.parent = new_node
                    self.propagate_cost_to_leaves(new_node)        


    def propagate_cost_to_leaves(self, parent_node):
        """after updating the cost of a node, update cost of all the nodes depending on this node"""
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)               

    def find_route(self):
        """Find a route starting from the closest node to the targer"""  
        pass

    def optimize_path(self):
        """Optimize path upon the visibility constraint. For every node in the path,
        if the path is obstacle free, connect the nodes directly"""

        pass         