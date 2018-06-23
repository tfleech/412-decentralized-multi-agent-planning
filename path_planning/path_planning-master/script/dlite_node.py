#!/usr/bin/env python

import sys
import collections
import rospy
import numpy as np
from Config import Config
from std_msgs.msg import Int8, Bool, UInt16
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path 
from geometry_msgs.msg import PoseStamped, Twist, Point32 
from random import randint
sys.path.insert(0, 'dlite_scripts/map-and-features/')
sys.path.insert(0, 'dlite_scripts/')
from dlite_scripts.dstar import DStar_planner
from dlite_scripts.map_and_features.maII_map import MAII_map
from dlite_scripts.KeyedVertex import KeyedVertex
from math import sqrt,pow

class PLANNER(object):
    def __init__(self):
        self.is_init = True
        self.agent_name  = rospy.get_namespace()[1:-1] 
        self.agent_index = int(rospy.get_namespace()[-2]) - 1 # Index of this agent
        print('[ INFO::{} ] agent_index: {}'.format(self.agent_name, self.agent_index))
        self.map = None

        self._init_planner_settings()
        self._init_subs()
        self._init_pubs()
        self.look_ahead = 2

    def _init_subs(self):
        print("Initializing subs...")

        topic_name = "pose"
        self.loc_sub = rospy.Subscriber(topic_name, PoseStamped, self.loc_cb, queue_size=1)
        print('[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name))

        topic_name = "target"
        self.target_sub = rospy.Subscriber(topic_name, Point32, self.target_cb, queue_size=1)
        print('[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name))

        topic_name = "/map"
        self.occ_grid_sub = rospy.Subscriber(topic_name, OccupancyGrid, self.grid_cb, queue_size=1)
        print('[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name))


    def _init_pubs(self):
        topic_name = "intermediate_waypoint"
        self.cmd_loc_goal_pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
        print('[ INFO::{} ] Publishing to {}'.format(self.agent_name, topic_name))

        topic_name = "new_target"
        self.targ_req_pub = rospy.Publisher(topic_name, Bool, queue_size=1)
        print('[ INFO::{} ] Publishing to {}'.format(self.agent_name, topic_name))

        topic_name = "heartbeat"
        self.heartbeat_pub = rospy.Publisher(topic_name, Bool, queue_size=1)
        print('[ INFO::{} ] Publishing to {}'.format(self.agent_name, topic_name))

    def _init_planner_settings(self):
        self.config = Config(summary=True) # Set flag False to disable summary print  
        self.X_SCALE = 1
        self.Y_SCALE = 1

        self.loc              = None
        self.goal             = None
        self.dlite_planner    = None
        self.local_goal       = None
        self.path             = []


    # To be called when info available to ingest
    def loc_cb(self, msg):
        self.loc = (int(round(msg.pose.position.x,2)*100), int(round(msg.pose.position.y,2)*100))

        # Ensure Planner has started here, if possible
        if((self.map is not None) and (self.goal is not None) and (self.dlite_planner is None)):
            self.dlite_planner = DStar_planner(self.loc,self.goal,self.map,1.5) 
            self.path = []
            self.local_goal = None
            print("initializing planner from location receiver")
        elif(self.dlite_planner is not None): # if all is good, can just update current pos
            self.dlite_planner.updateStart(self.loc)

    # Do primary pathfinding calculation in our main loop at our own discresion
    
    # To be called when info available to ingest
    def target_cb(self, msg):
        self.goal = (int(round(msg.x,2)*100), int(round(msg.y,2)*100))

        # Ensure Planner has started here, if possible
        if((self.map is not None) and (self.loc is not None) and (self.dlite_planner is None)):
            self.dlite_planner = DStar_planner(self.loc,self.goal,self.map,1.5) 
            self.path = []
            self.local_goal = None
        elif(self.dlite_planner is not None):
            if(msg != self.loc):
                self.dlite_planner = DStar_planner(self.loc,self.goal,self.map,1.5) # There is a less expensive version that doesn't completely re-init i will test next
                self.path = []
                self.local_goal = None
            # alternative, but needs testing: 
            # self.dlite_planner.updateGoal(self.goal) # cur location has to be set before doing this, as well (used w/in)

        # Do primary pathfinding calculation in our main loop at our own discresion

    def grid_cb(self, msg): #msg = grid, i think    
        ######## CONVERSION/INITIALIZATION OF MAP HERE ##########
        grid = msg # just to maintain similarity of names
        res = int(grid.info.resolution)
        width = int(grid.info.width)
        height = int(grid.info.height)
        # assuming the same origin for the moment
        origin_x = int(grid.info.origin.position.x*100)
        origin_y = int(grid.info.origin.position.y*100)
        data = grid.data  #temporary for test

        if(self.map is None):
            self.map = MAII_map.init_from_gMap(data, width, height)
        else:
            self.map.update_gMap(data, width, height)
        ######## END CONVERSION/INITIALIZATION OF MAP HERE ##########


        #After processing map, update path if all other pieces exist
        if((self.goal is not None) and (self.loc is not None) and (self.dlite_planner is None)):
            print("initializing planner from grid receiver")
            self.dlite_planner = DStar_planner(self.loc,self.goal,self.map,1.5) 
            self.path = []
            self.local_goal = None

    def pub_local_goal(self): #pub_vel
        if(self.local_goal is not None):
            loc_goal = PoseStamped();
            loc_goal.header.frame_id = "map"
            loc_goal.pose.position.x = self.local_goal[0]*self.X_SCALE;
            loc_goal.pose.position.y = self.local_goal[1]*self.Y_SCALE;
            loc_goal.pose.position.z = 0;
            self.cmd_loc_goal_pub.publish(loc_goal) 
            # print("Go to: {}".format(self.local_goal))
        elif(self.loc is not None):  
            dont_move = PoseStamped()
            dont_move.header.frame_id = "map"
            dont_move.pose.position.x = self.loc[0]
            dont_move.pose.position.y = self.loc[1]
            dont_move.pose.position.z = 0
            self.cmd_loc_goal_pub.publish(dont_move)    
            # print("Stay:  {}".format(self.loc))

    def pub_request_for_targets(self, tof): #pub_vel
        self.targ_req_pub.publish(tof)  

    def update_path(self):
        if(self.dlite_planner is not None): 
            # _,self.path = self.dlite_planner.checkAndUpdate(self.path,0) # Check for obstructions from start of path. Could ignore part of path we've passed, but since we're not assuming perfect movement, we'd need to do something tricky like calculate where the closest place on the path is to us.
            self.path = self.dlite_planner.computeShortestPath() # Check for obstructions from start of path. Could ignore part of path we've passed, but since we're not assuming perfect movement, we'd need to do something tricky like calculate where the closest place on the path is to us.
            if(not self.path):
                return False
            elif (len(self.path)>self.look_ahead):
                self.local_goal = self.path[self.look_ahead];
            elif(len(self.path)>0):
                self.local_goal = self.path[-1];
            else:
                print("No known path for robot {}.".format(self.agent_index))

        return True;

    def dist_to_target(self):
        try:
            return sqrt(pow((self.loc[0]-self.goal[0]),2)+pow((self.loc[1]-self.goal[1]),2))
        except TypeError:
            return 0
 

if __name__ == '__main__':
    rospy.init_node('d_star_planner', anonymous=True)
    planner = PLANNER()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # # Planner: Publish current goal or request new one
        pBool = planner.update_path()
        if(pBool and (planner.dist_to_target() > 1)):
            planner.pub_request_for_targets(False);
            planner.pub_local_goal()
        else:
            if(not pBool):
                print("Target @ {} unreachable. Request new target.".format(planner.goal))
            planner.pub_request_for_targets(True);

        # try:
        #     if(planner.map.checkObs(planner.loc)):
        #         print("Collision @ {}.".format(planner.loc))
        #         #TODO: publish something here to stop actuation?
        # except AttributeError:
        #     rospy.loginfo("D_star map is empty")

        rate.sleep()

