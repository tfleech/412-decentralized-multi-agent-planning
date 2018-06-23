#!/usr/bin/env python

import sys
import collections
import rospy
import numpy as np
from Config import Config
from std_msgs.msg import Int8, Bool, UInt16
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, Point32
from random import randint
from rvo_util import *


class RVO(object):
    def __init__(self):
        self.is_init = True
        self.agent_name  = rospy.get_namespace()[1:-1]
        self.agent_index = int(rospy.get_namespace()[-2]) - 1 # Index of this agent
        print '[ INFO::{} ] agent_index: {}'.format(self.agent_name, self.agent_index)

        self._init_rvo_settings()
        self._init_subs()
        self._init_pubs()

    def _init_subs(self):
        topic_name = "loc"
        self.loc_sub = rospy.Subscriber(topic_name, Point32, self.loc_cb, queue_size=1)
        print '[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name)

        topic_name = "intermediate_waypoint"
        self.target_sub = rospy.Subscriber(topic_name, PoseStamped, self.target_cb, queue_size=1)
        print '[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name)

        topic_name = "/n_agts"
        self.all_vels_desired_sub = rospy.Subscriber(topic_name, UInt16, self.n_agts_cb, queue_size=1)
        print '[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name)

    def _init_pubs(self):
        topic_name = "vel"
        self.vel_pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
        print '[ INFO::{} ] Publishing to {}'.format(self.agent_name, topic_name)

        #topic_name = "cmd_vel"
        #self.cmd_vel_pub = rospy.Publisher(topic_name, Twist, queue_size=1)
        #print '[ INFO::{} ] Publishing to {}'.format(self.agent_name, topic_name)

        topic_name = "vel_desired"
        self.vel_desired_pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
        print '[ INFO::{} ] Publishing to {}'.format(self.agent_name, topic_name)

        topic_name = "waypoint"
        self.waypoint_pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
        print '[ INFO::{} ] Publishing to {}'.format(self.agent_name, topic_name)

    def _init_rvo_settings(self):
        print "[ INFO::{} ] Initialize rvo settings ...".format(self.agent_name)
        self.config = Config(summary=True) # Set flag False to disable summary print
        self.loc              = None
        self.goal             = None
        self.vel              = [0., 0.]
        self.vel_desired      = None
        self.all_locs         = None
        self.all_vels         = None
        self.all_vels_desired = None
        self.step             = 0.2 # Unit in second

    def n_agts_cb(self, msg):
        if self.is_init:
            n_agt = int(msg.data)
            self.all_locs         = [None for i_agt in range(n_agt)]
            self.all_vels         = [None for i_agt in range(n_agt)]
            self.all_vels_desired = [None for i_agt in range(n_agt)]
            self.is_init = False

            # Create subscriber for other agents (excluding myself)
            for i_agt in range(1, n_agt + 1):
                if i_agt != self.agent_index + 1:
                    topic_name = "/agent_" + str(i_agt) + "/loc"
                    rospy.Subscriber(topic_name, Point32, self.other_agt_loc_cb, (i_agt))
                    #print '[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name)

                    topic_name = "/agent_" + str(i_agt) + "/vel"
                    rospy.Subscriber(topic_name, PoseStamped, self.other_agt_vel_cb, (i_agt))
                    #print '[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name)

                    topic_name = "/agent_" + str(i_agt) + "/vel_desired"
                    rospy.Subscriber(topic_name, PoseStamped, self.other_agt_vel_desired_cb, (i_agt))
                    #print '[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name)

    def other_agt_loc_cb(self, msg, i_agt):
        i_agt = i_agt - 1 # Convert to index which starts from 0
        self.all_locs[i_agt] = [msg.x, msg.y]
        #print '[ INFO::{} ] Received loc info {} for agent {}'.format(self.agent_name, self.all_locs[i_agt], i_agt + 1)

    def other_agt_vel_cb(self, msg, i_agt):
        i_agt = i_agt - 1 # Convert to index which starts from 0
        self.all_vels[i_agt] = [msg.pose.position.x, msg.pose.position.y]
        #print '[ INFO::{} ] Received vel info {} for agent {}'.format(self.agent_name, self.all_vels[i_agt], i_agt + 1)

    def other_agt_vel_desired_cb(self, msg, i_agt):
        i_agt = i_agt - 1 # Convert to index which starts from 0
        self.all_vels_desired[i_agt] = [msg.pose.position.x, msg.pose.position.y]
        #print '[ INFO::{} ] Received vel_desired info for agent {}'.format(self.agent_name, i_agt + 1)

    def loc_cb(self, msg):
        self.loc = [msg.x, msg.y]
        if self.all_locs is not None:
            self.all_locs[self.agent_index] = self.loc

    def target_cb(self, msg):
        self.goal = [msg.pose.position.x,msg.pose.position.y]

    def compute_vel_desired(self):
        if self.loc is None or self.goal is None:
            #print("something is none")
            pass
        else:
            self.vel_desired = compute_vel_desired(self.loc, self.goal, self.config.vel_max, self.config.goal_th)

    def pub_vel(self):
        if self.vel is None:
            pass
        else:
            # Using the position type to encode the velocities
            # NOTE that other information are dummy
            vel_msg = PoseStamped()
            vel_msg.header.stamp = rospy.Time.now()
            vel_msg.pose.position.x = self.vel[0] # In speed
            vel_msg.pose.position.y = self.vel[1] # In speed
            vel_msg.pose.position.z = 0
            vel_msg.pose.orientation.x = 0
            vel_msg.pose.orientation.y = 0
            vel_msg.pose.orientation.z = 0
            vel_msg.pose.orientation.w = 0

            self.vel_pub.publish(vel_msg)
            if self.all_vels is not None:
                self.all_vels[self.agent_index] = self.vel
            #print "[ INFO::{} ] Published vel {}".format(self.agent_name, self.vel)

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.vel[0] # In speed
            cmd_vel_msg.linear.y = self.vel[1] # In speed
            cmd_vel_msg.linear.z = 0

            #self.cmd_vel_pub.publish(cmd_vel_msg)

            # Also publish waypoint
            # NOTE used for underactuated
            if self.loc is not None:
                wpt_msg = PoseStamped()
                wpt_msg.header.stamp = rospy.Time.now()
                wpt_msg.header.frame_id = "map"
                wpt_msg.pose.position.x = self.loc[0] + self.step*self.vel[0]
                wpt_msg.pose.position.y = self.loc[1] + self.step*self.vel[1]
                wpt_msg.pose.position.z = 0
                wpt_msg.pose.orientation.x = 0
                wpt_msg.pose.orientation.y = 0
                wpt_msg.pose.orientation.z = 0
                wpt_msg.pose.orientation.w = 1

                self.waypoint_pub.publish(wpt_msg)

    def pub_vel_desired(self):
        if self.vel_desired is None:
            pass
        else:
            # Using the position type to encode the velocities
            # NOTE that other information are dummy
            vel_desired_msg = PoseStamped()
            vel_desired_msg.header.stamp = rospy.Time.now()
            vel_desired_msg.pose.position.x = self.vel_desired[0] # In speed
            vel_desired_msg.pose.position.y = self.vel_desired[1] # In speed
            vel_desired_msg.pose.position.z = 0
            vel_desired_msg.pose.orientation.x = 0
            vel_desired_msg.pose.orientation.y = 0
            vel_desired_msg.pose.orientation.z = 0
            vel_desired_msg.pose.orientation.w = 0

            self.vel_desired_pub.publish(vel_desired_msg)
            if self.all_vels_desired is not None:
                self.all_vels_desired[self.agent_index] = self.vel_desired
            #print "[ INFO::{} ] Published vel_desired: {}".format(self.agent_name, self.vel_desired)

    def check_data(self):
        if self.all_locs is None or self.all_vels is None or self.all_vels_desired is None:
            return True
        elif self.loc is None or \
           self.vel is None or \
           self.vel_desired is None or \
           self.goal is None or \
           None in self.all_locs or \
           None in self.all_vels or \
           None is self.all_vels_desired:
            return True
        else:
            return False

    def compute_vel(self):
        if self.check_data():
            pass
        else:
            #------------------------------
            # With the translation_func defined above,
            # Compute all velocity obstacles for all robots
            #------------------------------
            vel_obstacles = compute_vel_obstacles(self.all_locs, self.all_vels_desired, self.all_vels, 
                                                  self.config.robot_radius, translation_func=RVO_translation)
    
            #------------------------------
            # With the computed velocity obstacles
            # update velocities such that robots paths are collision-free
            #------------------------------
            self.vel = vel_update(self.loc, self.vel_desired, vel_obstacles[self.agent_index])
            #print "[ INFO::{} ] Updated vel".format(self.agent_name)
    
            # #------------------------------
            # # Update position with updated velocities
            # #------------------------------
            # for i_robot in range(len(config.locs)):
            #     self.config.locs[i_robot][0] += config.vels[i_robot][0]*config.step
            #     self.config.locs[i_robot][1] += config.vels[i_robot][1]*config.step

            # #------------------------------
            # # If all robots reached goal, then break the loop
            # #------------------------------
            # reached_goal = is_reached_goal(config.locs, config.goals, config)
            # step_counter += 1
        

if __name__ == '__main__':
    rospy.init_node('rvo', anonymous=True)
    rvo = RVO()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # RVO::Publish current vel 
        rvo.pub_vel()

        # RVO::Compute desired vel and publish it
        rvo.compute_vel_desired()
        rvo.pub_vel_desired()
   
        # RVO::Compute vel based on RVO
        rvo.compute_vel()

        rate.sleep()
   
