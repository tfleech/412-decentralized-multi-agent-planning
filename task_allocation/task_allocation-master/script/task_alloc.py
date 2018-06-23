#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import UInt16, Bool
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped, PoseArray, Point32, PointStamped

from random import randint


class Task_Allocation(object):
    def __init__(self):

        self.targets = {}
        self.agents = {}
        self.n_agents = 0
        self.fresh_targets = False
        self.valid_target_request = True
        self.A={} #global bidspace, one win bid is i:[(j,reward_value),() ... ]
        self.reward = 1000

        self.agent_name  = rospy.get_namespace()[1:-1]
        self.agent_index = int(rospy.get_namespace()[-2]) - 1 # Index of this agent
        print '[ INFO::{} ] agent_index: {}'.format(self.agent_name, self.agent_index)

        # Subscriber
        topic_name = "/possible_points"
        self.all_target_bids_sub = rospy.Subscriber(topic_name, PointCloud,self.all_targets_cb, queue_size=1)
        print '[ INFO ] Subscribe to {}'.format(topic_name)

        topic_name = "/n_agts"
        self.n_agents_sub = rospy.Subscriber(topic_name, UInt16, self.n_agents_cb, queue_size=1)
        print '[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name)

        topic_name = "/agent_loc"
        self.agent_loc_sub = rospy.Subscriber(topic_name, PointStamped, self.agent_loc_cb, queue_size=1)
        print '[ INFO::{} ] Subscribe to {}'.format(self.agent_name, topic_name)

        topic_name = "new_target"
        self.new_target_sub = rospy.Subscriber(topic_name, Bool,self.new_target_cb, queue_size=1)
        print '[ INFO ] Subscribe to {}'.format(topic_name)

        # Publisher
        topic_name = "target"
        self.target_pub = rospy.Publisher(topic_name, Point32, queue_size=1)
        print '[ INFO ] Publishing to {}'.format(topic_name)

        topic_name = "viz_target"
        self.viz_target_pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
        print '[ INFO ] Publishing to {}'.format(topic_name)

        topic_name = "/semaphore"
        self.new_targets_pub = rospy.Publisher(topic_name, Bool, queue_size=1)
        print '[ INFO ] Publishing to {}'.format(topic_name)

    ### Callbacks
    def new_target_cb(self, msg):
        if(msg.data and self.A and self.valid_target_request):
            self.A[self.agent_index].pop(0)
            self.valid_target_request = False
        if(not msg.data):
            self.valid_target_request = True

    def n_agents_cb(self, msg):
        self.n_agents = msg

    def agent_loc_cb(self, msg):
        self.agents[int(msg.header.frame_id)] = (msg.point.x,msg.point.y)

    def all_targets_cb(self, msg):
        for i in range(0,len(msg.points)):
            self.targets[i] = [msg.channels[i].values[0],(msg.points[i].x,msg.points[i].y)]
        self.fresh_targets = True
    ###

    def distance(self,p1,p2):
        #takes in coords, want to do better than Euclidian....
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

    def reward_value(self,agent, target):
        #Reward-accumulative_distance takes into account importance value for targets
        #should be replaced by D*
        i=agent
        j=target
        agent_coord=self.agents[i]
        target_coord=self.targets[j][1]

        #empty A, then is the current position to target j
        if not self.A[i]:
            accum_dist = self.distance(agent_coord, target_coord)

        else:
            accum_dist=0
            current_allocation=self.A[i] #list of touples (j,reward_value)
            if len(current_allocation)>1:

                for index in range(len(current_allocation)-1):
                    j=current_allocation[index][0]  #index of target
                    next=current_allocation[index+1][0]
                    j_coord=self.targets[j][1]
                    next_coord=self.targets[next][1]
                    weight=self.targets[index+1][0] #how important is it to go to this next_target
                    accum_dist+=self.distance(j_coord, next_coord)*(1-weight) #adding distance between all allocated targets to the accumulative distance
                accum_dist+=self.distance(self.targets[current_allocation[-1][0]][1], target_coord)*(1-self.targets[j][0]) #adding the final term from last allocated target, to the target of interest, j
            else:
                weight=self.targets[current_allocation[0][0]][0]
                accum_dist+=self.distance(agent_coord,self.targets[current_allocation[0][0]][1] )*(1-weight)
                weight=self.targets[j][0]
                accum_dist+=self.distance(agent_coord,self.targets[j][1])*(1-weight)
        return self.reward-accum_dist


    def update(self):

        for agent in self.agents:
            self.A[agent]=[]

        count=0
        while count<10:  #bidding process, wind bids on the fly
            count+=1
            for i in self.agents:
                for j in self.targets:
                    win=True
                    for agent in self.A:
                        if self.A[agent]:
                            for target in self.A[agent]: #target is a tuple (j, reward_value)
                                if target[0]==j:

                                    if self.reward_value(i,j)<=target[1]:
                                        win=False

                    if win:

                        for agent in self.A:
                            if self.A[agent]:
                                for target in self.A[agent]: #target is a tuple (j, reward_value)
                                    if target[0]==j:
                                        self.A[agent].remove(target)
                        self.A[i].append((j, self.reward_value(i,j)))



if __name__ == '__main__':
    rospy.init_node('task_allocation', anonymous=True)
    task_allocation = Task_Allocation()

    rate = rospy.Rate(1)
    task_allocation.new_targets_pub.publish(True)
    while not rospy.is_shutdown():

        if(task_allocation.fresh_targets):
            task_allocation.new_targets_pub.publish(False)
            task_allocation.fresh_targets = False
            task_allocation.update()

        #print("Current allocation: ",task_allocation.A)
        if(task_allocation.A):
            try:
                target_index = task_allocation.A[task_allocation.agent_index][0][0]

                target_msg = Point32()
                target_msg.x = task_allocation.targets[target_index][1][0]
                target_msg.y = task_allocation.targets[target_index][1][1]
                target_msg.z = 0
                task_allocation.target_pub.publish(target_msg)

                viz_target_msg = PoseStamped()
                viz_target_msg.header.frame_id = "map"
                viz_target_msg.pose.position.x = task_allocation.targets[target_index][1][0]
                viz_target_msg.pose.position.y = task_allocation.targets[target_index][1][1]
                viz_target_msg.pose.position.z = 0
                task_allocation.viz_target_pub.publish(viz_target_msg)
            except IndexError:
                rospy.loginfo("No target for agent " + str(task_allocation.agent_index))
                task_allocation.new_targets_pub.publish(True)
        rate.sleep()

