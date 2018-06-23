#!/usr/bin/env python
import sys
import collections
import rospy
import numpy as np
import matplotlib.pyplot as plt
from Config import Config
from std_msgs.msg import Int8, Bool, UInt16
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Point32
from random import randint
from rvo_util import *


class RVO_Test(object):
    def __init__(self):
        self.n_agts = 3

        self._init_settings()
        self._init_subs()
        self._init_pubs()

    def _init_settings(self):
        self.locs    = [[1., 0.], [-2., 0.], [-3., -2.]]
        self.vels    = [[0., 0.], [0., 0.], [0., 0.]]
        self.targets = [[5., 5.], [-5., 0.], [3., -5.]]
        self.step    = 0.01
        self.log     = [[[1.], [0.]], [[-2.], [0.]], [[-3.], [-2.]]]

    def _init_subs(self):
        for i_agt in range(1, self.n_agts + 1):
            # For vel sub
            topic_name = "/agent_" + str(i_agt) + "/vel"
            rospy.Subscriber(topic_name, PoseStamped, self.vel_cb, (i_agt))
            print '[ INFO ] Subscribe to {}'.format(topic_name)

    def _init_pubs(self):
        self.locs_pub    = []
        self.targets_pub = []
        for i_agt in range(1, self.n_agts + 1):
            # For loc pub
            topic_name = "/agent_" + str(i_agt) + "/loc"
            self.locs_pub.append(rospy.Publisher(topic_name, Point32, queue_size=1))
            print '[ INFO ] Publishing to {}'.format(topic_name)

            # For target pub
            topic_name = "/agent_" + str(i_agt) + "/intermediate_goal"
            self.targets_pub.append(rospy.Publisher(topic_name, Point32, queue_size=1))
            print '[ INFO ] Publishing to {}'.format(topic_name)

            if i_agt == 1:
                topic_name = "/n_agts"
                self.n_agt_pub = rospy.Publisher(topic_name, UInt16, queue_size=1)
                print '[ INFO ] Publishing to {}'.format(topic_name)

    def vel_cb(self, msg, i_agt):
        i_agt = i_agt - 1 # Convert to index which starts from 0
        self.vels[i_agt] = [msg.pose.position.x, msg.pose.position.y]
        print '[ INFO ] Received vel info {} for agent {}'.format(self.vels[i_agt], i_agt + 1)

        self.locs[i_agt][0] += self.vels[i_agt][0]*self.step
        self.locs[i_agt][1] += self.vels[i_agt][1]*self.step

        self.log[i_agt][0].append(self.locs[i_agt][0])
        self.log[i_agt][1].append(self.locs[i_agt][1])

    def publish_msgs(self):
        for i_agt in range(self.n_agts):
            # n_agt pub
            self.n_agt_pub.publish(self.n_agts)

            # Loc pub
            loc_msg = Point32()
            loc_msg.x = self.locs[i_agt][0]
            loc_msg.y = self.locs[i_agt][1]
            self.locs_pub[i_agt].publish(loc_msg)

            # Target pub
            target_msg = Point32()
            target_msg.x = self.targets[i_agt][0]
            target_msg.y = self.targets[i_agt][1]
            self.targets_pub[i_agt].publish(target_msg)

if __name__ == '__main__':
    rospy.init_node('rvo_test', anonymous=True)
    test = RVO_Test()

    rate = rospy.Rate(10)
    counter = 0
    while not rospy.is_shutdown():
        test.publish_msgs()
        counter += 1
        if counter % 5000 == 0:
            plt.scatter(test.log[0][0], test.log[0][1], c='r')
            plt.scatter(test.log[1][0], test.log[1][1], c='g')
            plt.scatter(test.log[2][0], test.log[2][1], c='b')
            plt.show()
