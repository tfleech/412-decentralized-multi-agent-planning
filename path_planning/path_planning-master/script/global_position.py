#!/usr/bin/env python

from threading import Lock
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Point32,PoseStamped, PointStamped
from std_msgs.msg import UInt16


class Global_Position(object):
    def __init__(self):

        self.agent_index = int(rospy.get_namespace()[-2]) - 1 # Index of this agent

        # Subscriber
        topic_name = "odom"
        self.odom_sub = rospy.Subscriber(topic_name, Odometry,
            self.odom_cb, queue_size=1)
        print '[ INFO ] Subscribe to {}'.format(topic_name)

        # Publisher
        topic_name = "loc"
        self.loc_pub = rospy.Publisher(topic_name, Point32, queue_size=1)
        print '[ INFO ] Publishing to {}'.format(topic_name)

        topic_name = "/agent_loc"
        self.stamped_loc_pub = rospy.Publisher(topic_name, PointStamped, queue_size=1)
        print '[ INFO ] Publishing to {}'.format(topic_name)

        topic_name = "/n_agts"
        self.n_agts_pub = rospy.Publisher(topic_name, UInt16, queue_size=1)
        print '[ INFO ] Publishing to {}'.format(topic_name)

        self.odom_msg = Odometry()

    def odom_cb(self, msg):
        self.odom_msg = msg


if __name__ == '__main__':
    rospy.init_node('global_pos_node', anonymous=True)
    global_position = Global_Position()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():

        loc_msg = Point32()
        loc_msg.x = global_position.odom_msg.pose.pose.position.x
        loc_msg.y = global_position.odom_msg.pose.pose.position.y
        loc_msg.z = 0

        stamped_loc_msg = PointStamped()
        stamped_loc_msg.header.frame_id = str(global_position.agent_index)
        stamped_loc_msg.point.x = global_position.odom_msg.pose.pose.position.x
        stamped_loc_msg.point.y = global_position.odom_msg.pose.pose.position.y
        stamped_loc_msg.point.z = 0

        global_position.loc_pub.publish(loc_msg)
        global_position.stamped_loc_pub.publish(stamped_loc_msg)

        global_position.n_agts_pub.publish(3)

        rate.sleep()

