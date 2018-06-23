#!/usr/bin/env python

import random
from threading import Lock
import rospy
from sensor_msgs.msg import PointCloud,ChannelFloat32
from geometry_msgs.msg import Point32


if __name__ == '__main__':
    rospy.init_node('test_targets_node', anonymous=True)

    topic_name = "/possible_points"
    targets_pub = rospy.Publisher(topic_name, PointCloud, queue_size=1)
    print '[ INFO ] Publishing to {}'.format(topic_name)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        target_msg = PointCloud()
        for i in range (0,20):
            point = Point32()
            point.x = random.uniform(-10,10)
            point.y = random.uniform(-10,10)
            point.z = 0
            point_reward = ChannelFloat32()
            point_reward.values = [random.uniform(0,1)]
            target_msg.points.append(point)
            target_msg.channels.append(point_reward)

        targets_pub.publish(target_msg)
        rate.sleep()
