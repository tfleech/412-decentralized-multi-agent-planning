#!/usr/bin/env python

from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
from threading import Lock
import rospy
from geometry_msgs.msg import Point32,Twist,PoseStamped
from tf.transformations import euler_from_quaternion

class GotoPoint(object):
    def __init__(self):

         # Subscriber
        topic_name = "waypoint"
        self.model_states_sub = rospy.Subscriber(topic_name, PoseStamped,
            self.goal_cb, queue_size=1)
        print '[ INFO ] Subscribe to {}'.format(topic_name)

        topic_name = "pose"
        self.model_states_sub = rospy.Subscriber(topic_name, PoseStamped,
            self.current_pose_cb, queue_size=1)
        print '[ INFO ] Subscribe to {}'.format(topic_name)

        # Publisher
        topic_name = "cmd_vel"
        self.cmd_vel = rospy.Publisher(topic_name, Twist, queue_size=1)
        print '[ INFO ] Publishing to {}'.format(topic_name)

        self.goal = PoseStamped()
        self.current_pose = PoseStamped()
        self.rate = rospy.Rate(10)
        self.goal_received = False

    def current_pose_cb(self, msg):
        self.current_pose = msg

    def goal_cb(self, msg):
        self.goal_received = True
        self.goal = msg

    def gen_vel(self,position,rotation,goal_x,goal_y,goal_z):
        move_cmd = Twist()
        last_rotation = 0
        linear_speed = .9
        angular_speed = 1
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance
        # print("position is:", position)
        # print("rotation is:", rotation)
        # print("goal_x is:", goal_x)
        # print("goal_y is:", goal_y)
        # print("goal_z is:", goal_z)

        # while distance > 0.05:
        # (position, rotation) = self.get_odom()
        x_start = position.x
        y_start = position.y
        path_angle = atan2(goal_y - y_start, goal_x- x_start)

        if path_angle < -pi/4 or path_angle > pi/4:
            if goal_y < 0 and y_start < goal_y:
                path_angle = -2*pi + path_angle
            elif goal_y >= 0 and y_start > goal_y:
                path_angle = 2*pi + path_angle
        if last_rotation > pi-0.1 and rotation <= 0:
            rotation = 2*pi + rotation
        elif last_rotation < -pi+0.1 and rotation > 0:
            rotation = -2*pi + rotation
        move_cmd.angular.z = angular_speed * path_angle-rotation

        distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
        # print("distance is", distance)
        move_cmd.linear.x = min(linear_speed * distance, 0.5) # default 0.1

        if move_cmd.angular.z > 0:
            move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
        else:
            move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

        last_rotation = rotation
        self.cmd_vel.publish(move_cmd)
        # self.rate.sleep()

        # while abs(rotation - goal_z) > 0.05:
        #     if goal_z >= 0:
        #         if rotation <= goal_z and rotation >= goal_z - pi:
        #             move_cmd.linear.x = 0.00
        #             move_cmd.angular.z = 0.5
        #         else:
        #             move_cmd.linear.x = 0.00
        #             move_cmd.angular.z = -0.5
        #     else:
        #         if rotation <= goal_z + pi and rotation > goal_z:
        #             move_cmd.linear.x = 0.00
        #             move_cmd.angular.z = -0.5
        #         else:
        #             move_cmd.linear.x = 0.00
        #             move_cmd.angular.z = 0.5
        #     self.cmd_vel.publish(move_cmd)
        #     self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('go_to_point', anonymous=True)
    goto = GotoPoint()

    while not rospy.is_shutdown():

        position = Point32()
        position.x = goto.current_pose.pose.position.x
        position.y = goto.current_pose.pose.position.y
        position.z = goto.current_pose.pose.position.z

        orientation = [goto.current_pose.pose.orientation.x, \
                        goto.current_pose.pose.orientation.y, \
                        goto.current_pose.pose.orientation.z, \
                        goto.current_pose.pose.orientation.w]
        [roll, pitch, yaw] = euler_from_quaternion(orientation)

        if goto.goal_received:
            goto.gen_vel(position, yaw, goto.goal.pose.position.x, goto.goal.pose.position.y, goto.goal.pose.position.z)
        # goto.cmd_vel.publish(Twist())

        goto.rate.sleep()
