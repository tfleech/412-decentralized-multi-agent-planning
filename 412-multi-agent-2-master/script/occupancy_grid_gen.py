#!/usr/bin/env python

from threading import Lock
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose,Point32,PoseStamped
from gazebo_msgs.msg import ModelStates
from copy import deepcopy

class Occupancy_Grid(object):
    def __init__(self):

         # Subscriber
        topic_name = "/gazebo/model_states"
        self.model_states_sub = rospy.Subscriber(topic_name, ModelStates,
            self.model_states_cb, queue_size=1)
        print '[ INFO ] Subscribe to {}'.format(topic_name)

        # Publisher
        topic_name = "/gazebo_occupancy_grid"
        self.grid_pub = rospy.Publisher(topic_name, OccupancyGrid, queue_size=1)
        print '[ INFO ] Publishing to {}'.format(topic_name)


        self.model_states_mutex = Lock()
        self.model_states = ModelStates()
        self.unmodified_model_states = ModelStates()

    def model_states_cb(self, msg):
        self.model_states_mutex.acquire()
        self.unmodified_model_states = deepcopy(msg)
        '''
        self.model_states = msg
        for index, model in enumerate(msg.pose):
            #round to nearest grid cell
            self.model_states.pose[index].position.x = int(round(msg.pose[index].position.x,0))
            self.model_states.pose[index].position.y = int(round(msg.pose[index].position.y,0))
        '''
        self.model_states_mutex.release()




if __name__ == '__main__':
    rospy.init_node('occupancy_grid', anonymous=True)
    occupancy_grid = Occupancy_Grid()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        '''
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()

        grid_msg.info.map_load_time = rospy.Time.now()
        grid_msg.info.resolution = 1 # m/cell
        grid_msg.info.width = 20 # cells
        grid_msg.info.height = 20 # cells
        grid_pose = Pose()
        grid_pose.position.x = - grid_msg.info.width/2
        grid_pose.position.y = - grid_msg.info.height/2
        grid_pose.position.z = 0
        grid_msg.info.origin = grid_pose

        occupancy_grid.model_states_mutex.acquire()
        for y in range(-grid_msg.info.width/2,grid_msg.info.width/2):
            for x in range(-grid_msg.info.height/2,grid_msg.info.height/2):
                obstacle_found = False
                for model in occupancy_grid.model_states.pose[1:]:
                    if(x == model.position.x and y == model.position.y):
                        #print("found obstacle at :",x,y)
                        grid_msg.data.append(100)
                        obstacle_found = True
                        break
                if(not obstacle_found):
                        grid_msg.data.append(0)
        occupancy_grid.model_states_mutex.release()

        occupancy_grid.grid_pub.publish(grid_msg)
        '''
        occupancy_grid.model_states_mutex.acquire()
        for index,agent in enumerate(occupancy_grid.unmodified_model_states.pose):
            agent_topic = "/"+ occupancy_grid.unmodified_model_states.name[index] + "/loc"
            pose_topic = "/"+ occupancy_grid.unmodified_model_states.name[index] + "/pose"
            location_pub = rospy.Publisher(agent_topic, Point32, queue_size=1)
            pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
 
            agent_location = Point32()
            agent_location.x = occupancy_grid.unmodified_model_states.pose[index].position.x
            agent_location.y = occupancy_grid.unmodified_model_states.pose[index].position.y
            agent_location.z = occupancy_grid.unmodified_model_states.pose[index].position.z
            location_pub.publish(agent_location)

            agent_pose = PoseStamped()
            agent_pose.header.stamp = rospy.Time.now()
            agent_pose.header.frame_id = "map"
            agent_pose.pose = agent
            pose_pub.publish(agent_pose)
        occupancy_grid.model_states_mutex.release()
    rate.sleep()

