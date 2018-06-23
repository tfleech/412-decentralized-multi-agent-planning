#include "planner_node_core.cpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include <sstream>
#include "nav_msgs/Path.h"

using std::string;

int main(int argc, char **argv) {
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;


    Planner *planner = new Planner();


    ros::Publisher loc_goal_pub = n.advertise<geometry_msgs::PoseStamped>("intermediate_waypoint",1000);

    ros::Publisher new_target_pub = n.advertise<std_msgs::Bool>("new_target",1);

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("current_path",1);

    ros::Subscriber sub_grid = n.subscribe("/map", 1, &Planner::grid_cb, planner);

    ros::Subscriber sub_target = n.subscribe("target", 1000, &Planner::target_cb, planner);

    ros::Subscriber sub_my_loc = n.subscribe("pose", 1000, &Planner::my_loc_cb, planner);

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        planner->publish_loc_goal(&loc_goal_pub);
        planner->publish_path(&path_pub);
        planner->publish_new_target(&new_target_pub);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
