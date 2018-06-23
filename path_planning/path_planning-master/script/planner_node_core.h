#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/OccupancyGrid.h"
#include <costmap_2d/costmap_2d.h>
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <mutex>
#include <thread>

#define look_ahead 1

class Planner {

std::mutex map_mutex;

public:
	Planner();
	~Planner();

	void publish_loc_goal(ros::Publisher *pub_message);
	void publish_new_target(ros::Publisher *pub_message);
	void publish_path(ros::Publisher *pub_message);
	void target_cb(const geometry_msgs::Point32::ConstPtr& target);
	void grid_cb(const nav_msgs::OccupancyGrid::ConstPtr& grid);
	void my_loc_cb(const geometry_msgs::PoseStamped::ConstPtr& loc);
    void compute_waypoint();

private:
	double my_loc_x;
	double my_loc_y;
	double loc_goal_x;
	double loc_goal_y;
	double target_x;
	double target_y;
	bool target_received;
	
    std::vector<geometry_msgs::PoseStamped> current_path;

    nav_msgs::OccupancyGrid *occ_grid;
    costmap_2d::Costmap2D *cost_map;

	bool reached_target();

};
