#include "planner_node_core.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include <costmap_2d/costmap_2d.h>
#include "global_planner/planner_core.h"
#include <vector>
#include "std_msgs/Bool.h"
#include <cmath>
#include <math.h>

Planner::Planner() {
	this->loc_goal_x = 0;
	this->loc_goal_y = 0;
	this->target_x = 0;
	this->target_y = 0;
    this->target_received = false;
}

Planner::~Planner() {}

void Planner::grid_cb(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
	double res = (double)(grid->info.resolution);
	int width = grid->info.width;
	int height = grid->info.height;
	double origin_x = (double)(grid->info.origin.position.x);
	double origin_y = (double)(grid->info.origin.position.y);
	
	std::lock_guard<std::mutex> guard(map_mutex);
    this->cost_map = new costmap_2d::Costmap2D(width, height, res, origin_x, origin_y);
	for (int i=0; i<width; i++){
		for (int j=0; j<height; j++){
			this->cost_map->setCost(i, j, grid->data[i*width + j]);
		}
	}

}

void Planner::target_cb(const geometry_msgs::Point32::ConstPtr& target) {
	this->target_x = target->x;
	this->target_y = target->y;
    this->target_received = true;

    this->compute_waypoint();
}

void Planner::my_loc_cb(const geometry_msgs::PoseStamped::ConstPtr& loc) {
	this->my_loc_x = loc->pose.position.x;
	this->my_loc_y = loc->pose.position.y;

   // this->compute_waypoint();
}

void Planner::publish_loc_goal(ros::Publisher *pub_message) {

    if (this->target_received){

            geometry_msgs::PoseStamped loc_goal;
            loc_goal.header.frame_id = "map";
            loc_goal.pose.position.x = this->loc_goal_x;
            loc_goal.pose.position.y = this->loc_goal_y;
            loc_goal.pose.position.z = 0;
            pub_message->publish(loc_goal);
    }
};

void Planner::publish_new_target(ros::Publisher *pub_message) {
	std_msgs::Bool msg;
	msg.data = reached_target();
	pub_message->publish(msg);
};

void Planner::publish_path(ros::Publisher *pub_message) {

    if(this->target_received){
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.poses = this->current_path;
        pub_message->publish(path);
    }
};

bool Planner::reached_target() {
	double dist = 0;
	dist = std::sqrt(std::pow(this->my_loc_x - this->target_x, 2) + std::pow(this->my_loc_y - this->target_y, 2));
	if (dist <= .1) {
		return(true);
	}
	return(false);
};


void Planner::compute_waypoint() {

	std::lock_guard<std::mutex> guard(map_mutex);
    global_planner::GlobalPlanner *GP = new global_planner::GlobalPlanner();	
    GP->initialize("global_planner", this->cost_map, "map");
	//MAKE PATH(new_map, this->my_loc, this->target)
	std::vector<geometry_msgs::PoseStamped> Path;
	
    geometry_msgs::PoseStamped start_point;
	start_point.header.frame_id = "map";
	start_point.pose.position.x = this->my_loc_x;
	start_point.pose.position.y = this->my_loc_y;

	geometry_msgs::PoseStamped goal_point;
    goal_point.header.frame_id = "map";
	goal_point.pose.position.x = this->target_x;
	goal_point.pose.position.y = this->target_y;

	GP->makePlan(start_point, goal_point, Path);
	this->current_path = Path;

	if (Path.size()>look_ahead) {
		this->loc_goal_x = Path[Path.size()/2].pose.position.x;
		this->loc_goal_y = Path[Path.size()/2].pose.position.y;
	} else {
		this->loc_goal_x = this->target_x;
		this->loc_goal_y = this->target_y;
	}
};
