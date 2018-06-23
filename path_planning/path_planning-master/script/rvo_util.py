import math
from math import atan2, asin, cos, sin
from math import pi as PI
import numpy as np


def compute_vel_desired(loc, goal, vel_max, goal_th):
    """Compute desired velocity for each robot

    The desired velocities are the preferred speeds in the directions of the target locations. 
    Please note that the desired velocities don't consider other robots or obstacles, but simply
    their current locations and target locations.

    Please compute desired velocities (m/s) for all robots and return in this function.
    Please make sure that:
    1. Compute the desired speeds in the directions of the target locations.
    2. The maximum velocities must be used to ensure that the magnitudes of the desired velocities do not exceed the 
    maximum velocities.
    3. If a robot reached its goal (within the goal threshold (<= config.goal_th), its desired velocity is set to zero

    Inputs:
    - locs: Current locations of robots. Type: List of lists (e.g., [[3, 0], [0, -3]])
    - goals: Goal locations of robots. Type: List of lists (e.g., [[-3, 0], [0, 3]])
    - vels_max: Maximum velocities (m/s) specified for each robot. Type: List (e.g., [1.0, 1.0])
    - goal_th: Threshold for reaching a goal Type: Float (e.g., 0.1)

    Output:
    - vels_desired: List of lists that contains computed desired velocities for each robot (e.g., [[-0.9998333611064822, 0.0], [0.0, 0.9998333611064822]])
    """
    vel_desired = [] # Initialize

    dif_x = [goal[k] - loc[k] for k in range(2)]
    norm = eucl_dist(dif_x, [0, 0])
    norm_dif_x = [dif_x[k]*vel_max/norm for k in range(2)]
    vel_desired = norm_dif_x[:]

    if eucl_dist(loc, goal) <= goal_th:
        vel_desired[0] = 0
        vel_desired[1] = 0

    return vel_desired

def RVO_translation(loc_A, vel_A, vel_B):
    """Define translation for reciprocal velocity obstacle of obstacle B (i.e., robot B) to agent A.
    Please note that the translation translates the cone such that its apex is lies at 
    (vel_A + vel_B)*0.5 with respect to loc_A (current location of Robot A).

    Please look at section IV.A of the paper (http://gamma.cs.unc.edu/RVO/icra2008.pdf) 
    for more details.

    Inputs:
    - loc_A: Current location for robot A. Type: List (e.g., [3, 0])
    - vel_A: Current velocity for robot A. Type: List (e.g., [0.3, 0])
    - vel_B: Current velocity for robot B. Type: List (e.g., [0.5, 0])

    Output:
    - translated_cone: A list with two elements that translates the cone such that 
                       its apex lies at (vel_A + vel_B)*0.5 with respect to loc_A (e.g., [3.4, 0])
    """
    translated_cone = [loc_A[0] + 0.5*(vel_B[0] + vel_A[0]), 
                       loc_A[1] + 0.5*(vel_B[1] + vel_A[1])]

    return translated_cone

def compute_vel_obstacles(locs, vels_desired, vels, robot_radius, translation_func):
    """ Compute all velocity obstacles for all robots

    With the defined translation_func, the code additionally computes 
    bound left and right for the velocity obstacle.
    The code computes the velocity obstacles considering all robots.

    Inputs: 
    - locs: Current locations of robots. Type: List of lists (e.g., [[3, 0], [0, -3]])
    - vels_desired: List of lists that contains computed desired velocities for each robot (e.g., [[-0.9998333611064822, 0.0], [0.0, 0.9998333611064822]])
    - vels: Current velocities of robots. Type: List of lists (e.g., [[1, 0], [0, -1]])
    - robot_radius: Robot radius. Type: Float (e.g., 0.1)
    - mode: Either "VO" or "RVO". Type: String ("RVO")
    - translation_func: Translation function defined. Type: Python function

    Outputs:
    - vel_obstacles_for_all_robots: Contains velocity obstacle information (i.e., translation, boundary left, boundary right, 
                                    distance between robot A and B, and robot diameter) for all robots. 
                                    Information about velocity obstacles for robot 0 could be found by: `vel_obstacles_for_all_robots[0]`.
                                    Type: List of Lists of Lists
    """
    robot_radius += 0.1 # NOTE Required padding. Results in smoother trajectory
    vel_obstacles_for_all_robots = [] # Variable to return

    for i_robot in range(len(locs)):
        vel_A = [vels[i_robot][0], vels[i_robot][1]]
        loc_A = [locs[i_robot][0], locs[i_robot][1]]
        vel_obstacles = []

        # Compute RVO or VO for "dynamic" obstacles
        for j_robot in range(len(locs)):
            if i_robot != j_robot:
                vel_B = [vels[j_robot][0], vels[j_robot][1]]
                loc_B = [locs[j_robot][0], locs[j_robot][1]]

                # Calculate transl_vel_B_vel_A
                transl_vel_B_vel_A = translation_func(loc_A, vel_A, vel_B)

                # Compute boundary left and right for RVO or VO
                dist_BA = eucl_dist(loc_A, loc_B)
                if 2*robot_radius > dist_BA:
                    dist_BA = 2.*robot_radius
                theta_BA = atan2(loc_B[1] - loc_A[1], 
                                 loc_B[0] - loc_A[0])
                theta_BAort = asin(2.*robot_radius/float(dist_BA))

                theta_ort_left = theta_BA + theta_BAort
                bound_left = [cos(theta_ort_left), sin(theta_ort_left)]

                theta_ort_right = theta_BA - theta_BAort
                bound_right = [cos(theta_ort_right), sin(theta_ort_right)]

                # Add information
                velocity_obstacle = [transl_vel_B_vel_A, bound_left, bound_right, dist_BA, 2*robot_radius]
                vel_obstacles.append(velocity_obstacle)
                
        vel_obstacles_for_all_robots.append(vel_obstacles)

    return vel_obstacles_for_all_robots

def vel_update(loc, vel_desired, vel_obstacles):
    """ Update velocities such that robots paths are collision-free

    With the information about velocity obstacles, this function updates velocities
    such that robot paths upto next time step are collision-free.

    Inputs: 
    - locs: Current locations of robots. Type: List of lists (e.g., [[3, 0], [0, -3]])
    - vels_desired: List of lists that contains computed desired velocities for each robot (e.g., [[-0.9998333611064822, 0.0], [0.0, 0.9998333611064822]])
    - vels: Current velocities of robots. Type: List of lists (e.g., [[1, 0], [0, -1]])
    - vel_obstacles: Contains velocity obstacle information (i.e., translation, boundary left, boundary right, 
                     distance between robot A and B, and robot diameter) for all robots. 
                     Information about velocity obstacles for robot 0 could be found by: `vel_obstacles_for_all_robots[0]`.
                     Type: List of Lists of Lists
    """
    return intersect(loc, vel_desired, vel_obstacles)
    # vels_updated = list(vels) # NOTE Simply copy

    # for i_robot in range(len(locs)):
    #     vel_A = [vels[i_robot][0], vels[i_robot][1]]
    #     loc_A = [locs[i_robot][0], locs[i_robot][1]]

    #     # Based on computed RVO or VO, check whether current velocity intersects.
    #     # Then update accordingly
    #     vel_A_post = intersect(loc_A, vels_desired[i_robot], vel_obstacles[i_robot])
    #     vels_updated[i_robot] = vel_A_post[:]

    # return vels_updated

def intersect(loc_A, vel_desired, vel_obstacles):
    """ Check whether current velocity for robot A intersects (i.e., collision) with velocity obstacles.

    If there is collision, then update velocity such that it is collision-free.
    If current velocity is collision-free, then return with the velocity that is closest to 
    the desired velocity.

    Inputs: 
    - loc_A: Current location for robot A. Type: List (e.g., [3, 0])
    - vels_desired: Computed desired velocity for robot A. Type: List (e.g., [-0.9998333611064822, 0.0])
    - vel_obstacles: Contains velocity obstacle information (i.e., translation, boundary left, boundary right, 
                     distance between robot A and B, and robot diameter) for all robots. 
                     Type: List of Lists

    Outputs: Updated velocity for robot A. Type: List (e.g., [-0.9998333611064822, 0.0])
    """
    suitable_vels   = []
    unsuitable_vels = []

    # Serach for suitable and unsuitable velocities based on its location
    # Search through a certain radius and theta 
    norm_vel = eucl_dist(vel_desired, [0, 0])
    for theta in np.arange(0, 2*PI, 0.1):
        for rad in np.arange(0.02, norm_vel+0.02, norm_vel/5.0):
            new_v = [rad*cos(theta), rad*sin(theta)]
            suit = True
            for vel_obstacle in vel_obstacles:
                transl_vel_B_vel_A   = vel_obstacle[0]
                bound_left  = vel_obstacle[1]
                bound_right = vel_obstacle[2]

                diff = [new_v[0] + loc_A[0] - transl_vel_B_vel_A[0], 
                        new_v[1] + loc_A[1] - transl_vel_B_vel_A[1]]
                theta_diff  = atan2(diff[1], diff[0])
                theta_right = atan2(bound_right[1], bound_right[0])
                theta_left  = atan2(bound_left[1], bound_left[0])

                if in_between(theta_right, theta_diff, theta_left):
                    suit = False
                    break
            if suit:
                suitable_vels.append(new_v)
            else:
                unsuitable_vels.append(new_v)                

    # Check whether the desired velocity intersects with velocity obstacles
    new_v = vel_desired[:]
    suit = True
    for vel_obstacle in vel_obstacles:                
        transl_vel_B_vel_A = vel_obstacle[0]
        bound_left         = vel_obstacle[1]
        bound_right        = vel_obstacle[2]

        diff = [new_v[0] + loc_A[0] - transl_vel_B_vel_A[0], 
                new_v[1] + loc_A[1] - transl_vel_B_vel_A[1]]
        theta_diff  = atan2(diff[1], diff[0])
        theta_right = atan2(bound_right[1], bound_right[0])
        theta_left  = atan2(bound_left[1], bound_left[0])

        if in_between(theta_right, theta_diff, theta_left):
            suit = False
            break
    if suit:
        suitable_vels.append(new_v)
    else:
        unsuitable_vels.append(new_v)

    # If there exists suitable velocity then return the closest velocity to the desired vel
    if suitable_vels:
        vel_A_post = min(suitable_vels, key=lambda v: eucl_dist(v, vel_desired))

    # If no suitable velocity is found, then update velocity
    else:
        tc_V = dict()
        for unsuit_vel in unsuitable_vels:
            tc_V[tuple(unsuit_vel)] = 0.
            tc = []

            for vel_obstacle in vel_obstacles:
                transl_vel_B_vel_A = vel_obstacle[0]
                bound_left         = vel_obstacle[1]
                bound_right        = vel_obstacle[2]
                dist_BA            = vel_obstacle[3]
                big_robot_radius   = vel_obstacle[4]
                diff = [unsuit_vel[0] + loc_A[0] - transl_vel_B_vel_A[0], 
                        unsuit_vel[1] + loc_A[1] - transl_vel_B_vel_A[1]]
                theta_diff  = atan2(diff[1], diff[0])
                theta_right = atan2(bound_right[1], bound_right[0])
                theta_left  = atan2(bound_left[1], bound_left[0])

                if in_between(theta_right, theta_diff, theta_left):
                    small_theta = abs(theta_diff - 0.5*(theta_left + theta_right))
                    if abs(dist_BA*sin(small_theta)) >= big_robot_radius:
                        big_robot_radius = abs(dist_BA*sin(small_theta))
                    big_theta = asin(abs(dist_BA*sin(small_theta))/big_robot_radius)
                    dist_tg = abs(dist_BA*cos(small_theta)) - abs(big_robot_radius*cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0                    
                    tc_v = dist_tg/eucl_dist(diff, [0, 0])
                    tc.append(tc_v)
            tc_V[tuple(unsuit_vel)] = min(tc) + 0.001
        WT = 0.2
        vel_A_post = min(unsuitable_vels, key=lambda v: ((WT/tc_V[tuple(v)]) + eucl_dist(v, vel_desired)))

    return vel_A_post 

def in_between(theta_right, theta_diff, theta_left):
    """ Check whether current velocity theta for robot A intersects (i.e., collision) with velocity obstacle,
    defined by theta_right and theta_left.

    Inputs: 
    - theta_right: Velocity obstacle boundary theta right. Type: Float (e.g., -1.0). Unit in rad.
    - theta_diff: Current velocity theta. Type: Float (e.g., -1.0). Unit in rad.
    - theta_left: Velocity obstacle boundary theta left. Type: Float (e.g., -1.0). Unit in rad.

    Outputs: 
    - Bool: Return True if current velocity intersects with velocity obstacle. Else return False
    """
    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_diff <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left < 0) and (theta_right > 0):
            theta_left += 2*PI
            if theta_diff < 0:
                theta_diff += 2*PI
            if theta_right <= theta_diff <= theta_left:
                return True
            else:
                return False

        if (theta_left > 0) and (theta_right < 0):
            theta_right += 2*PI
            if theta_diff < 0:
                theta_diff += 2*PI
            if theta_left <= theta_diff <= theta_right:
                return True
            else:
                return False

def eucl_dist(loc_A, loc_B):
    """ Euclidean distance between two poses

    Note that poses have to be in 2D. If not, throws an error
    Epsilon is added to avoid zero dist value


    Inputs:
    - loc_A: Location of robot A. Type: List (e.g., [-3, 0])
    - loc_B: Location of robot B. Type: List (e.g., [3, 0])

    Outputs:
    - distance: Euclidiean distance between the two locations (unit: m). E.g., 6.00
    """
    if len(loc_A) != 2 or len(loc_B) != 2:
        raise RuntimeError("Error. The variable has to be a list with a length of 2 (x, y)")

    epsilon = 0.001 # To prevent zero distance value
    return math.sqrt((loc_A[0] - loc_B[0])**2 + (loc_A[1] - loc_B[1])**2) + epsilon

def is_reached_goal(locs, goals, config):
    """Check whether all robots reached their goals

    This function checks whether all robots reached their respective goals.
    A robot reached a goal if a distance from its current location to its goal 
    is smaller than a given threshold.

    Inputs:
    - locs: Current locations of robots. Type: List of lists (e.g., [[3, 0], [0, -3]])
    - goals: Goal locations of robots. Type: List of lists (e.g., [[-3, 0], [0, 3]])
    - config: Configuration of an environement. Type: Config class (defined in src/rvo/Config.py)

    Outputs:
    - Bool: Return True if all robots reached their goals. Else return False
    """
    reached_goal_counter = 0
    for i_robot in range(len(locs)):
        diff = np.linalg.norm(np.array(locs[i_robot]) - np.array(goals[i_robot]))

        if diff < config.goal_th:
            reached_goal_counter += 1

    if config.num_robot == reached_goal_counter:
        return True
    else:
        return False

def init_traj(locs):
    """Initialize trajectories

    This function initializes trajectories for all robots.
    `trajs` is only used for visualization purpose.

    Inputs:
    - locs: Current locations of robots. Type: List of lists (e.g., [[3, 0], [0, -3]])

    Outputs:
    - trajs: List of lists of lists that contain trajectories of all robots (e.g., [[[3], [0]], [[0], [-3]]])
    """
    trajs = []
    for ind in range(len(locs)):
        trajs.append([[locs[ind][0]], [locs[ind][1]]])

    return trajs

def compute_total_distance(trajs):
    """Compute total distance travelved for all robots

    Input:
    - trajs: List of lists of lists that contain trajectories of all robots (e.g., [[[3], [0]], [[0], [-3]]])

    Output:
    - total_dist: Total distance traveled by all robots. Type: Float (16.32). Unit in meter
    """
    total_dist = 0

    for i_robot in range(len(trajs)):
        for i_loc in range(len(trajs[i_robot][0])):
            if i_loc + 1 < len(trajs[i_robot][0]):
                loc_A = [trajs[i_robot][0][i_loc], 
                         trajs[i_robot][1][i_loc]]
                loc_B = [trajs[i_robot][0][i_loc+1], 
                         trajs[i_robot][1][i_loc+1]]
                total_dist += eucl_dist(loc_A, loc_B)

    return total_dist
