#include <global_planner/rrt.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <random>
#include <iostream>
#include <ctime>
#include <stdlib.h>
namespace global_planner
{
/**
 * @brief  Get a path using Rapidly-exploring random tree
 * @note   
 * @param  *potential: 
 * @param  *costs: 
 * @param  start_x: 
 * @param  start_y: 
 * @param  end_x: 
 * @param  end_y: 
 * @param  &path: 
 * @retval Find a path or not
 */
bool rrtPlannerC::getPath(float *potential, unsigned char *costs, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float>> &path)
{
    ros::NodeHandle nh("~/rrt");
    nh.param("/move_base/GlobalPlanner/step", step, 4.0);
    nh.param("/move_base/GlobalPlanner/k", k, 0.5);
    nh.param("/move_base/GlobalPlanner/smooth", smooth, false);
    nh.param("/move_base/GlobalPlanner/use_potential", use_potential, false);
    ROS_INFO("step = %f, k(for potential) = %f", step, k);
    ROS_INFO(use_potential ? "Using potential to get random point" : "No potential, origin path");
    ROS_INFO(smooth ? "Using smooth function, points well be decreased" : "No smooth, origin path");
    ros::Time start_time = ros::Time::now();
    ns_ = xs_ * ys_; //map size
    std::fill(potential, potential + ns_, POT_HIGH);
    std::vector<rrt_point> search_tree; //the RRT-tree
    rrt_point goal_point, start_point;
    size_t cycle = 0;
    goal_point.x = end_x;
    goal_point.y = end_y;
    start_point.x = start_x;
    start_point.y = start_y;
    start_point.parent = -1;
    int ran;                                                      //the first point
    search_tree.push_back(start_point);                           //add point to the RRT-tree
    rrt_point random_point, nearest_point, new_point, back_point; //define point for updating usage
    while (true)
    {
        cycle++;
        random_point = getRandomPoint();

        int nearest = getNearestPoint(search_tree, random_point);

        nearest_point = search_tree[nearest];
        if (distance(nearest_point, random_point) < step)
            continue;
        else
        {
            if (use_potential)
            {
                ran = rand() % 2;
                if (ran == 0)
                    new_point = getNewPointPlus(random_point, nearest_point, goal_point, nearest, step, k);
                else
                    new_point = getNewPoint(random_point, nearest_point, nearest, step);
            }
            else
            {
                new_point = getNewPoint(random_point, nearest_point, nearest, step);
            }
            if (costs[getIndex(new_point.x, new_point.y)] > 0)
                continue;

            search_tree.push_back(new_point);
            potential[getIndex(new_point.x, new_point.y)] = 255;
            if (distance(goal_point, new_point) < step)
                break;
        }
    }
    //the path has been planned, now it's time to get the path from the RRT-tree
    back_point = new_point;
    goal_point.parent = search_tree.size() - 1;
    search_tree.push_back(goal_point);
    std::pair<float, float> temp;
    while (back_point.parent != -1)
    {
        temp.first = back_point.x;
        temp.second = back_point.y;
        path.push_back(temp);
        back_point = search_tree[back_point.parent];
    }
    bool a = checkPathOK(path.at(0), path.at(path.size() - 1), costs);
    //ROS_INFO(a ? "No wall" : "there is a wall");
    if (smooth)
        optimal(path, costs, 0);
    ros::Duration delta_t = ros::Time::now() - start_time;
    double delta_t_sec = delta_t.toSec();
    double distance;
    if (smooth)
    {
        distance = calDistance(path);
    }
    else
    {
        distance = (path.size() - 1) * step;
    }
    ROS_INFO("Made a plan successfully! Use %f seconds. Cycled %d times", delta_t_sec, cycle);
    ROS_INFO("There is %d point in the tree and %d point in the path ", search_tree.size(), path.size());
    ROS_INFO("Total path distance is %f", distance);
    return true;
}

} // namespace global_planner